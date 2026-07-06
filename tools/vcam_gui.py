#!/usr/bin/env python3
"""GTK3 GUI: live view of the rendering node + realtime virtual-cam control.

Video:   rosimagesrc (ros_gst_bridge) -> videoconvert -> gtksink, embedded.
Control: pure WebSocket client of tools/vcam_ws_bridge.py — this GUI is the
         reference third-party client; it has no direct ROS dependency.

Interaction (mirrors the pygame prototype in tpsprojector/app.py):
  - preset buttons 1-5      eased preset switch (via the node's tween)
  - left-drag on the video  orbit: azimuth/elevation around the robot origin
                            (fixed target [0,0,0.3], eye sphere centred z=+0.5
                            — exactly the prototype's orbit_shot geometry)
  - scroll wheel            dolly distance in/out
az/el/dist persist across preset switches; grabbing the view jumps back to
the stored orbit pose — the prototype's [o]-toggle behaviour.

Run (ROS sourced so the gst plugin's ROS node can join the graph):
    python3 tools/vcam_gui.py [--ws ws://localhost:8765] [--topic /rendering/image]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import queue
import threading

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst, Gtk  # noqa: E402

PRESETS = ["config", "reverse_follow", "left_side", "right_side", "top_down"]
EL_MIN, EL_MAX = math.radians(5.0), math.radians(85.0)   # prototype clamps
DIST_MIN = 1.5
DRAG_GAIN = 0.006      # rad per pixel
SCROLL_GAIN = 0.3      # m per wheel notch

# Prototype orbit geometry (app.py orbit_shot): the robot at the origin is the
# orbit centre — eye moves on a sphere centred at z=+0.5, target is fixed.
ORBIT_TARGET = [0.0, 0.0, 0.3]
ORBIT_Z_OFFSET = 0.5


def orbit_eye(az: float, el: float, dist: float) -> list[float]:
    """Prototype's orbit_shot(): spherical (az, el, dist) about the origin -> eye."""
    return [dist * math.cos(el) * math.cos(az),
            dist * math.cos(el) * math.sin(az),
            dist * math.sin(el) + ORBIT_Z_OFFSET]


class WsClient(threading.Thread):
    """Background websocket client; auto-reconnects. Thread-safe send()."""

    def __init__(self, url: str, on_state, on_conn):
        super().__init__(daemon=True)
        self.url = url
        self.on_state = on_state          # called with the state dict (any thread)
        self.on_conn = on_conn            # called with bool connected (any thread)
        self._q: queue.Queue = queue.Queue()

    def send(self, obj: dict):
        self._q.put(obj)

    def run(self):
        asyncio.run(self._main())

    async def _main(self):
        import websockets
        while True:
            try:
                async with websockets.connect(self.url) as ws:
                    self.on_conn(True)
                    sender = asyncio.ensure_future(self._sender(ws))
                    try:
                        async for text in ws:
                            msg = json.loads(text)
                            if msg.get("type") == "state":
                                self.on_state(msg)
                    finally:
                        sender.cancel()
            except (OSError, Exception):  # ponytail: reconnect on anything
                pass
            self.on_conn(False)
            await asyncio.sleep(1.0)

    async def _sender(self, ws):
        loop = asyncio.get_running_loop()
        while True:
            obj = await loop.run_in_executor(None, self._q.get)
            # coalesce a burst of drag events — only the newest pose matters
            while obj.get("cmd") == "set_look" and not self._q.empty():
                try:
                    nxt = self._q.get_nowait()
                except queue.Empty:
                    break
                if nxt.get("cmd") == "set_look":
                    obj = nxt
                else:
                    await ws.send(json.dumps(obj))
                    obj = nxt
            await ws.send(json.dumps(obj))


class VcamWindow(Gtk.Window):
    def __init__(self, ws_url: str, topic: str):
        super().__init__(title="TPSProjector — virtual cam")
        self.set_default_size(1000, 640)
        self.connect("destroy", self._quit)

        # latest telemetry + orbit state (prototype defaults)
        self._state: dict | None = None
        self._az, self._el, self._dist = math.radians(180.0), math.radians(28.0), 4.5
        self._drag_xy: tuple[float, float] | None = None

        # ── gstreamer video ──────────────────────────────────────────────────
        self._pipeline = Gst.parse_launch(
            f"rosimagesrc ros-topic={topic} ! videoconvert ! gtksink name=sink sync=false")
        video = self._pipeline.get_by_name("sink").props.widget
        video.set_size_request(640, 360)

        ebox = Gtk.EventBox()
        ebox.add(video)
        ebox.connect("button-press-event", self._on_press)
        ebox.connect("button-release-event", self._on_release)
        ebox.connect("motion-notify-event", self._on_motion)
        ebox.connect("scroll-event", self._on_scroll)

        # ── controls ─────────────────────────────────────────────────────────
        btns = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        btns.set_margin_top(6)
        btns.set_margin_bottom(6)
        btns.set_margin_start(6)
        for i, name in enumerate(PRESETS, start=1):
            b = Gtk.Button(label=f"{i} {name}")
            b.connect("clicked", self._on_preset, i)
            btns.pack_start(b, False, False, 0)

        self._status = Gtk.Label(label="connecting…", xalign=0.0)
        self._status.set_margin_start(8)
        self._status.set_margin_bottom(4)

        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        vbox.pack_start(ebox, True, True, 0)
        vbox.pack_start(btns, False, False, 0)
        vbox.pack_start(self._status, False, False, 0)
        self.add(vbox)

        # ── websocket client ─────────────────────────────────────────────────
        self._ws = WsClient(
            ws_url,
            on_state=lambda s: GLib.idle_add(self._apply_state, s),
            on_conn=lambda ok: GLib.idle_add(self._apply_conn, ok))
        self._ws.start()
        self._connected = False

        self._pipeline.set_state(Gst.State.PLAYING)

    # ── telemetry → UI ─────────────────────────────────────────────────────────
    def _apply_state(self, s: dict):
        self._state = s
        p = s.get("preset", 0)
        name = PRESETS[p - 1] if 1 <= p <= len(PRESETS) else "free look"
        eye, tgt = s["eye"], s["target"]
        self._status.set_text(
            f"{'●' if self._connected else '○'} {name}   "
            f"eye ({eye[0]:+.2f}, {eye[1]:+.2f}, {eye[2]:+.2f})   "
            f"target ({tgt[0]:+.2f}, {tgt[1]:+.2f}, {tgt[2]:+.2f})")
        return False

    def _apply_conn(self, ok: bool):
        self._connected = ok
        if not ok:
            self._status.set_text("○ bridge disconnected — retrying…")
        return False

    # ── orbit interaction ──────────────────────────────────────────────────────
    def _send_look(self):
        self._ws.send({"cmd": "set_look",
                       "eye": orbit_eye(self._az, self._el, self._dist),
                       "target": ORBIT_TARGET})

    def _on_press(self, _w, ev):
        if ev.button == 1:
            self._drag_xy = (ev.x, ev.y)
            self._send_look()  # jump to the stored orbit pose, like the prototype's [o]
        return True

    def _on_release(self, _w, ev):
        if ev.button == 1:
            self._drag_xy = None
        return True

    def _on_motion(self, _w, ev):
        if self._drag_xy is None:
            return False
        dx, dy = ev.x - self._drag_xy[0], ev.y - self._drag_xy[1]
        self._drag_xy = (ev.x, ev.y)
        self._az -= dx * DRAG_GAIN
        self._el = max(EL_MIN, min(EL_MAX, self._el + dy * DRAG_GAIN))
        self._send_look()
        return True

    def _on_scroll(self, _w, ev):
        from gi.repository import Gdk
        if ev.direction == Gdk.ScrollDirection.UP:
            self._dist = max(DIST_MIN, self._dist - SCROLL_GAIN)
        elif ev.direction == Gdk.ScrollDirection.DOWN:
            self._dist += SCROLL_GAIN
        else:
            return False
        self._send_look()
        return True

    def _on_preset(self, _btn, preset: int):
        self._ws.send({"cmd": "set_preset", "preset": preset})

    def _quit(self, *_a):
        self._pipeline.set_state(Gst.State.NULL)
        Gtk.main_quit()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ws", default="ws://localhost:8765", help="vcam_ws_bridge URL")
    ap.add_argument("--topic", default="/rendering/image", help="rendered image topic")
    args = ap.parse_args()

    Gst.init(None)
    win = VcamWindow(args.ws, args.topic)
    win.show_all()
    Gtk.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
