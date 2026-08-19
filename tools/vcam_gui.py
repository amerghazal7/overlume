#!/usr/bin/env python3
"""GTK3 GUI: live view of the rendering node + realtime virtual-cam control.

Video:   rosimagesrc (ros_gst_bridge) -> videoconvert -> gtksink, embedded.
Control: pure WebSocket client of tools/vcam_ws_bridge.py — this GUI is the
         reference third-party client; it has no direct ROS dependency.

Interaction (mirrors the pygame prototype in tpsprojector/app.py):
  - preset buttons 1-5      eased preset switch (via the node's tween)
  - view cycle button       bowl -> pointcloud -> visual render mode
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
import collections
import json
import math
import queue
import threading

import numpy as np

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


# ── camera extrinsics <-> user-friendly pose ─────────────────────────────────
# A camera row is 12 floats [R(9 row-major)|t(3)], R columns = optical
# right/down/fwd in the rig frame (x-fwd, y-left, z-up). The panel edits it as
# x/y/z (m) + yaw/pitch/roll (deg): yaw = heading of the optical axis, pitch =
# its elevation, roll = rotation about it (0 = horizon level).
CAM_NAMES_6 = ["fl", "fm", "fr", "bl", "bm", "br"]
POSE_KEYS = ["x", "y", "z", "yaw", "pitch", "roll"]


def _roll_basis(fwd):
    r0 = np.cross(fwd, [0.0, 0.0, 1.0])
    n = np.linalg.norm(r0)
    if n < 1e-6:  # looking straight up/down
        r0, n = np.array([0.0, -1.0, 0.0]), 1.0
    r0 = r0 / n
    return r0, np.cross(fwd, r0)


def rt_to_pose(row):
    R = np.array(row[:9], float).reshape(3, 3)
    right, fwd = R[:, 0], R[:, 2]
    yaw = math.degrees(math.atan2(fwd[1], fwd[0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(fwd[2])))))
    r0, d0 = _roll_basis(fwd)
    roll = math.degrees(math.atan2(float(np.dot(right, d0)), float(np.dot(right, r0))))
    return [row[9], row[10], row[11], yaw, pitch, roll]


def pose_to_rt(x, y, z, yaw, pitch, roll):
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    fwd = np.array([cp * cy, cp * sy, sp])
    r0, d0 = _roll_basis(fwd)
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    right = cr * r0 + sr * d0
    down = np.cross(fwd, right)
    R = np.stack([right, down, fwd], axis=1)
    return [float(v) for v in R.reshape(-1)] + [float(x), float(y), float(z)]


# (label, lo, hi, step, digits) for the render-tunable scalars.
# Bounds are deliberately loose — they only guard against nonsense (negative
# radii), not taste; tune freely and judge by the render.
RENDER_SPINS = [
    ("bowl_R0", 0.1, 500.0, 0.5, 2),
    ("bowl_k", 0.0, 10.0, 0.01, 3),
    ("bowl_Rmax", 0.5, 1000.0, 1.0, 1),
    ("feather_margin", 0.0, 5000.0, 10.0, 0),
    ("virtual_vfov_deg", 5.0, 175.0, 1.0, 1),
    ("max_sync_latency", 0.0, 10.0, 0.01, 2),
    ("splat_radius", 0.0, 30.0, 1.0, 0),
]
RENDER_BOOLS = ["fill_blind_zone", "exposure_match"]

# A numeric tuning row: slider + value box sharing one Adjustment, plus
# editable min/max boxes that rewrite the slider's range on the fly.
Row = collections.namedtuple("Row", "val adj mn mx")


def set_row_value(row: Row, v: float):
    """Set a row's value, auto-expanding its slider range if v falls outside."""
    if v < row.adj.get_lower():
        row.adj.set_lower(v)
        row.mn.set_value(v)
    if v > row.adj.get_upper():
        row.adj.set_upper(v)
        row.mx.set_value(v)
    row.val.set_value(v)
POSE_SPINS = [  # (key, lo, hi, step, digits)
    ("x", -10.0, 10.0, 0.01, 3),
    ("y", -10.0, 10.0, 0.01, 3),
    ("z", -10.0, 10.0, 0.01, 3),
    ("yaw", -180.0, 180.0, 0.1, 2),
    ("pitch", -90.0, 90.0, 0.1, 2),
    ("roll", -180.0, 180.0, 0.1, 2),
]


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
                            self.on_state(json.loads(text))  # all frame types
                    finally:
                        sender.cancel()
            except (OSError, Exception):  # ponytail: reconnect on anything
                pass
            self.on_conn(False)
            await asyncio.sleep(1.0)

    async def _sender(self, ws):
        loop = asyncio.get_running_loop()

        def same_stream(a, b):
            # bursts where only the newest value matters
            if a.get("cmd") != b.get("cmd"):
                return False
            if a.get("cmd") == "set_look":
                return True
            return a.get("cmd") == "set_param" and a.get("name") == b.get("name")

        while True:
            obj = await loop.run_in_executor(None, self._q.get)
            while obj.get("cmd") in ("set_look", "set_param") and not self._q.empty():
                try:
                    nxt = self._q.get_nowait()
                except queue.Empty:
                    break
                if same_stream(obj, nxt):
                    obj = nxt
                else:
                    await ws.send(json.dumps(obj))
                    obj = nxt
            await ws.send(json.dumps(obj))


class VcamWindow(Gtk.Window):
    def __init__(self, ws_url: str, topic: str):
        super().__init__(title="TPSProjector — virtual cam")
        self.set_default_size(1500, 680)
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

        # view switch: bowl -> pointcloud -> visual cycle (label shows CURRENT
        # mode as reported by the node's telemetry; click sends the next one)
        self._render_mode = 2
        self._mode_btn = Gtk.Button(label="view: pointcloud")
        self._mode_btn.connect("clicked", self._on_mode_toggle)
        btns.pack_end(self._mode_btn, False, False, 6)

        # day/night theme toggle (Epic 1 Task 3 / VM-014, visual mode only).
        # Unlike _mode_btn above, there's no telemetry echo of the active
        # theme yet (that's Epic 3's ~/diagnostics, VM-034) -- the label just
        # optimistically flips on click, same as how preset buttons don't
        # wait for confirmation today. Starts on "dark_adas", the shipped
        # default (visualization_node's initial_theme).
        self._theme = "dark_adas"
        self._theme_btn = Gtk.Button(label="theme: dark_adas")
        self._theme_btn.connect("clicked", self._on_theme_toggle)
        btns.pack_end(self._theme_btn, False, False, 6)

        self._status = Gtk.Label(label="connecting…", xalign=0.0)
        self._status.set_margin_start(8)
        self._status.set_margin_bottom(4)

        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        vbox.pack_start(ebox, True, True, 0)
        vbox.pack_start(btns, False, False, 0)
        vbox.pack_start(self._status, False, False, 0)

        # ── live tuning panel (params from the node via the bridge) ──────────
        self._loading = False          # True while populating widgets from node
        self._params_loaded = False    # real values received from the node
        self._param_spins = {}
        self._param_switches = {}
        self._pose_spins = {}
        self._extrinsics: list[float] | None = None
        panel = self._build_panel()
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroll.add(panel)
        scroll.set_size_request(470, -1)

        outer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        outer.pack_start(vbox, True, True, 0)
        outer.pack_start(scroll, False, False, 0)
        self.add(outer)

        # ── websocket client ─────────────────────────────────────────────────
        self._ws = WsClient(
            ws_url,
            on_state=lambda s: GLib.idle_add(self._on_ws_msg, s),
            on_conn=lambda ok: GLib.idle_add(self._apply_conn, ok))
        self._ws.start()
        self._connected = False

        self._pipeline.set_state(Gst.State.PLAYING)

    # ── tuning panel ───────────────────────────────────────────────────────────
    def _build_panel(self) -> Gtk.Box:
        panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
        for m in ("set_margin_top", "set_margin_bottom", "set_margin_start",
                  "set_margin_end"):
            getattr(panel, m)(8)

        def section(title):
            lbl = Gtk.Label(xalign=0.0)
            lbl.set_markup(f"<b>{title}</b>")
            lbl.set_margin_top(6)
            panel.pack_start(lbl, False, False, 0)

        def spin_row(label, lo, hi, step, digits, cb, box=None):
            # label | min | ── slider ── | max | value. Slider + value share one
            # Adjustment; the min/max boxes rewrite the slider range live.
            outer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=3)
            l = Gtk.Label(label=label, xalign=0.0)
            l.set_size_request(105, -1)
            adj = Gtk.Adjustment(value=lo, lower=lo, upper=hi,
                                 step_increment=step, page_increment=step * 10)

            def bound_spin(v0):
                b = Gtk.SpinButton(adjustment=Gtk.Adjustment(
                    value=v0, lower=-1e9, upper=1e9, step_increment=step),
                    digits=digits)
                b.set_numeric(True)
                b.set_width_chars(5)
                return b

            mn, mx = bound_spin(lo), bound_spin(hi)
            mn.connect("value-changed",
                       lambda s: adj.set_lower(min(s.get_value(), adj.get_upper())))
            mx.connect("value-changed",
                       lambda s: adj.set_upper(max(s.get_value(), adj.get_lower())))
            scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL, adjustment=adj)
            scale.set_draw_value(False)
            scale.set_size_request(120, -1)
            val = Gtk.SpinButton(adjustment=adj, digits=digits)
            val.set_numeric(True)
            val.set_width_chars(7)
            val.connect("value-changed", cb)
            outer.pack_start(l, False, False, 0)
            outer.pack_start(mn, False, False, 0)
            outer.pack_start(scale, True, True, 0)
            outer.pack_start(mx, False, False, 0)
            outer.pack_start(val, False, False, 0)
            (box or panel).pack_start(outer, False, False, 0)
            return Row(val, adj, mn, mx)
        self._spin_row = spin_row

        section("Render")
        for name, lo, hi, step, digits in RENDER_SPINS:
            self._param_spins[name] = spin_row(
                name, lo, hi, step, digits,
                lambda sp, n=name: self._on_param_spin(n, sp))
        for name in RENDER_BOOLS:
            row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
            l = Gtk.Label(label=name, xalign=0.0)
            l.set_size_request(130, -1)
            sw = Gtk.Switch()
            sw.connect("notify::active", lambda s, _p, n=name: self._on_param_switch(n, s))
            row.pack_start(l, False, False, 0)
            row.pack_start(sw, False, False, 0)
            panel.pack_start(row, False, False, 0)
            self._param_switches[name] = sw

        section("Camera poses (calib)")
        # one collapsible block per camera, created when extrinsics arrive
        self._cam_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        panel.pack_start(self._cam_box, False, False, 0)

        section("Save")
        upd = Gtk.Button(label="Update node config file")
        upd.connect("clicked", lambda _b: self._ws.send({"cmd": "save_params"}))
        panel.pack_start(upd, False, False, 0)
        save_as = Gtk.Button(label="Save As…")
        save_as.connect("clicked", self._on_save_as)
        panel.pack_start(save_as, False, False, 0)
        refresh = Gtk.Button(label="Reload from node")
        refresh.connect("clicked", lambda _b: self._ws.send({"cmd": "get_params"}))
        panel.pack_start(refresh, False, False, 0)
        return panel

    def _on_param_spin(self, name, spin):
        if self._loading:
            return
        v = spin.get_value()  # the row's value SpinButton
        if name == "splat_radius":
            v = int(round(v))
        self._ws.send({"cmd": "set_param", "name": name, "value": v})

    def _on_param_switch(self, name, sw):
        if self._loading:
            return
        self._ws.send({"cmd": "set_param", "name": name, "value": bool(sw.get_active())})

    def _build_cam_expanders(self, n: int):
        names = CAM_NAMES_6 if n == 6 else [f"cam{i}" for i in range(n)]
        for ci, nm in enumerate(names):
            exp = Gtk.Expander(label=nm)
            inner = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
            inner.set_margin_start(12)
            exp.add(inner)
            spins = {}
            for key, lo, hi, step, digits in POSE_SPINS:
                unit = "m" if key in ("x", "y", "z") else "deg"
                spins[key] = self._spin_row(
                    f"{key} ({unit})", lo, hi, step, digits,
                    lambda sp, c=ci, k=key: self._on_pose_spin(c, k), box=inner)
            self._pose_spins[ci] = spins
            self._cam_box.pack_start(exp, False, False, 0)
        self._cam_box.show_all()

    def _refresh_pose_spins(self):
        if not self._extrinsics:
            return
        was = self._loading
        self._loading = True
        for ci, spins in self._pose_spins.items():
            pose = rt_to_pose(self._extrinsics[ci * 12:(ci + 1) * 12])
            for k, v in zip(POSE_KEYS, pose):
                set_row_value(spins[k], v)
        self._loading = was

    def _on_pose_spin(self, ci, _key):
        if self._loading or not self._extrinsics:
            return
        pose = [self._pose_spins[ci][k].val.get_value() for k in POSE_KEYS]
        self._extrinsics[ci * 12:(ci + 1) * 12] = pose_to_rt(*pose)
        self._ws.send({"cmd": "set_param", "name": "camera_extrinsics",
                       "value": self._extrinsics})

    def _apply_params(self, values: dict):
        if not any(v is not None for v in values.values()):
            return False  # node not configured yet — retry timer keeps polling
        self._params_loaded = True
        self._loading = True
        try:
            for name, row in self._param_spins.items():
                v = values.get(name)
                if v is not None:
                    set_row_value(row, float(v))
            for name, sw in self._param_switches.items():
                v = values.get(name)
                if v is not None:
                    sw.set_active(bool(v))
            ext = values.get("camera_extrinsics")
            if ext:
                first = self._extrinsics is None
                self._extrinsics = list(ext)
                if first:
                    self._build_cam_expanders(len(ext) // 12)
        finally:
            self._loading = False
        self._refresh_pose_spins()
        return False

    def _apply_ack(self, msg: dict):
        if msg.get("cmd") == "save_params":
            ok = msg.get("success")
            self._status.set_text(
                ("✔ saved " if ok else "✘ save failed: ") + str(msg.get("path")))
        return False

    def _on_save_as(self, _btn):
        dlg = Gtk.FileChooserDialog(title="Save params as…", parent=self,
                                    action=Gtk.FileChooserAction.SAVE)
        dlg.add_buttons("Cancel", Gtk.ResponseType.CANCEL, "Save", Gtk.ResponseType.OK)
        dlg.set_do_overwrite_confirmation(True)
        dlg.set_current_name("tuned_params.yaml")
        if dlg.run() == Gtk.ResponseType.OK:
            self._ws.send({"cmd": "save_params", "path": dlg.get_filename()})
        dlg.destroy()

    # ── telemetry → UI ─────────────────────────────────────────────────────────
    def _apply_state(self, s: dict):
        self._state = s
        mode = s.get("render_mode", 2)
        if mode != self._render_mode:
            self._render_mode = mode
            self._mode_btn.set_label(
                {1: "view: bowl", 2: "view: pointcloud",
                 3: "view: visual"}.get(mode, f"view: {mode}"))
        p = s.get("preset", 0)
        name = PRESETS[p - 1] if 1 <= p <= len(PRESETS) else "free look"
        eye, tgt = s["eye"], s["target"]
        self._status.set_text(
            f"{'●' if self._connected else '○'} {name}   "
            f"eye ({eye[0]:+.2f}, {eye[1]:+.2f}, {eye[2]:+.2f})   "
            f"target ({tgt[0]:+.2f}, {tgt[1]:+.2f}, {tgt[2]:+.2f})")
        return False

    def _on_ws_msg(self, msg: dict):
        t = msg.get("type")
        if t == "state":
            return self._apply_state(msg)
        if t == "params":
            return self._apply_params(msg.get("values") or {})
        if t == "ack":
            return self._apply_ack(msg)
        if t == "error":
            self._status.set_text(f"✘ {msg.get('message')}")
        return False

    def _apply_conn(self, ok: bool):
        self._connected = ok
        if not ok:
            self._status.set_text("○ bridge disconnected — retrying…")
        else:
            # Populate the tuning panel; keep retrying until the node is
            # configured (its params only exist after the lifecycle transition).
            self._params_loaded = False
            self._ws.send({"cmd": "get_params"})
            GLib.timeout_add_seconds(2, self._retry_params)
        return False

    def _retry_params(self):
        if self._connected and not self._params_loaded:
            self._ws.send({"cmd": "get_params"})
            return True   # keep the timer running
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

    def _on_mode_toggle(self, _btn):
        target = {1: 2, 2: 3, 3: 1}.get(self._render_mode, 3)
        # Optimistic: telemetry echoes correct this when frames flow, but
        # without it a dead node freezes self._render_mode and every click
        # re-sends the same mode forever (review finding).
        self._render_mode = target
        self._ws.send({"cmd": "set_render_mode",
                       "mode": {1: "bowl", 2: "pointcloud", 3: "visual"}[target]})

    def _on_theme_toggle(self, _btn):
        self._theme = "light_clay" if self._theme == "dark_adas" else "dark_adas"
        self._theme_btn.set_label(f"theme: {self._theme}")
        self._ws.send({"cmd": "set_theme", "theme": self._theme})

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
