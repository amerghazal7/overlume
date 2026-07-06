#!/usr/bin/env python3
"""WebSocket <-> ROS2 bridge for the rendering node's virtual camera.

Exposes a generic JSON control/telemetry API (see docs/superpowers/specs/
2026-07-06-vcam-gui-ws-bridge-design.md) so any app — the GTK GUI or a
third-party client — can drive the virtual camera:

  client -> server:
    {"cmd": "set_look", "eye": [x,y,z], "target": [x,y,z]}   (rig frame, m)
    {"cmd": "set_preset", "preset": 1..5}
  server -> client:
    {"type": "state", "eye": [...], "target": [...], "preset": 0..5}  (~15 Hz)
    {"type": "ack", "cmd": "set_preset", "success": bool, "active": str}
    {"type": "error", "message": str}

Run (ROS sourced + ros_apps install sourced for the SetVirtualCam type):
    python3 tools/vcam_ws_bridge.py [--host 0.0.0.0] [--port 8765]

ROS imports are deferred to main() so parse_cmd() stays unit-testable
without a sourced ROS environment.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import threading

STATE_HZ = 15.0
PRESET_RANGE = (1, 5)


def parse_cmd(text: str):
    """Validate one inbound JSON command.

    Returns ("set_look", (eye, target)) or ("set_preset", preset).
    Raises ValueError on anything malformed — the caller answers with an
    {"type":"error"} frame instead of touching ROS.
    """
    try:
        msg = json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(f"invalid JSON: {e}") from e
    if not isinstance(msg, dict):
        raise ValueError("command must be a JSON object")
    cmd = msg.get("cmd")
    if cmd == "set_look":
        eye, target = msg.get("eye"), msg.get("target")
        for name, v in (("eye", eye), ("target", target)):
            if not isinstance(v, (list, tuple)) or len(v) != 3 or \
                    not all(isinstance(x, (int, float)) for x in v):
                raise ValueError(f"set_look: {name} must be a list of 3 numbers")
        return "set_look", ([float(v) for v in eye], [float(v) for v in target])
    if cmd == "set_preset":
        p = msg.get("preset")
        if not isinstance(p, int) or isinstance(p, bool) or \
                not (PRESET_RANGE[0] <= p <= PRESET_RANGE[1]):
            raise ValueError(
                f"set_preset: preset must be an int {PRESET_RANGE[0]}..{PRESET_RANGE[1]}")
        return "set_preset", p
    raise ValueError(f"unknown cmd {cmd!r}")


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from micropilot_rendering_node.srv import SetVirtualCam
    import websockets

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()

    class BridgeNode(Node):
        def __init__(self):
            super().__init__("vcam_ws_bridge")
            self.state: list[float] | None = None  # [eye3, target3, preset]
            self._pub_look = self.create_publisher(
                Float64MultiArray, "/rendering_node/set_look", 10)
            self._cli = self.create_client(
                SetVirtualCam, "/rendering_node/set_virtual_cam")
            self.create_subscription(
                Float64MultiArray, "/rendering_node/vcam_state", self._on_state, 10)

        def _on_state(self, msg):
            self.state = list(msg.data)

        def set_look(self, eye, target):
            m = Float64MultiArray()
            m.data = [*eye, *target]
            self._pub_look.publish(m)

        def set_preset_async(self, preset: int):
            """Returns an rclpy Future, or None if the service is unavailable."""
            if not self._cli.service_is_ready():
                return None
            req = SetVirtualCam.Request()
            req.preset = preset
            return self._cli.call_async(req)

    rclpy.init()
    # rclpy.init() hooks SIGTERM but nothing here watches rclpy's shutdown flag,
    # which would leave the process unkillable except by SIGKILL — restore default.
    import signal
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    node = BridgeNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    clients: set = set()

    async def handle_client(ws):
        clients.add(ws)
        node.get_logger().info(f"client connected ({len(clients)} total)")
        try:
            async for text in ws:
                try:
                    cmd, payload = parse_cmd(text)
                except ValueError as e:
                    await ws.send(json.dumps({"type": "error", "message": str(e)}))
                    continue
                if cmd == "set_look":
                    node.set_look(*payload)
                else:  # set_preset
                    fut = node.set_preset_async(payload)
                    if fut is None:
                        await ws.send(json.dumps({
                            "type": "error",
                            "message": "set_virtual_cam service unavailable"}))
                        continue
                    loop = asyncio.get_running_loop()
                    afut = loop.create_future()
                    fut.add_done_callback(
                        lambda f: loop.call_soon_threadsafe(
                            lambda: afut.done() or afut.set_result(f.result())))
                    try:
                        res = await asyncio.wait_for(afut, timeout=5.0)
                        await ws.send(json.dumps({
                            "type": "ack", "cmd": "set_preset",
                            "success": res.success, "active": res.active}))
                    except asyncio.TimeoutError:
                        await ws.send(json.dumps({
                            "type": "error", "message": "set_virtual_cam timed out"}))
        finally:
            clients.discard(ws)
            node.get_logger().info(f"client disconnected ({len(clients)} total)")

    async def broadcast_state():
        last = None
        while True:
            await asyncio.sleep(1.0 / STATE_HZ)
            s = node.state
            if s is None or s == last or not clients:
                continue
            last = list(s)
            frame = json.dumps({
                "type": "state",
                "eye": s[0:3], "target": s[3:6],
                "preset": int(s[6]) if len(s) > 6 else 0})
            await asyncio.gather(
                *(ws.send(frame) for ws in list(clients)), return_exceptions=True)

    async def serve():
        async with websockets.serve(handle_client, args.host, args.port):
            node.get_logger().info(f"vcam WS bridge listening on ws://{args.host}:{args.port}")
            await broadcast_state()  # runs forever

    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
