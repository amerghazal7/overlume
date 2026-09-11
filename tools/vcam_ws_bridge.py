#!/usr/bin/env python3
"""WebSocket <-> ROS2 bridge for the rendering node's virtual camera.

Exposes a generic JSON control/telemetry API (see docs/superpowers/specs/
2026-07-06-vcam-gui-ws-bridge-design.md) so any app — the GTK GUI or a
third-party client — can drive the virtual camera:

  client -> server:
    {"cmd": "set_look", "eye": [x,y,z], "target": [x,y,z]}   (rig frame, m)
    {"cmd": "set_preset", "preset": 1..5}
    {"cmd": "set_render_mode", "mode": "bowl" | "pointcloud"}  (also 1 | 2)
    {"cmd": "set_theme", "theme": "dark_adas" | "light_clay"}  (visual mode only)
    {"cmd": "get_params"}                                (tunable param values)
    {"cmd": "set_param", "name": str, "value": num|bool|[floats]}
    {"cmd": "save_params"}                  (update the node's launch config yaml)
    {"cmd": "save_params", "path": "/abs/new.yaml"}      (save as a new yaml)
    {"cmd": "set_layers", "layers": {name: bool, ...}}   (visual mode only,
        Epic 3 Task 5/VM-032 -- names from LAYER_NAMES below; applies live,
        no restart)
    {"cmd": "set_quality", "preset": "low"|"medium"|"high"|0|1|2}  (VM-032 --
        writes visualization_node's `quality` param only; takes effect on
        its NEXT restart, not live -- see that node's create_renderer())
    {"cmd": "set_surround_profile", "profile": "bowl"|"hybrid"}  (Task 4/
        VM-093 -- writes visualization_node's `surround_stitching_profile`
        param; live, same on_params() live-tuning contract as layer_*)
  server -> client:
    {"type": "state", "eye": [...], "target": [...], "preset": 0..5,
     "render_mode": 1|2}  (~15 Hz)
    {"type": "params", "values": {name: value, ...}}
    {"type": "ack", "cmd": "save_params", "success": bool, "path": str}
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
import re
import threading

STATE_HZ = 15.0
PRESET_RANGE = (1, 5)
RENDER_MODES = {"bowl": 1, "pointcloud": 2, "visual": 3, 1: 1, 2: 2, 3: 3}

# Epic 3 Task 5 (VM-032) + VM-077 + Task 4/VM-093: the layer_<name> bool
# params visualization_node declares (scene_assembly.hpp's live categories --
# trajectory_carpet added VM-077, surround_stitching added VM-093 (Surround
# Stitching, follow-up USER DIRECTIVE 2026-09-11) -- see that node's
# on_configure()). surround_stitching is the one entry here that doesn't gate
# a SceneAssembly category (it gates set_bowl_visible() instead); it's a
# plain layer_* bool param, same live-tuning contract as every other name
# here, so it belongs in the same set.
LAYER_NAMES = {
    "objects", "paths", "map_elements", "grids", "alerts", "markers", "point_clouds",
    "trajectory_carpet", "surround_stitching",
}
# quality preset name -> visualization_node's `quality` param encoding
# (0=low, 1=med, 2=high, api.h's RenderConfig::quality).
QUALITY_PRESETS = {"low": 0, "medium": 1, "high": 2, 0: 0, 1: 1, 2: 2}
# Surround Stitching content profile (Task 4/VM-093 follow-up USER
# DIRECTIVE) -- visualization_node's on_params() accepts exactly these two,
# rejecting anything else (test_mode_dispatch.py check 3).
SURROUND_PROFILES = {"bowl", "hybrid"}

# Params the GUI tuning panel may read/write, with their declared ROS types.
TUNABLE_PARAMS = {
    "bowl_R0": float, "bowl_k": float, "bowl_Rmax": float,
    "feather_margin": float, "max_sync_latency": float, "virtual_vfov_deg": float,
    "splat_radius": int, "fill_blind_zone": bool, "exposure_match": bool,
    "sky_color": list, "camera_extrinsics": list,
}


def patch_yaml_text(text: str, values: dict) -> str:
    """Update scalar and flat-list keys in a ROS params YAML, preserving all
    other lines (comments, ordering). Missing keys are appended under the
    first ros__parameters: block."""
    def fmt(v):
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, float):
            return f"{v:.6g}"
        return str(v)

    lines = text.splitlines(keepends=True)
    pending = dict(values)
    out = []
    i = 0
    while i < len(lines):
        m = re.match(r"^(\s*)([A-Za-z_]\w*):\s*(.*?)\s*(#.*)?$", lines[i])
        if m and m.group(2) in pending:
            ind, key = m.group(1), m.group(2)
            v = pending.pop(key)
            if isinstance(v, (list, tuple)):
                out.append(f"{ind}{key}:\n")
                i += 1
                while i < len(lines) and re.match(rf"^{re.escape(ind)}- ", lines[i]):
                    i += 1
                out.extend(f"{ind}- {fmt(float(x))}\n" for x in v)
                continue
            out.append(f"{ind}{key}: {fmt(v)}\n")
            i += 1
            continue
        out.append(lines[i])
        i += 1
    if pending:
        for idx, ln in enumerate(out):
            m = re.match(r"^(\s*)ros__parameters:\s*$", ln)
            if m:
                ind = m.group(1) + "  "
                ins = []
                for key, v in pending.items():
                    if isinstance(v, (list, tuple)):
                        ins.append(f"{ind}{key}:\n")
                        ins.extend(f"{ind}- {fmt(float(x))}\n" for x in v)
                    else:
                        ins.append(f"{ind}{key}: {fmt(v)}\n")
                out[idx + 1:idx + 1] = ins
                break
    return "".join(out)


def parse_cmd(text: str):
    """Validate one inbound JSON command.

    Returns ("set_look", (eye, target)), ("set_preset", preset) or
    ("set_render_mode", mode 1|2).
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
    if cmd == "set_render_mode":
        m = msg.get("mode")
        if isinstance(m, bool) or m not in RENDER_MODES:
            raise ValueError(
                'set_render_mode: mode must be "bowl", "pointcloud", "visual", 1, 2 or 3')
        return "set_render_mode", RENDER_MODES[m]
    if cmd == "set_theme":
        theme = msg.get("theme")
        if not isinstance(theme, str) or not theme:
            raise ValueError("set_theme: theme must be a non-empty string")
        return "set_theme", theme
    if cmd == "get_params":
        return "get_params", None
    if cmd == "set_param":
        name = msg.get("name")
        if name not in TUNABLE_PARAMS:
            raise ValueError(f"set_param: unknown/untunable param {name!r}")
        want = TUNABLE_PARAMS[name]
        v = msg.get("value")
        if want is bool:
            if not isinstance(v, bool):
                raise ValueError(f"set_param: {name} expects a bool")
        elif want is int:
            if isinstance(v, bool) or not isinstance(v, int):
                raise ValueError(f"set_param: {name} expects an int")
        elif want is float:
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ValueError(f"set_param: {name} expects a number")
            v = float(v)
        else:  # list of numbers
            if not isinstance(v, (list, tuple)) or not v or \
                    not all(isinstance(x, (int, float)) and not isinstance(x, bool)
                            for x in v):
                raise ValueError(f"set_param: {name} expects a list of numbers")
            v = [float(x) for x in v]
        return "set_param", (name, v)
    if cmd == "save_params":
        path = msg.get("path")
        if path is not None and (not isinstance(path, str) or not path):
            raise ValueError("save_params: path must be a non-empty string")
        return "save_params", path
    if cmd == "set_layers":
        layers = msg.get("layers")
        if not isinstance(layers, dict) or not layers:
            raise ValueError("set_layers: layers must be a non-empty object")
        out = {}
        for name, v in layers.items():
            if name not in LAYER_NAMES:
                raise ValueError(f"set_layers: unknown layer {name!r}")
            if not isinstance(v, bool):
                raise ValueError(f"set_layers: {name} must be a bool")
            out[name] = v
        return "set_layers", out
    if cmd == "set_quality":
        preset = msg.get("preset")
        if isinstance(preset, bool) or preset not in QUALITY_PRESETS:
            raise ValueError(
                'set_quality: preset must be "low", "medium", "high", or 0/1/2')
        return "set_quality", QUALITY_PRESETS[preset]
    if cmd == "set_surround_profile":
        profile = msg.get("profile")
        if profile not in SURROUND_PROFILES:
            raise ValueError('set_surround_profile: profile must be "bowl" or "hybrid"')
        return "set_surround_profile", profile
    raise ValueError(f"unknown cmd {cmd!r}")


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64MultiArray, Int32, String
    from diagnostic_msgs.msg import DiagnosticArray
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from micropilot_rendering_node.srv import SetVirtualCam
    import websockets

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()

    # Both the CUDA node (modes 1-2) and the visualization node (mode 3)
    # implement an identical vcam surface under their own namespaces (spec
    # §6) — camera commands fan out to BOTH; the inactive one just updates
    # state, so a client orbiting in one mode keeps its viewpoint after
    # switching modes.
    VCAM_NAMESPACES = ["/rendering_node", "/visualization_node"]

    class BridgeNode(Node):
        def __init__(self):
            super().__init__("vcam_ws_bridge")
            self.state: list[float] | None = None  # [eye3, target3, preset, mode]
            # Both nodes publish ~/vcam_state continuously (spec §9), so
            # picking "whichever arrived last" flickers between them. Index 7
            # is render_mode_ on BOTH namespaces as of VM-093 (rendering_node's
            # own 1|2, visualization_node's own local 1|2|3 -- see that node's
            # scene_assembly.hpp RenderMode) -- that parity is exactly what
            # VM-093 bought, but it still can't be a shared filter value
            # (rendering_node has no mode 3) -- instead track which NAMESPACE
            # is authoritative for the last commanded mode. Starts at
            # rendering_node, matching its default render_mode_ (2).
            self._active_ns = VCAM_NAMESPACES[0]
            # diagnostics only exists on visualization_node (mode 3) -- no
            # mux needed, harmless if it keeps arriving while mode 1/2 is
            # active, same "ingest continues regardless of mode" philosophy
            # vcam_state already follows. Display-only: reuses this same
            # telemetry pipe rather than opening a second WS transport.
            self.diagnostics: dict | None = None
            self.create_subscription(
                DiagnosticArray, "/visualization_node/diagnostics",
                self._on_diagnostics, 10)
            self._pub_look = [
                self.create_publisher(Float64MultiArray, f"{ns}/set_look", 10)
                for ns in VCAM_NAMESPACES]
            # Global mux topic: 1|2|3, shared by both nodes -- switches which
            # one owns /rendering/image AND (for 1|2) the CUDA node's
            # bowl/pointcloud view, same semantics as the old per-node
            # "~/set_render_mode" it replaces here. transient_local + reliable,
            # depth 1 (VM-037 Step (a)): both nodes' subscriptions are now the
            # same durable QoS, and a VOLATILE publisher is QoS-INCOMPATIBLE
            # with a transient_local subscription (DDS won't match them at
            # all), so this publisher must match or nothing gets delivered.
            self._pub_mode = self.create_publisher(
                Int32, "/rendering/set_mode",
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1))
            # node-private, mode-3-only concept -- no mux needed, harmless if
            # published while mode 1/2 is active (same "ingest continues
            # regardless of mode" philosophy as /rendering/set_mode above).
            self._pub_theme = self.create_publisher(
                String, "/visualization_node/set_theme", 10)
            self._cli = [
                self.create_client(SetVirtualCam, f"{ns}/set_virtual_cam")
                for ns in VCAM_NAMESPACES]
            self._cli_getp = self.create_client(
                GetParameters, "/rendering_node/get_parameters")
            self._cli_setp = self.create_client(
                SetParameters, "/rendering_node/set_parameters")
            # Epic 3 Task 5 (VM-032): layer_*/quality are visualization_node's
            # own params, not rendering_node's TUNABLE_PARAMS above -- a
            # separate client, same SetParameters service type.
            self._cli_setp_viz = self.create_client(
                SetParameters, "/visualization_node/set_parameters")
            # get twin (review 2026-09-09): the GUI's layer switches must
            # reflect the node's REAL layer_* values on load, not assert the
            # defaults -- see get_layers_async()/the get_params handler.
            self._cli_getp_viz = self.create_client(
                GetParameters, "/visualization_node/get_parameters")
            # State telemetry is only meaningful from whichever node is
            # currently active; both publish the same [eye|target|preset|mode]
            # layout, so the GUI/WS clients don't care which one it came from.
            for ns in VCAM_NAMESPACES:
                self.create_subscription(
                    Float64MultiArray, f"{ns}/vcam_state",
                    lambda msg, ns=ns: self._on_state(ns, msg), 10)

        def _on_state(self, ns, msg):
            if ns == self._active_ns:
                self.state = list(msg.data)

        def _on_diagnostics(self, msg):
            # DiagnosticStatus.level is `byte` (rclpy: a 1-length bytes
            # object), not an int -- unwrap it once here so the GUI/WS
            # client only ever sees plain JSON-serializable values.
            render_ms = None
            rows = []
            for st in msg.status:
                values = {kv.key: kv.value for kv in st.values}
                if st.name == "render_ms":
                    render_ms = values.get("render_ms")
                    continue
                rows.append({
                    "topic": st.name,
                    "level": st.level[0] if isinstance(st.level, (bytes, bytearray)) else int(st.level),
                    "age": values.get("last_msg_age_sec"),
                    "dropped_malformed": values.get("dropped_malformed"),
                    "dropped_stale": values.get("dropped_stale"),
                    "dropped_no_tf": values.get("dropped_no_tf"),
                    "dropped_by_rule": values.get("dropped_by_rule"),
                })
            self.diagnostics = {"render_ms": render_ms, "rows": rows}

        def set_look(self, eye, target):
            m = Float64MultiArray()
            m.data = [*eye, *target]
            for pub in self._pub_look:
                pub.publish(m)

        def set_render_mode(self, mode: int):
            self._active_ns = "/visualization_node" if mode == 3 else "/rendering_node"
            m = Int32()
            m.data = mode
            self._pub_mode.publish(m)

        def set_theme(self, name: str):
            m = String()
            m.data = name
            self._pub_theme.publish(m)

        def set_preset_async(self, preset: int):
            """Calls ~/set_virtual_cam on every node whose service is ready.

            Returns a list of (namespace, Future) pairs — empty if neither
            node's service is up.
            """
            futs = []
            for ns, cli in zip(VCAM_NAMESPACES, self._cli):
                if cli.service_is_ready():
                    req = SetVirtualCam.Request()
                    req.preset = preset
                    futs.append((ns, cli.call_async(req)))
            return futs

        def get_params_async(self, names):
            if not self._cli_getp.service_is_ready():
                return None
            req = GetParameters.Request()
            req.names = list(names)
            return self._cli_getp.call_async(req)

        def set_param_async(self, name, value):
            if not self._cli_setp.service_is_ready():
                return None
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = value
            elif isinstance(value, int):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = value
            elif isinstance(value, float):
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = value
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                pv.double_array_value = [float(x) for x in value]
            req = SetParameters.Request()
            req.parameters = [Parameter(name=name, value=pv)]
            return self._cli_setp.call_async(req)

        def get_layers_async(self):
            """GetParameters for the nine layer_* bools from
            visualization_node -- the read twin of set_layers_async below,
            so the GUI can show real values instead of asserted defaults."""
            if not self._cli_getp_viz.service_is_ready():
                return None
            req = GetParameters.Request()
            req.names = [f"layer_{name}" for name in LAYER_NAMES]
            return self._cli_getp_viz.call_async(req)

        def set_layers_async(self, layers: dict):
            """One set_parameters call carrying N layer_<name> bools --
            "one WS message -> N parameter sets, still one client round
            trip" (VM-032 Step 1): the batching is N Parameter entries in a
            single SetParameters.Request, not N separate service calls."""
            if not self._cli_setp_viz.service_is_ready():
                return None
            params = [
                Parameter(name=f"layer_{name}",
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                bool_value=value))
                for name, value in layers.items()]
            req = SetParameters.Request()
            req.parameters = params
            return self._cli_setp_viz.call_async(req)

        def set_quality_async(self, preset: int):
            """Writes visualization_node's `quality` param only -- no live
            in-process effect (P4, deferred to Epic 5); read once at that
            node's next create_renderer() (i.e. its next restart)."""
            if not self._cli_setp_viz.service_is_ready():
                return None
            pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=preset)
            req = SetParameters.Request()
            req.parameters = [Parameter(name="quality", value=pv)]
            return self._cli_setp_viz.call_async(req)

        def set_surround_profile_async(self, profile: str):
            """Writes visualization_node's `surround_stitching_profile`
            param -- live, same on_params() contract as layer_* (unlike
            set_quality_async above, which only takes effect on restart)."""
            if not self._cli_setp_viz.service_is_ready():
                return None
            pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=profile)
            req = SetParameters.Request()
            req.parameters = [Parameter(name="surround_stitching_profile", value=pv)]
            return self._cli_setp_viz.call_async(req)

        @staticmethod
        def param_value(pv):
            """ParameterValue -> python value (None for unset)."""
            t = pv.type
            if t == ParameterType.PARAMETER_BOOL: return pv.bool_value
            if t == ParameterType.PARAMETER_INTEGER: return pv.integer_value
            if t == ParameterType.PARAMETER_DOUBLE: return pv.double_value
            if t == ParameterType.PARAMETER_STRING: return pv.string_value
            if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
                return list(pv.double_array_value)
            return None

    rclpy.init()
    # rclpy.init() hooks SIGTERM but nothing here watches rclpy's shutdown flag,
    # which would leave the process unkillable except by SIGKILL — restore default.
    import signal
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    node = BridgeNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    clients: set = set()

    async def await_ros(fut, timeout=5.0):
        """Await an rclpy future from asyncio."""
        loop = asyncio.get_running_loop()
        afut = loop.create_future()
        fut.add_done_callback(
            lambda f: loop.call_soon_threadsafe(
                lambda: afut.done() or afut.set_result(f.result())))
        return await asyncio.wait_for(afut, timeout=timeout)

    async def fetch_params(names):
        fut = node.get_params_async(names)
        if fut is None:
            raise RuntimeError("get_parameters service unavailable")
        res = await await_ros(fut)
        return {n: node.param_value(v) for n, v in zip(names, res.values)}

    async def do_save_params(path):
        vals = await fetch_params(list(TUNABLE_PARAMS) + ["config_path"])
        src = vals.pop("config_path")
        if not src:
            raise RuntimeError("node has no config_path (relaunch with the "
                               "updated launch file)")
        dst = path or src
        with open(src) as f:
            text = f.read()
        patch = {k: v for k, v in vals.items() if v is not None}
        with open(dst, "w") as f:
            f.write(patch_yaml_text(text, patch))
        return dst

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
                elif cmd == "set_render_mode":
                    node.set_render_mode(payload)
                elif cmd == "set_theme":
                    node.set_theme(payload)
                elif cmd == "set_param":
                    # fire-and-forget: slider drags stream updates
                    if node.set_param_async(*payload) is None:
                        await ws.send(json.dumps({
                            "type": "error",
                            "message": "set_parameters service unavailable"}))
                elif cmd == "get_params":
                    try:
                        vals = await fetch_params(list(TUNABLE_PARAMS))
                        # layer_* live on visualization_node, best-effort
                        # (review 2026-09-09): absent when that node isn't
                        # up (bowl/pointcloud-only sessions) -- the GUI
                        # skips switches it gets no value for.
                        lfut = node.get_layers_async()
                        if lfut is not None:
                            try:
                                lres = await await_ros(lfut, timeout=2.0)
                                for name, v in zip(LAYER_NAMES, lres.values):
                                    vals[f"layer_{name}"] = node.param_value(v)
                            except Exception:
                                pass
                        await ws.send(json.dumps({"type": "params", "values": vals}))
                    except Exception as e:
                        await ws.send(json.dumps({"type": "error",
                                                  "message": f"get_params: {e}"}))
                elif cmd == "set_layers":
                    fut = node.set_layers_async(payload)
                    if fut is None:
                        await ws.send(json.dumps({
                            "type": "error",
                            "message": "visualization_node set_parameters unavailable"}))
                        continue
                    try:
                        res = await await_ros(fut)
                        ok = all(r.successful for r in res.results)
                        await ws.send(json.dumps({
                            "type": "ack", "cmd": "set_layers", "success": ok}))
                    except Exception as e:
                        await ws.send(json.dumps({
                            "type": "error", "message": f"set_layers: {e}"}))
                elif cmd == "set_quality":
                    fut = node.set_quality_async(payload)
                    if fut is None:
                        await ws.send(json.dumps({
                            "type": "error",
                            "message": "visualization_node set_parameters unavailable"}))
                        continue
                    try:
                        res = await await_ros(fut)
                        ok = all(r.successful for r in res.results)
                        await ws.send(json.dumps({
                            "type": "ack", "cmd": "set_quality", "success": ok}))
                    except Exception as e:
                        await ws.send(json.dumps({
                            "type": "error", "message": f"set_quality: {e}"}))
                elif cmd == "set_surround_profile":
                    fut = node.set_surround_profile_async(payload)
                    if fut is None:
                        await ws.send(json.dumps({
                            "type": "error",
                            "message": "visualization_node set_parameters unavailable"}))
                        continue
                    try:
                        res = await await_ros(fut)
                        ok = all(r.successful for r in res.results)
                        await ws.send(json.dumps({
                            "type": "ack", "cmd": "set_surround_profile", "success": ok}))
                    except Exception as e:
                        await ws.send(json.dumps({
                            "type": "error", "message": f"set_surround_profile: {e}"}))
                elif cmd == "save_params":
                    try:
                        dst = await do_save_params(payload)
                        await ws.send(json.dumps({"type": "ack", "cmd": "save_params",
                                                  "success": True, "path": dst}))
                    except Exception as e:
                        await ws.send(json.dumps({"type": "ack", "cmd": "save_params",
                                                  "success": False, "path": str(e)}))
                else:  # set_preset
                    futs = node.set_preset_async(payload)
                    if not futs:
                        await ws.send(json.dumps({
                            "type": "error",
                            "message": "set_virtual_cam service unavailable"}))
                        continue
                    # Fan out to every node whose service is ready (both node
                    # namespaces share the preset table — spec §6); ack from
                    # whichever answers first (VCAM_NAMESPACES order), since
                    # both report the same success/active for the same preset.
                    responses = await asyncio.gather(
                        *(await_ros(fut, timeout=5.0) for _, fut in futs),
                        return_exceptions=True)
                    ok = next((r for r in responses if not isinstance(r, Exception)), None)
                    if ok is None:
                        await ws.send(json.dumps({
                            "type": "error", "message": "set_virtual_cam timed out"}))
                    else:
                        await ws.send(json.dumps({
                            "type": "ack", "cmd": "set_preset",
                            "success": ok.success, "active": ok.active}))
        finally:
            clients.discard(ws)
            node.get_logger().info(f"client disconnected ({len(clients)} total)")

    async def broadcast_state():
        last = None
        last_diag = None
        while True:
            await asyncio.sleep(1.0 / STATE_HZ)
            if not clients:
                continue
            s = node.state
            if s is not None and s != last:
                last = list(s)
                frame = json.dumps({
                    "type": "state",
                    "eye": s[0:3], "target": s[3:6],
                    "preset": int(s[6]) if len(s) > 6 else 0,
                    "render_mode": int(s[7]) if len(s) > 7 else 2})
                await asyncio.gather(
                    *(ws.send(frame) for ws in list(clients)), return_exceptions=True)
            # Same "send only on change" shape as state above, its own frame
            # type -- the GUI panel renders it display-only, no ack/command
            # round-trip involved.
            d = node.diagnostics
            if d is not None and d != last_diag:
                last_diag = d
                diag_frame = json.dumps({"type": "diagnostics", **d})
                await asyncio.gather(
                    *(ws.send(diag_frame) for ws in list(clients)), return_exceptions=True)

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
