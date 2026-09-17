"""Unit test for the WS bridge command parsing (no ROS needed).

Run: pytest tools/test_vcam_ws_bridge.py
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from vcam_ws_bridge import (  # noqa: E402
    ENVIRONMENT_PRESET_URIS, ENVIRONMENT_PRESETS, parse_cmd, patch_yaml_text,
)


def test_set_look_valid():
    cmd, (eye, target) = parse_cmd(json.dumps(
        {"cmd": "set_look", "eye": [1, 2, 3.5], "target": [0, 0, 0.3]}))
    assert cmd == "set_look"
    assert eye == [1.0, 2.0, 3.5]
    assert target == [0.0, 0.0, 0.3]


def test_set_preset_valid():
    assert parse_cmd('{"cmd": "set_preset", "preset": 5}') == ("set_preset", 5)


@pytest.mark.parametrize("mode,expect", [
    ("bowl", 1), ("pointcloud", 2), ("visual", 3), (1, 1), (2, 2), (3, 3),
])
def test_set_render_mode_valid(mode, expect):
    assert parse_cmd(json.dumps({"cmd": "set_render_mode", "mode": mode})) == \
        ("set_render_mode", expect)


@pytest.mark.parametrize("theme", ["dark_adas", "light_clay"])
def test_set_theme_valid(theme):
    assert parse_cmd(json.dumps({"cmd": "set_theme", "theme": theme})) == \
        ("set_theme", theme)


@pytest.mark.parametrize("text", [
    "not json",
    "[1,2,3]",                                            # not an object
    '{"cmd": "warp"}',                                    # unknown cmd
    '{"cmd": "set_look", "eye": [1, 2], "target": [0, 0, 0]}',       # short eye
    '{"cmd": "set_look", "eye": [1, 2, "x"], "target": [0, 0, 0]}',  # non-number
    '{"cmd": "set_look", "eye": [1, 2, 3]}',              # missing target
    '{"cmd": "set_preset", "preset": 0}',                 # out of range
    '{"cmd": "set_preset", "preset": 6}',
    '{"cmd": "set_preset", "preset": true}',              # bool is not an index
    '{"cmd": "set_preset", "preset": "2"}',
    '{"cmd": "set_render_mode", "mode": 4}',              # unknown mode
    '{"cmd": "set_render_mode", "mode": "depth"}',
    '{"cmd": "set_render_mode", "mode": true}',           # bool is not a mode
    '{"cmd": "set_theme"}',                               # missing theme
    '{"cmd": "set_theme", "theme": ""}',                  # empty string
    '{"cmd": "set_theme", "theme": 1}',                   # non-string
    '{"cmd": "set_param", "name": "nope", "value": 1}',   # untunable param
    '{"cmd": "set_param", "name": "bowl_R0", "value": "6"}',
    '{"cmd": "set_param", "name": "splat_radius", "value": 2.5}',
    '{"cmd": "set_param", "name": "fill_blind_zone", "value": 1}',
    '{"cmd": "set_param", "name": "camera_extrinsics", "value": []}',
    '{"cmd": "save_params", "path": ""}',
    '{"cmd": "set_layers"}',                              # missing layers
    '{"cmd": "set_layers", "layers": {}}',                # empty
    '{"cmd": "set_layers", "layers": {"nope": true}}',    # unknown layer name
    '{"cmd": "set_layers", "layers": {"objects": 1}}',    # non-bool value
    '{"cmd": "set_quality"}',                             # missing preset
    '{"cmd": "set_quality", "preset": "ultra"}',           # unknown preset
    '{"cmd": "set_quality", "preset": 3}',                 # out of range
    '{"cmd": "set_quality", "preset": true}',              # bool is not a preset
    '{"cmd": "set_surround_profile"}',                    # missing profile
    '{"cmd": "set_surround_profile", "profile": "lidar"}',  # unknown profile
    '{"cmd": "set_environment_enabled"}',                 # missing enabled
    '{"cmd": "set_environment_enabled", "enabled": "true"}',  # non-bool
    '{"cmd": "set_environment_enabled", "enabled": 1}',   # non-bool (int)
    '{"cmd": "set_environment_source"}',                  # missing preset
    '{"cmd": "set_environment_source", "preset": "satellite"}',  # unknown preset
    '{"cmd": "set_environment_source", "preset": true}',  # bool is not a preset
])
def test_rejects_malformed(text):
    with pytest.raises(ValueError):
        parse_cmd(text)


def test_set_layers_valid():
    assert parse_cmd(json.dumps(
        {"cmd": "set_layers", "layers": {"objects": False}})) == \
        ("set_layers", {"objects": False})
    assert parse_cmd(json.dumps(
        {"cmd": "set_layers", "layers": {"objects": True, "grids": False}})) == \
        ("set_layers", {"objects": True, "grids": False})


@pytest.mark.parametrize("preset,expect", [
    ("low", 0), ("medium", 1), ("high", 2), (0, 0), (1, 1), (2, 2),
])
def test_set_quality_valid(preset, expect):
    assert parse_cmd(json.dumps({"cmd": "set_quality", "preset": preset})) == \
        ("set_quality", expect)


@pytest.mark.parametrize("profile", ["bowl", "hybrid"])
def test_set_surround_profile_valid(profile):
    assert parse_cmd(json.dumps({"cmd": "set_surround_profile", "profile": profile})) == \
        ("set_surround_profile", profile)


@pytest.mark.parametrize("enabled", [True, False])
def test_set_environment_enabled_valid(enabled):
    assert parse_cmd(json.dumps({"cmd": "set_environment_enabled", "enabled": enabled})) == \
        ("set_environment_enabled", enabled)


@pytest.mark.parametrize("preset", ["baked", "osm", "clipped", "google"])
def test_set_environment_source_valid(preset):
    assert parse_cmd(json.dumps({"cmd": "set_environment_source", "preset": preset})) == \
        ("set_environment_source", preset)


def test_google_preset_ships_cache_off():
    # Regression: the google preset must keep the shipped `cache=off`
    # compliance lever (default_params.yaml, cesium.md's Google section) --
    # dropping it silently starts persisting Google tiles to an on-disk
    # SQLite cache on every deployment that picks this preset.
    assert ENVIRONMENT_PRESET_URIS["google"].endswith("cache=off")


def test_param_cmds_valid():
    assert parse_cmd('{"cmd": "get_params"}') == ("get_params", None)
    assert parse_cmd('{"cmd": "set_param", "name": "bowl_R0", "value": 6}') == \
        ("set_param", ("bowl_R0", 6.0))
    assert parse_cmd('{"cmd": "set_param", "name": "splat_radius", "value": 3}') == \
        ("set_param", ("splat_radius", 3))
    assert parse_cmd(
        '{"cmd": "set_param", "name": "exposure_match", "value": false}') == \
        ("set_param", ("exposure_match", False))
    cmd, (name, v) = parse_cmd(
        '{"cmd": "set_param", "name": "camera_extrinsics", "value": [1, 0, 2.5]}')
    assert v == [1.0, 0.0, 2.5]
    assert parse_cmd('{"cmd": "save_params"}') == ("save_params", None)
    assert parse_cmd('{"cmd": "save_params", "path": "/tmp/x.yaml"}') == \
        ("save_params", "/tmp/x.yaml")


YAML = """# header comment
/**:
  ros__parameters:
    n_cameras: 6
    bowl_R0: 6.0  # tuned
    fill_blind_zone: true
    camera_extrinsics:
    - 1.0
    - 2.0
    image_topics:
    - /a
    - /b
"""


def test_patch_yaml_scalar_and_list():
    out = patch_yaml_text(YAML, {"bowl_R0": 9.5, "fill_blind_zone": False,
                                 "camera_extrinsics": [3.0, 4.0, 5.0]})
    assert "# header comment" in out
    assert "    bowl_R0: 9.5\n" in out
    assert "    fill_blind_zone: false\n" in out
    assert "    - 3\n" in out and "    - 5\n" in out and "- 1.0" not in out
    assert "- /a" in out and "- /b" in out          # untouched string list
    assert "    n_cameras: 6" in out


def test_patch_yaml_appends_missing_key():
    out = patch_yaml_text(YAML, {"exposure_match": True})
    assert "    exposure_match: true\n" in out


# ── Bridge E2E: mode-3 switch + orbit while streaming ────────────────────────
# Full-stack integration: bridge <-> both ROS nodes <-> a real websocket
# client. Skips cleanly (not a failure) when the ROS install this repo
# builds isn't present -- same spirit as the C++ GL tests skipping without a
# GPU: this test needs `colcon_build.sh` to have run first.
import asyncio
import subprocess
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INSTALL_DIR = os.path.join(REPO_ROOT, "ros", "install")
BRIDGE_SCRIPT = os.path.join(os.path.dirname(__file__), "vcam_ws_bridge.py")
E2E_OUT_W, E2E_OUT_H = 160, 120


def _ros_env():
    # The retired CUDA reprojector's pybind libs used to be prepended to
    # LD_LIBRARY_PATH here; the Filament node needs nothing beyond its install.
    return os.environ.copy()


def _popen(cmd: str) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd], env=_ros_env(),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            start_new_session=True)


def _kill(proc: subprocess.Popen):
    try:
        os.killpg(os.getpgid(proc.pid), 15)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), 9)
        except ProcessLookupError:
            pass
        proc.wait()


def _lifecycle(node_name: str, transition: str) -> bool:
    cmd = f"source /opt/ros/humble/setup.bash && ros2 lifecycle set {node_name} {transition}"
    result = subprocess.run(["bash", "-c", cmd], env=_ros_env(), capture_output=True,
                            text=True, timeout=15.0)
    return result.returncode == 0


def _param_get(node_name: str, param_name: str) -> str:
    cmd = f"source /opt/ros/humble/setup.bash && ros2 param get {node_name} {param_name}"
    result = subprocess.run(["bash", "-c", cmd], env=_ros_env(), capture_output=True,
                            text=True, timeout=15.0)
    return result.stdout.strip()


def _param_get_bool(node_name: str, param_name: str) -> bool:
    out = _param_get(node_name, param_name)
    if "True" in out:
        return True
    if "False" in out:
        return False
    raise AssertionError(f"unexpected `ros2 param get {node_name} {param_name}` output: {out!r}")


def _wait_running(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


@pytest.mark.skipif(not os.path.isdir(INSTALL_DIR), reason="ros/install not built")
def test_bridge_e2e_mode3_orbit_and_frames():
    """set_render_mode 3 over WS -> orbit via set_look -> frames keep flowing
    and vcam_state.mode == 3, with BOTH nodes and the real bridge process."""
    rclpy = pytest.importorskip("rclpy")
    from rclpy.node import Node as RclpyNode
    from sensor_msgs.msg import Image
    import websockets

    port = 18765  # fixed test port; distinct from the default 8765
    # Post-cutover (VM-095): the merged node is the ONLY rendering process --
    # it serves every mode via its local render_mode dispatch, and
    # /rendering/set_mode assigns render_mode directly (Step 2).
    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"-p out_width:={E2E_OUT_W} -p out_height:={E2E_OUT_H} -p initial_mode:=1")
    bridge_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"python3 {BRIDGE_SCRIPT} --port {port}")

    viz_proc = _popen(viz_cmd)

    rclpy.init()

    class FrameCounter(RclpyNode):
        def __init__(self):
            super().__init__("e2e_frame_counter")
            self.frames: list[tuple[float, str]] = []
            self.create_subscription(Image, "/rendering/image", self._on_image, 10)

        def _on_image(self, msg: Image):
            self.frames.append((time.time(), msg.header.frame_id))

    counter = FrameCounter()
    bridge_proc = None
    try:
        assert _wait_running(viz_proc), (
            f"overlume_node exited early:\n"
            f"{viz_proc.stderr.read().decode(errors='replace')[-2000:]}")
        assert _lifecycle("/overlume_node", "configure")
        assert _lifecycle("/overlume_node", "activate")

        bridge_proc = _popen(bridge_cmd)
        # Give the bridge's rclpy node + websocket server time to come up.
        deadline = time.time() + 12.0
        connected = None
        last_err = None

        async def run_client():
            nonlocal connected
            uri = f"ws://127.0.0.1:{port}"
            while time.time() < deadline and connected is None:
                try:
                    connected = await websockets.connect(uri, open_timeout=1.0)
                except OSError as e:
                    last_err = e
                    await asyncio.sleep(0.3)
            assert connected is not None, f"could not connect to bridge: {last_err}"
            ws = connected

            async def collect_states(duration):
                frames = []
                t0 = time.time()
                while time.time() - t0 < duration:
                    try:
                        frame = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.1))
                        if frame.get("type") == "state":
                            frames.append(frame)
                    except asyncio.TimeoutError:
                        pass
                return frames

            try:
                # --- Regression: default config (no set_render_mode sent
                # yet) must still emit state from the sole node (bug: a stale
                # `_last_mode` filter initialized to 1 rejected this forever).
                # render_mode expected: 1 (this launch's initial_mode). ---
                default_states = await collect_states(3.0)
                assert default_states, (
                    "no vcam_state telemetry in the default configuration "
                    "(before any set_render_mode) — regression of shipped "
                    "GUI behavior")
                assert all(s["render_mode"] == 1 for s in default_states), (
                    f"default-config state should read render_mode synced "
                    f"from initial_mode:=1 (1, bowl): {default_states}")

                # --- Single-process pose stability: mode switches must not
                # oscillate the reported pose (the two-node flicker this
                # once guarded is gone with the mux; a stable pose per mode
                # is still the contract). Drive to 2 first so the mode-1
                # command below is a REAL value change (broadcast_state()
                # emits only on change). ---
                await ws.send(json.dumps({"cmd": "set_render_mode", "mode": 2}))
                await collect_states(0.5)
                await ws.send(json.dumps({"cmd": "set_render_mode", "mode": 1}))
                mode1_states = await collect_states(2.0)
                assert mode1_states, "no state frames after set_render_mode 1"
                poses = {(tuple(s["eye"]), tuple(s["target"])) for s in mode1_states}
                assert len(poses) == 1, (
                    f"vcam_state pose oscillated between nodes while mode == 1: {poses}")

                await ws.send(json.dumps({"cmd": "set_render_mode", "mode": 3}))
                # Orbit a couple of steps via set_look while mode 3 is active.
                for eye, target in (([1.0, 2.0, 3.0], [0.0, 0.0, 0.5]),
                                    ([-1.0, -2.0, 3.5], [0.0, 0.0, 0.3])):
                    await ws.send(json.dumps(
                        {"cmd": "set_look", "eye": eye, "target": target}))
                    await asyncio.sleep(0.3)

                # Frames must keep flowing (now from overlume_node).
                counter.frames.clear()
                t0 = time.time()
                mode3_state = None
                while time.time() - t0 < 3.0:
                    rclpy.spin_once(counter, timeout_sec=0.02)
                    try:
                        frame = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.05))
                        if frame.get("type") == "state":
                            mode3_state = frame
                    except asyncio.TimeoutError:
                        pass
                assert len(counter.frames) >= 3, (
                    f"frames did not keep flowing after set_render_mode 3: "
                    f"{len(counter.frames)} in 3s")
                assert all(fid == "visualization_virtual_cam" for _, fid in counter.frames), (
                    "unexpected frame_id from the sole rendering process")
                assert mode3_state is not None, "no {'type':'state'} frame observed"
                assert mode3_state["render_mode"] == 3, (
                    f"vcam_state.mode != 3: {mode3_state}")
            finally:
                await ws.close()

        asyncio.run(run_client())
    finally:
        counter.destroy_node()
        rclpy.shutdown()
        if bridge_proc is not None:
            _kill(bridge_proc)
        _kill(viz_proc)


LIVE_LAYERS = ("objects", "paths", "map_elements", "grids", "alerts", "markers")


@pytest.mark.skipif(not os.path.isdir(INSTALL_DIR), reason="ros/install not built")
def test_bridge_e2e_set_layers_hides_and_shows():
    """set_layers over WS -> N params on overlume_node's own
    set_parameters service, live (no restart) -- hide -> ros2 param get
    reads false for all six LIVE categories AND the published frame's
    pixels actually change (review 2026-09-09: param readback alone cannot
    distinguish "param written" from "gate applied"; the empty-scene frame
    shows the ground grid, so hiding layer_grids must change pixels) ->
    show -> both recover (VM-032 Step 1). Coverage shape (review 2026-09-09):
    per-layer `ros2 param get` readback for all six LIVE_LAYERS, plus ONE
    aggregate pixel-change assertion with all six hidden together
    (attributable to layer_grids -- not per-layer pixels). layer_point_clouds
    gates a live category as of Task 6/VM-035 but is not asserted here (no
    sensor_msgs/PointCloud2 recording, named FIXTURE GAP); its gate is covered
    instead by test_scene_assembly.cpp's ApplyLayerGates* cases. Mode 3 so
    overlume_node actually publishes frames."""
    websockets = pytest.importorskip("websockets")
    rclpy = pytest.importorskip("rclpy")
    from rclpy.node import Node as RclpyNode
    from sensor_msgs.msg import Image

    port = 18766
    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"-p out_width:={E2E_OUT_W} -p out_height:={E2E_OUT_H} -p initial_mode:=3")
    bridge_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"python3 {BRIDGE_SCRIPT} --port {port}")

    viz_proc = _popen(viz_cmd)
    bridge_proc = None
    rclpy.init()

    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray

    class FrameGrabber(RclpyNode):
        """Grabs frames AND feeds the scene: an empty scene renders only the
        renderer-internal ground+grid, which no layer_* gates -- so the
        pixel-diff assertion needs real category content. Publishes a
        map->base_link TF (ego anchor) and one CUBE object marker in front
        of the default camera; hiding layer_objects must then change pixels."""
        def __init__(self):
            super().__init__("e2e_layer_frame_grabber")
            self.latest = None
            self.create_subscription(Image, "/rendering/image", self._on_image, 1)
            self._tf_pub = self.create_publisher(TFMessage, "/tf", 10)
            self._obj_pub = self.create_publisher(
                MarkerArray, "/perception/dynamic_objects_list", 10)

        def _on_image(self, msg: Image):
            self.latest = bytes(msg.data)

        def feed_scene(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.rotation.w = 1.0
            self._tf_pub.publish(TFMessage(transforms=[t]))
            m = Marker()
            m.header.stamp = t.header.stamp
            m.header.frame_id = "map"
            m.ns = "e2e"
            m.id = 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = 4.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 2.0
            m.color.a = 1.0
            self._obj_pub.publish(MarkerArray(markers=[m]))

    grabber = FrameGrabber()

    def settled_frame(timeout=6.0):
        """Spin until a FRESH frame arrives (clears first so a pre-toggle
        frame can't satisfy the read), return its pixel bytes."""
        grabber.latest = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            grabber.feed_scene()
            rclpy.spin_once(grabber, timeout_sec=0.05)
            if grabber.latest is not None:
                # one more fresh frame: the first may have been mid-flight
                # (rendered before the toggle landed)
                first = grabber.latest
                grabber.latest = None
                t1 = time.time()
                while time.time() - t1 < timeout and grabber.latest is None:
                    grabber.feed_scene()
                    rclpy.spin_once(grabber, timeout_sec=0.05)
                return grabber.latest if grabber.latest is not None else first
        raise AssertionError("no /rendering/image frame within timeout")

    try:
        assert _wait_running(viz_proc), (
            f"overlume_node exited early:\n"
            f"{viz_proc.stderr.read().decode(errors='replace')[-2000:]}")
        assert _lifecycle("/overlume_node", "configure")
        assert _lifecycle("/overlume_node", "activate")

        bridge_proc = _popen(bridge_cmd)
        deadline = time.time() + 12.0
        connected = None
        last_err = None

        async def run_client():
            nonlocal connected
            uri = f"ws://127.0.0.1:{port}"
            while time.time() < deadline and connected is None:
                try:
                    connected = await websockets.connect(uri, open_timeout=1.0)
                except OSError as e:
                    last_err = e
                    await asyncio.sleep(0.3)
            assert connected is not None, f"could not connect to bridge: {last_err}"
            ws = connected
            try:
                async def recv_ack(cmd, timeout=5.0):
                    # The bridge also streams "state"/"diagnostics" frames on
                    # this same socket (broadcast_state(), ~15 Hz) -- skip
                    # anything that isn't this command's own ack.
                    t0 = time.time()
                    while time.time() - t0 < timeout:
                        frame = json.loads(await asyncio.wait_for(
                            ws.recv(), timeout=timeout - (time.time() - t0)))
                        if frame.get("type") == "ack" and frame.get("cmd") == cmd:
                            return frame
                    raise AssertionError(f"no {cmd!r} ack within {timeout}s")

                baseline_frame = settled_frame()
                await ws.send(json.dumps({
                    "cmd": "set_layers",
                    "layers": {name: False for name in LIVE_LAYERS}}))
                ack = await recv_ack("set_layers")
                assert ack.get("success"), ack
                for name in LIVE_LAYERS:
                    assert _param_get_bool("/overlume_node", f"layer_{name}") is False, \
                        f"layer_{name} did not hide"
                hidden_frame = settled_frame()
                assert hidden_frame != baseline_frame, (
                    "hiding every layer (incl. grids) left the published "
                    "frame byte-identical -- the gate never applied")

                await ws.send(json.dumps({
                    "cmd": "set_layers",
                    "layers": {name: True for name in LIVE_LAYERS}}))
                ack2 = await recv_ack("set_layers")
                assert ack2.get("success"), ack2
                for name in LIVE_LAYERS:
                    assert _param_get_bool("/overlume_node", f"layer_{name}") is True, \
                        f"layer_{name} did not recover"
                shown_frame = settled_frame()
                assert shown_frame != hidden_frame, (
                    "re-showing the layers did not change the published "
                    "frame back -- the gate is stuck hidden")
            finally:
                await ws.close()

        asyncio.run(run_client())
    finally:
        grabber.destroy_node()
        rclpy.shutdown()
        if bridge_proc is not None:
            _kill(bridge_proc)
        _kill(viz_proc)


def test_pose_roundtrip():
    pytest.importorskip("gi")
    np = pytest.importorskip("numpy")
    from vcam_gui import pose_to_rt, rt_to_pose
    # a real bl-style row: yawed back-left, pitched slightly down
    row = pose_to_rt(0.21, 0.38, 0.72, 139.0, -2.2, 1.5)
    back = rt_to_pose(row)
    assert np.allclose(back, [0.21, 0.38, 0.72, 139.0, -2.2, 1.5], atol=1e-6)
    R = np.array(row[:9]).reshape(3, 3)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9)  # orthonormal


def test_resolve_environment_preset():
    pytest.importorskip("gi")
    from vcam_gui import resolve_environment_preset
    assert resolve_environment_preset("", "") == "baked"
    assert resolve_environment_preset("ion://96188", "") == "osm"
    assert resolve_environment_preset(
        "ion://2275207?materials=original&cache=off", "") == "google"
    assert resolve_environment_preset("ion://12345", "ion://12345") == "clipped"
    # no own asset configured yet -- must never guess "clipped"
    assert resolve_environment_preset("ion://12345", "") is None
    # a hand-edited URI matching nothing known -- leave the combo alone
    assert resolve_environment_preset("ion://99999?foo=bar", "ion://12345") is None


def test_environment_initial_sensitivity_fails_closed():
    # Regression for gate finding #21: every preset must start sensitive
    # EXCEPT "clipped", which fails closed until the first get_params
    # round-trip proves an own asset is configured. Reverting to "always
    # True" (the pre-fix bug) makes this fail on the "clipped" case.
    pytest.importorskip("gi")
    from vcam_gui import ENVIRONMENT_PRESETS, initial_environment_sensitivity
    for preset in ENVIRONMENT_PRESETS:
        assert initial_environment_sensitivity(preset) == (preset != "clipped")


def test_ack_failure_text():
    # Regression for gate finding #4: a rejected ack must render its cmd +
    # reason (or "rejected" when the bridge sent none), not be silently
    # dropped.
    pytest.importorskip("gi")
    from vcam_gui import ack_failure_text
    assert ack_failure_text({"cmd": "set_environment_source",
                             "reason": "geo-anchor not solved yet"}) == \
        "✘ set_environment_source: geo-anchor not solved yet"
    assert ack_failure_text({"cmd": "set_layers"}) == "✘ set_layers: rejected"


def test_gui_and_bridge_environment_preset_tables_agree():
    """The GUI keeps its own literal copy of the preset->URI table (it is a
    pure WS client, deliberately not importing the bridge module). That
    decoupling is fine; silent DRIFT is not -- edit the google URI's
    cache=off compliance lever in one file only and the GUI's reverse map
    stops recognizing the node's real source, so the combo silently stops
    reflecting reality. This test is what makes that fail loudly instead."""
    pytest.importorskip("gi")
    from vcam_gui import ENVIRONMENT_PRESETS as GUI_PRESETS
    from vcam_gui import ENVIRONMENT_PRESET_URIS_FIXED

    # Same preset names on both sides (the GUI orders them for display; the
    # bridge validates membership, hence list-vs-set).
    assert set(GUI_PRESETS) == ENVIRONMENT_PRESETS
    # "clipped" is per-deployment (environment_own_asset_uri), so it is
    # absent from BOTH literal tables by design -- assert that too, so a
    # future fabricated id in either file fails here.
    assert "clipped" not in ENVIRONMENT_PRESET_URIS_FIXED
    assert "clipped" not in ENVIRONMENT_PRESET_URIS
    # Every fixed URI byte-identical across the two copies.
    assert ENVIRONMENT_PRESET_URIS_FIXED == ENVIRONMENT_PRESET_URIS
