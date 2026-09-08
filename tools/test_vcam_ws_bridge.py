"""Unit test for the WS bridge command parsing (no ROS needed).

Run: pytest tools/test_vcam_ws_bridge.py
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from vcam_ws_bridge import parse_cmd, patch_yaml_text  # noqa: E402


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
])
def test_rejects_malformed(text):
    with pytest.raises(ValueError):
        parse_cmd(text)


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
INSTALL_DIR = os.path.join(REPO_ROOT, "cuda", "install", "ros_apps")
RENDERING_LIBS = os.path.join(
    REPO_ROOT, "cuda", "install", "libs", "rendering_reprojector", "libs")
BRIDGE_SCRIPT = os.path.join(os.path.dirname(__file__), "vcam_ws_bridge.py")
E2E_OUT_W, E2E_OUT_H = 160, 120


def _ros_env():
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = RENDERING_LIBS + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


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


def _wait_running(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


@pytest.mark.skipif(not os.path.isdir(INSTALL_DIR), reason="cuda/install/ros_apps not built")
def test_bridge_e2e_mode3_orbit_and_frames():
    """set_render_mode 3 over WS -> orbit via set_look -> frames keep flowing
    and vcam_state.mode == 3, with BOTH nodes and the real bridge process."""
    rclpy = pytest.importorskip("rclpy")
    from rclpy.node import Node as RclpyNode
    from sensor_msgs.msg import Image
    import websockets

    port = 18765  # fixed test port; distinct from the default 8765
    rendering_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_rendering_node rendering_node --ros-args "
        f"-p out_width:={E2E_OUT_W} -p out_height:={E2E_OUT_H} -p initial_mode:=1")
    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node --ros-args "
        f"-p out_width:={E2E_OUT_W} -p out_height:={E2E_OUT_H} -p initial_mode:=1")
    bridge_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"python3 {BRIDGE_SCRIPT} --port {port}")

    rendering_proc = _popen(rendering_cmd)
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
        for proc, name in ((rendering_proc, "rendering_node"), (viz_proc, "visualization_node")):
            assert _wait_running(proc), (
                f"{name} exited early:\n"
                f"{proc.stderr.read().decode(errors='replace')[-2000:]}")
        assert _lifecycle("/rendering_node", "configure")
        assert _lifecycle("/rendering_node", "activate")
        assert _lifecycle("/visualization_node", "configure")
        assert _lifecycle("/visualization_node", "activate")

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
                # rendering_node's own frame-sync gate needs real per-camera
                # images before it ever renders (not this test's concern —
                # covered by rendering_node's own smoke test); this test only
                # needs both nodes ALIVE so the mux + WS fan-out are real.

                # --- Regression: default config (no set_render_mode sent
                # yet) must still emit state, sourced from rendering_node
                # (the only node an un-configured client can mean) whose own
                # default render_mode_ is 2 — bug: a stale `_last_mode`
                # filter initialized to 1 rejected this forever. ---
                default_states = await collect_states(3.0)
                assert default_states, (
                    "no vcam_state telemetry in the default configuration "
                    "(before any set_render_mode) — regression of shipped "
                    "GUI behavior")
                assert all(s["render_mode"] == 2 for s in default_states), (
                    f"default-config state should read rendering_node's own "
                    f"default render_mode (2): {default_states}")

                # --- Regression: modes 1/2 must show ONLY rendering_node's
                # pose, never flicker with visualization_node's (different)
                # default pose — bug: filtering on vcam_state[7] passed both
                # nodes' messages through in modes 1/2 since rendering_node
                # reports render_mode_ and visualization_node reports
                # active_mode_, which read the same (1) while both are
                # configured with initial_mode:=1. ---
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

                # Frames must keep flowing (now from visualization_node).
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
                    "expected only visualization_node frames while mode == 3")
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
        _kill(rendering_proc)
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
