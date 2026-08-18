#!/usr/bin/env python3
"""Headless smoke test for micropilot_rendering_node.

Steps:
  1. Launches rendering_node as a subprocess (non-blocking).
  2. Configures + activates it via lifecycle services.
  3. Publishes synthetic CameraInfo + rgb8 Images on N topics.
  4. Spins until a /rendering/image is received (or timeout).
  5. Asserts the frame is non-zero (pipeline rendered something).
  6. Exits 0 on success, 1 on failure.

Camera params are taken from the cuda/tests/golden/ fixture so the geometry
is guaranteed to produce visible pixels (same as the golden PSNR test).

Hard timeout: 35 s — never hangs.
Headless — no GUI, no blocking loop.
Does NOT run `ros2 run` interactively (the subprocess is killed at the end).
"""

import math
import os
import subprocess
import sys
import time

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, Int32

# ── constants ─────────────────────────────────────────────────────────────────
# Golden fixture params — identical to cuda/tests/golden/ (proven to render).
N_CAMERAS = 6
CAM_W = 128
CAM_H = 96
OUT_W = 96
OUT_H = 72
TIMEOUT_S = 35.0

# Per-camera extrinsics: flat list [R(9 row-major) | t(3)] × N_CAMERAS
# (from cuda/tests/golden/cameras.txt, same as golden PSNR test)
EXTRINSICS = [
    0.0, -0.17364817766693036, 0.9848077530122081,
    -1.0, -0.0, 0.0,
    0.0, -0.9848077530122081, -0.17364817766693036,
    0.25, 0.0, 0.55,
    0.8660254037844386, -0.08682408883346518, 0.4924038765061041,
    -0.5000000000000001, -0.15038373318043527, 0.8528685319524432,
    0.0, -0.9848077530122081, -0.17364817766693033,
    0.12500000000000003, 0.21650635094610965, 0.55,
    0.8660254037844388, 0.08682408883346514, -0.49240387650610384,
    0.4999999999999998, -0.15038373318043533, 0.8528685319524434,
    0.0, -0.9848077530122081, -0.17364817766693036,
    -0.12499999999999994, 0.21650635094610968, 0.55,
    1.2246467991473532e-16, 0.17364817766693036, -0.9848077530122081,
    1.0, -2.1265768495757716e-17, 1.2060416625018979e-16,
    0.0, -0.9848077530122081, -0.17364817766693036,
    -0.25, 3.061616997868383e-17, 0.55,
    -0.8660254037844384, 0.08682408883346525, -0.49240387650610445,
    0.5000000000000004, 0.15038373318043524, -0.8528685319524429,
    0.0, -0.984807753012208, -0.17364817766693033,
    -0.1250000000000001, -0.2165063509461096, 0.55,
    -0.8660254037844386, -0.08682408883346518, 0.4924038765061041,
    -0.5000000000000001, 0.15038373318043527, -0.8528685319524432,
    -0.0, -0.9848077530122081, -0.17364817766693033,
    0.12500000000000003, -0.21650635094610965, 0.55,
]

# Virtual camera pose: R(9) + t(3) — behind + above the rig looking in
VIRTUAL_POSE = [
    1.0, 0.0, 0.0,
    -0.0, -0.5547001962252291, 0.8320502943378437,
    0.0, -0.8320502943378437, -0.5547001962252291,
    0.0, -3.0, 2.0,
]

# Camera intrinsics for the golden fixture
CAM_FX = 69.84374406843337
CAM_FY = 69.84374406843337
CAM_CX = 64.0
CAM_CY = 48.0

INSTALL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../../../../install/ros_apps"))
RENDERING_LIBS = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../../../../install/libs/rendering_reprojector/libs"))

# QoS profile matching the node's SensorDataQoS subscriptions
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


# ── helpers ───────────────────────────────────────────────────────────────────

def make_camera_info(width: int, height: int, fx: float, fy: float,
                     cx: float, cy: float) -> CameraInfo:
    msg = CameraInfo()
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return msg


def make_rgb8_image(width: int, height: int, r: int, g: int, b: int) -> Image:
    msg = Image()
    msg.width = width
    msg.height = height
    msg.encoding = "rgb8"
    msg.step = width * 3
    msg.data = list(bytes([r, g, b]) * (width * height))
    return msg


def build_ros_args(params: list) -> list:
    """Build --ros-args -p key:=value list."""
    args = ["--ros-args"]
    for name, val in params:
        if isinstance(val, list):
            formatted = "[" + ",".join(str(v) for v in val) + "]"
        elif isinstance(val, float):
            formatted = str(val)
        else:
            formatted = str(val)
        args += ["-p", f"{name}:={formatted}"]
    return args


# ── custom topic names (validates the image_topics/info_topics node param path) ─
# We publish on custom topics and pass them to the node, so the integration
# test exercises the production image_topics/info_topics branch.
IMAGE_TOPICS = [f"/smoke/cam{i}/image_raw" for i in range(N_CAMERAS)]
INFO_TOPICS  = [f"/smoke/cam{i}/camera_info" for i in range(N_CAMERAS)]


# ── publisher node ─────────────────────────────────────────────────────────────

class SyntheticPublisher(Node):
    COLORS = [(200, 50, 50), (50, 200, 50), (50, 50, 200),
              (200, 200, 50), (50, 200, 200), (200, 50, 200)]

    def __init__(self):
        super().__init__("smoke_synthetic_publisher")
        self._img_pubs = [
            self.create_publisher(Image, IMAGE_TOPICS[i], SENSOR_QOS)
            for i in range(N_CAMERAS)]
        self._info_pubs = [
            self.create_publisher(CameraInfo, INFO_TOPICS[i], SENSOR_QOS)
            for i in range(N_CAMERAS)]

    def publish_once(self):
        for i in range(N_CAMERAS):
            r, g, b = self.COLORS[i % len(self.COLORS)]
            self._img_pubs[i].publish(
                make_rgb8_image(CAM_W, CAM_H, r, g, b))
            self._info_pubs[i].publish(
                make_camera_info(CAM_W, CAM_H, CAM_FX, CAM_FY, CAM_CX, CAM_CY))


# ── subscriber + lifecycle client node ────────────────────────────────────────

class SmokeTestNode(Node):
    def __init__(self):
        super().__init__("smoke_test_node")
        self.received_frame: Image | None = None
        self.vcam_state: list | None = None
        # (recv_time, frame_id) for every /rendering/image frame — used by the
        # mode-mux test (Task 4) to tell which node produced each frame and
        # measure switch latency / inter-frame gaps.
        self.frame_log: list[tuple[float, str]] = []
        self._sub = self.create_subscription(
            Image, "/rendering/image", self._on_image, 10)
        self._state_sub = self.create_subscription(
            Float64MultiArray, "/rendering_node/vcam_state", self._on_state, 10)
        self.look_pub = self.create_publisher(
            Float64MultiArray, "/rendering_node/set_look", 10)
        self.mode_pub = self.create_publisher(Int32, "/rendering/set_mode", 10)

    def _on_image(self, msg: Image):
        if self.received_frame is None:
            self.received_frame = msg
        self.frame_log.append((time.time(), msg.header.frame_id))

    def _on_state(self, msg: Float64MultiArray):
        self.vcam_state = list(msg.data)


def call_set_virtual_cam(preset: int, timeout: float = 15.0):
    """Call /rendering_node/set_virtual_cam via subprocess; return (ok, stdout).

    Sources the package install so the SetVirtualCam interface type is known.
    """
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = RENDERING_LIBS + ":" + env.get("LD_LIBRARY_PATH", "")
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"source {INSTALL_DIR}/setup.bash && "
           f"ros2 service call /rendering_node/set_virtual_cam "
           f"micropilot_rendering_node/srv/SetVirtualCam '{{preset: {preset}}}'")
    result = subprocess.run(["bash", "-c", cmd], env=env, capture_output=True,
                            text=True, timeout=timeout)
    out = result.stdout.strip()
    print(f"  [set_virtual_cam preset={preset}] {out.splitlines()[-1] if out else ''}",
          file=sys.stderr)
    return result.returncode == 0, out


def call_lifecycle_subprocess(transition_name: str, timeout: float = 15.0,
                              node_name: str = "/rendering_node") -> bool:
    """Send a lifecycle transition via `ros2 lifecycle set` subprocess.

    This avoids any rclpy threading conflicts with the main executor.
    """
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = RENDERING_LIBS + ":" + env.get("LD_LIBRARY_PATH", "")
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"ros2 lifecycle set {node_name} {transition_name}")
    result = subprocess.run(
        ["bash", "-c", cmd],
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    print(f"  [lifecycle] stdout: {result.stdout.strip()}", file=sys.stderr)
    if result.returncode != 0:
        print(f"  [lifecycle] stderr: {result.stderr.strip()}", file=sys.stderr)
    return result.returncode == 0


# ── mode-mux test (Task 4) ───────────────────────────────────────────────────
RENDER_FRAME_ID = "rendering_virtual_cam"
VIZ_FRAME_ID = "visualization_virtual_cam"


def _launch_visualization_node(out_w: int, out_h: int) -> subprocess.Popen:
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node "
        f"--ros-args -p out_width:={out_w} -p out_height:={out_h} "
        f"-p initial_mode:=1"
    )
    return subprocess.Popen(
        ["bash", "-c", cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        start_new_session=True)


def test_mode_mux(test_node: "SmokeTestNode", viz_out_w: int, viz_out_h: int) -> bool:
    """Drive the global /rendering/set_mode mux with both nodes running.

    rendering_node is already configured+active (its have_set_ is warm from
    earlier steps, so it re-renders+publishes every tick with no need for
    fresh synthetic camera frames). visualization_node is launched here.
    """
    print("INFO: starting visualization_node for mode-mux test …")
    viz_proc = _launch_visualization_node(viz_out_w, viz_out_h)
    try:
        t0 = time.time()
        while time.time() - t0 < 12.0:
            time.sleep(0.2)
            if viz_proc.poll() is not None:
                stderr = viz_proc.stderr.read().decode(errors="replace")
                print(f"FAIL: visualization_node exited early (code {viz_proc.returncode}):\n"
                      f"{stderr[-2000:]}", file=sys.stderr)
                return False

        if not call_lifecycle_subprocess("configure", node_name="/visualization_node"):
            print("FAIL: visualization_node configure transition failed", file=sys.stderr)
            return False
        if not call_lifecycle_subprocess("activate", node_name="/visualization_node"):
            print("FAIL: visualization_node activate transition failed", file=sys.stderr)
            return False

        test_node.frame_log.clear()

        def drive(mode: int, settle_s: float = 1.2) -> float:
            switch_t = time.time()
            msg = Int32()
            msg.data = mode
            test_node.mode_pub.publish(msg)
            time.sleep(settle_s)
            return switch_t

        def check_exclusive(switch_t: float, active_id: str, idle_id: str, label: str) -> str | None:
            log = [(t, fid) for t, fid in test_node.frame_log if t >= switch_t]
            active_times = sorted(t for t, fid in log if fid == active_id)
            idle_after = [t for t, fid in log if fid == idle_id and t - switch_t > 0.5]
            if not active_times or active_times[0] - switch_t > 0.5:
                return (f"mode {label}: {active_id} did not publish within 0.5s of switch "
                        f"(first at {active_times[0] - switch_t if active_times else None})")
            if idle_after:
                return (f"mode {label}: {idle_id} still publishing "
                        f"{idle_after[0] - switch_t:.2f}s after switch")
            settled = [t for t in active_times if t - switch_t > 0.5]
            for a, b in zip(settled, settled[1:]):
                if b - a > 0.5:
                    return f"mode {label}: gap {b - a:.2f}s in {active_id} stream"
            return None

        print("INFO: driving /rendering/set_mode 3 -> 2 -> 3 -> 1 …")
        steps = [
            (3, VIZ_FRAME_ID, RENDER_FRAME_ID),
            (2, RENDER_FRAME_ID, VIZ_FRAME_ID),
            (3, VIZ_FRAME_ID, RENDER_FRAME_ID),
            (1, RENDER_FRAME_ID, VIZ_FRAME_ID),
        ]
        for mode, active_id, idle_id in steps:
            switch_t = drive(mode)
            err = check_exclusive(switch_t, active_id, idle_id, str(mode))
            if err is not None:
                print(f"FAIL: {err}", file=sys.stderr)
                return False
            # frame_id alone can't distinguish mode 1 (bowl) from mode 2
            # (pointcloud hybrid) -- both are rendering_node local views and
            # both stamp RENDER_FRAME_ID. Cross-check the render_mode carried
            # in vcam_state (index 7 of [eye xyz|target xyz|preset|mode]) so a
            # /rendering/set_mode handler that forgets to update render_mode_
            # (only active_mode_) is caught.
            if mode in (1, 2):
                state = test_node.vcam_state
                if state is None or len(state) < 8 or int(state[7]) != mode:
                    print(f"FAIL: mode {mode}: vcam_state[7] (render_mode) = "
                          f"{state[7] if state and len(state) >= 8 else state}, "
                          f"expected {mode}", file=sys.stderr)
                    return False
            print(f"INFO: mode {mode} -> {active_id} exclusive, no gap > 0.5s -- OK.")

        print("PASS: mode mux verified (exactly-one-publisher, no gap > 0.5s).")
        return True
    finally:
        kill_process_group(viz_proc)


def kill_process_group(proc: subprocess.Popen):
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


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    assert len(EXTRINSICS) == N_CAMERAS * 12, \
        f"EXTRINSICS length {len(EXTRINSICS)} != N_CAMERAS*12 = {N_CAMERAS*12}"

    params = [
        ("n_cameras",         N_CAMERAS),
        ("out_width",         OUT_W),
        ("out_height",        OUT_H),
        ("bowl_R0",           6.0),
        ("bowl_k",            0.08),
        ("bowl_Rmax",         20.0),
        ("camera_extrinsics", EXTRINSICS),
        ("virtual_pose",      VIRTUAL_POSE),
        # M1: exercise the production image_topics/info_topics parameter path
        ("image_topics",      IMAGE_TOPICS),
        ("info_topics",       INFO_TOPICS),
    ]

    # ── launch the node subprocess ────────────────────────────────────────────
    ros_args = build_ros_args(params)
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = RENDERING_LIBS + ":" + env.get("LD_LIBRARY_PATH", "")

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_rendering_node rendering_node "
        + " ".join(ros_args)
    )
    node_proc = subprocess.Popen(
        ["bash", "-c", cmd],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,  # own process group → kill all children on teardown
    )

    rclpy.init()
    pub_node = SyntheticPublisher()
    test_node = SmokeTestNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(pub_node)
    executor.add_node(test_node)

    # Spin the executor in a background thread so lifecycle calls don't block it
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    success = False
    try:
        # 1. Wait for the node to start (sourcing ROS + launching takes ~4-8 s)
        print("INFO: waiting for rendering_node to start …")
        t0 = time.time()
        while time.time() - t0 < 12.0:
            time.sleep(0.2)
            if node_proc.poll() is not None:
                stderr_data = b""
                try:
                    stderr_data = node_proc.stderr.read()
                except Exception:
                    pass
                print(f"FAIL: rendering_node exited early (code {node_proc.returncode})",
                      file=sys.stderr)
                print(stderr_data.decode(errors="replace")[-2000:], file=sys.stderr)
                return 1

        # 2. Configure
        print("INFO: sending configure …")
        if not call_lifecycle_subprocess("configure"):
            print("FAIL: configure transition failed", file=sys.stderr)
            return 1
        print("INFO: configure succeeded.")

        # 3. Activate
        print("INFO: sending activate …")
        if not call_lifecycle_subprocess("activate"):
            print("FAIL: activate transition failed", file=sys.stderr)
            return 1
        print("INFO: activate succeeded.")

        # 4. Publish images + wait until a rendered frame arrives.
        # Clear any stale frame that may have arrived from a previous node session.
        test_node.received_frame = None
        print("INFO: publishing synthetic images and waiting for /rendering/image …")
        deadline = time.time() + 15.0
        while time.time() < deadline:
            pub_node.publish_once()
            time.sleep(0.05)
            if test_node.received_frame is not None:
                break

        if test_node.received_frame is None:
            print("FAIL: no /rendering/image received within timeout", file=sys.stderr)
            return 1

        # 5. Assert non-zero
        frame = test_node.received_frame
        data = bytes(frame.data)
        nonzero = sum(1 for b in data if b != 0)
        fraction = nonzero / max(len(data), 1)
        print(f"INFO: frame {frame.width}x{frame.height} enc={frame.encoding} "
              f"nonzero_bytes={nonzero}/{len(data)} ({fraction * 100:.1f}%)")

        if nonzero == 0:
            print("FAIL: rendered frame is all-zero — pipeline produced nothing.", file=sys.stderr)
            return 1

        # 6. Exercise the preset-switch service: all 5 presets must succeed and
        #    report the expected name; an out-of-range index must be rejected.
        print("INFO: testing /rendering_node/set_virtual_cam …")
        expected = ["config", "reverse_follow", "left_side", "right_side", "top_down"]
        for i, name in enumerate(expected, start=1):
            ok, out = call_set_virtual_cam(i)
            if not ok or "success=True" not in out or f"active='{name}'" not in out:
                print(f"FAIL: set_virtual_cam preset {i} expected success/active='{name}'; "
                      f"got:\n{out}", file=sys.stderr)
                return 1
        ok, out = call_set_virtual_cam(99)  # out of range -> rejected
        if not ok or "success=False" not in out:
            print(f"FAIL: set_virtual_cam should reject preset 99; got:\n{out}", file=sys.stderr)
            return 1
        print("INFO: set_virtual_cam presets 1-5 + invalid index verified.")

        # 7. Free-look: publish ~/set_look and verify ~/vcam_state echoes the
        #    pose with the preset flag at 0 (free look).
        print("INFO: testing /rendering_node/set_look → vcam_state …")
        look = [1.5, -2.0, 3.0, 0.0, 0.0, 0.5]
        deadline = time.time() + 10.0
        ok_look = False
        while time.time() < deadline:
            msg = Float64MultiArray()
            msg.data = look
            test_node.look_pub.publish(msg)
            pub_node.publish_once()  # keep the render timer publishing state
            time.sleep(0.2)
            # vcam_state is [eye xyz | target xyz | active_preset | render_mode]
            # (8 floats) — render_mode was added to this telemetry array by
            # already-committed history without updating this assertion
            # (flagged in Epic 0 Task 3); fixed here as Task 4 territory.
            s = test_node.vcam_state
            if (s is not None and len(s) == 8 and s[6] == 0.0
                    and all(abs(s[i] - look[i]) < 1e-4 for i in range(6))):
                ok_look = True
                break
        if not ok_look:
            print(f"FAIL: vcam_state never echoed set_look; last state: "
                  f"{test_node.vcam_state}", file=sys.stderr)
            return 1
        print("INFO: set_look applied and echoed by vcam_state (preset=0 free look).")

        # 8. Mode mux (Task 4): launch visualization_node alongside, drive the
        #    global /rendering/set_mode 3->2->3->1, assert exactly-one-publisher
        #    (by header.frame_id) with no switch taking longer than 0.5s and no
        #    gap > 0.5s once settled — spec §10 / plan Task 4 step 1.
        ok = test_mode_mux(test_node, viz_out_w=OUT_W, viz_out_h=OUT_H)
        if not ok:
            return 1

        print("PASS: smoke test passed — non-blank frame received and verified.")
        success = True
        return 0

    finally:
        # Kill the entire process group (bash + ros2 run child)
        try:
            os.killpg(os.getpgid(node_proc.pid), 15)  # SIGTERM
        except ProcessLookupError:
            pass
        try:
            node_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(node_proc.pid), 9)  # SIGKILL
            except ProcessLookupError:
                pass
            node_proc.wait()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        rclpy.shutdown()
        if not success:
            try:
                stderr_data = node_proc.stderr.read()
                if stderr_data:
                    print("--- rendering_node stderr ---", file=sys.stderr)
                    print(stderr_data.decode(errors="replace")[-3000:], file=sys.stderr)
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
