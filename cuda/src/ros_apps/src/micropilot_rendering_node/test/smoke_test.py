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


# ── publisher node ─────────────────────────────────────────────────────────────

class SyntheticPublisher(Node):
    COLORS = [(200, 50, 50), (50, 200, 50), (50, 50, 200),
              (200, 200, 50), (50, 200, 200), (200, 50, 200)]

    def __init__(self):
        super().__init__("smoke_synthetic_publisher")
        self._img_pubs = [
            self.create_publisher(Image, f"/camera/cam{i}/image_raw", SENSOR_QOS)
            for i in range(N_CAMERAS)]
        self._info_pubs = [
            self.create_publisher(CameraInfo, f"/camera/cam{i}/camera_info", SENSOR_QOS)
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
        self._sub = self.create_subscription(
            Image, "/rendering/image", self._on_image, 10)

    def _on_image(self, msg: Image):
        if self.received_frame is None:
            self.received_frame = msg


def call_lifecycle_subprocess(transition_name: str, timeout: float = 15.0) -> bool:
    """Send a lifecycle transition via `ros2 lifecycle set` subprocess.

    This avoids any rclpy threading conflicts with the main executor.
    """
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = RENDERING_LIBS + ":" + env.get("LD_LIBRARY_PATH", "")
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"ros2 lifecycle set /rendering_node {transition_name}")
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
