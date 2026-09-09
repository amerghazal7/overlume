#!/usr/bin/env python3
"""Ego-anchored vcam composition test (2026-08-19 user directive, live
validation session; docs/superpowers/plans/2026-08-18-visual-mode-epic1.md
Task 5 scope addition).

Proves the composition in visualization_node.cpp's timer_callback did NOT
leak into the vcam's existing contract surface: publishes a static
map->base_link TF at a known, non-trivial pose (nonzero x/y/z, 45 degree
heading -- exercising the "do not assume ego z == 0" requirement) against a
real running node, drives the vcam via ~/set_look (a known offset), and
asserts:

  (a) ~/vcam_state keeps echoing the exact OFFSET values sent via
      ~/set_look, unaffected by the ego pose/heading -- i.e. the ego-anchored
      composition happens only at the render-pose boundary in
      timer_callback, never feeding back into cur_/telemetry (same
      byte-for-byte contract test_vcam_contract.py already locks down for
      the no-TF case; this proves it also holds once TF/anchoring is live).
  (b) ~/ego_state reports ego.valid == 1 throughout -- so (a) is actually
      exercising the anchored path, not vacuously passing because TF never
      came up.
  (c) /rendering/image keeps flowing (no stalled frames) across the whole
      window -- anchoring must not break the render path.

Reuses the harness patterns already in this directory: TF publishing from
test_tf_adapter.py, node subprocess + Image frame recording from
test_theme_ws.py.

Run (ROS + this repo's ros_apps install sourced first):
    source /opt/ros/humble/setup.bash
    source cuda/install/ros_apps/setup.bash
    python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/test_ego_anchored_vcam.py

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install isn't
present.
"""

import math
import os
import subprocess
import sys
import time

OUT_W, OUT_H = 160, 120
TF_RATE_HZ = 20.0
TF_DT = 1.0 / TF_RATE_HZ
# Known, non-trivial ego pose: nonzero x/y/z (the z in particular guards
# against the "z is 0 in fixture bags" trap -- code must not assume it) and a
# 45 degree heading (not 0/90/180, so a heading-agnostic bug wouldn't hide).
EGO_POS = (5.0, 3.0, 1.5)
EGO_HEADING_RAD = math.pi / 4.0
# Offset sent via ~/set_look -- applied immediately (no tween), echoed
# verbatim by ~/vcam_state in the existing (unchanged) offset frame.
LOOK = [2.0, -6.0, 3.0, 0.5, 0.5, 0.5]
SETTLE_SEC = 1.0     # TF flowing + ~/set_look applied + a few timer ticks
OBSERVE_SEC = 1.5    # window to sample telemetry + frame cadence over
MAX_FRAME_GAP_SEC = 0.3  # generous vs. the 33ms tick -- see test_theme_ws.py

REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", ".."))
INSTALL_DIR = os.path.join(REPO_ROOT, "cuda", "install", "ros_apps")


def _popen(cmd: str) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd],
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


def _lifecycle(transition: str, timeout: float = 15.0) -> bool:
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"ros2 lifecycle set /visualization_node {transition}")
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout)
    if result.returncode != 0:
        print(f"  [lifecycle {transition}] stderr: {result.stderr.strip()}", file=sys.stderr)
    return result.returncode == 0


def _wait_running(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


def main() -> int:
    if not os.path.isdir(INSTALL_DIR):
        print("SKIP: cuda/install/ros_apps not built -- run colcon_build.sh first.")
        return 0

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64MultiArray
    from tf2_ros import TransformBroadcaster

    class Fixture(Node):
        def __init__(self):
            super().__init__("ego_anchored_vcam_fixture")
            self.broadcaster = TransformBroadcaster(self)
            self.vcam_states: list[list[float]] = []
            self.ego_states: list[list[float]] = []
            self.frame_times: list[float] = []
            self.look_pub = self.create_publisher(
                Float64MultiArray, "/visualization_node/set_look", 10)
            self.create_subscription(
                Float64MultiArray, "/visualization_node/vcam_state", self._on_vcam, 10)
            self.create_subscription(
                Float64MultiArray, "/visualization_node/ego_state", self._on_ego, 10)
            self.create_subscription(Image, "/rendering/image", self._on_image, 10)

        def _on_vcam(self, msg: Float64MultiArray):
            self.vcam_states.append(list(msg.data))

        def _on_ego(self, msg: Float64MultiArray):
            self.ego_states.append(list(msg.data))

        def _on_image(self, msg: Image):
            self.frame_times.append(time.time())

        def publish_tf(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = EGO_POS[0]
            t.transform.translation.y = EGO_POS[1]
            t.transform.translation.z = EGO_POS[2]
            t.transform.rotation.z = math.sin(EGO_HEADING_RAD / 2.0)
            t.transform.rotation.w = math.cos(EGO_HEADING_RAD / 2.0)
            self.broadcaster.sendTransform(t)

    cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node --ros-args "
        f"-p out_width:={OUT_W} -p out_height:={OUT_H} -p initial_mode:=3")
    node_proc = _popen(cmd)
    rclpy.init()
    fixture = Fixture()
    try:
        if not _wait_running(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            print(f"FAIL: visualization_node exited early:\n{stderr[-2000:]}", file=sys.stderr)
            return 1
        if not _lifecycle("configure") or not _lifecycle("activate"):
            print("FAIL: lifecycle transition failed", file=sys.stderr)
            return 1

        # Get TF flowing and send the offset BEFORE sampling -- the loop
        # below keeps publishing TF at TF_RATE_HZ the whole time so the ego
        # stays valid/fresh for the entire observation window.
        msg = Float64MultiArray()
        msg.data = LOOK
        t_end = time.time() + SETTLE_SEC
        sent_look = False
        while time.time() < t_end:
            fixture.publish_tf()
            if not sent_look:
                fixture.look_pub.publish(msg)
                sent_look = True
            rclpy.spin_once(fixture, timeout_sec=0.0)
            time.sleep(TF_DT)

        fixture.vcam_states.clear()
        fixture.ego_states.clear()
        fixture.frame_times.clear()

        t_end = time.time() + OBSERVE_SEC
        while time.time() < t_end:
            fixture.publish_tf()
            rclpy.spin_once(fixture, timeout_sec=0.0)
            time.sleep(TF_DT)
        # Drain any frames still in flight.
        t_end = time.time() + 0.3
        while time.time() < t_end:
            rclpy.spin_once(fixture, timeout_sec=0.05)

        vcam_states = fixture.vcam_states
        ego_states = fixture.ego_states
        frame_times = fixture.frame_times
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
        _kill(node_proc)

    # (b) ego actually anchored -- not a vacuous no-TF pass.
    if not ego_states:
        print("FAIL: no ~/ego_state samples received", file=sys.stderr)
        return 1
    if not all(s[5] == 1.0 for s in ego_states):
        print(f"FAIL: expected ego.valid==1 throughout, got "
              f"{[s[5] for s in ego_states]}", file=sys.stderr)
        return 1

    # (a) telemetry stayed in the offset frame: ~/vcam_state must still echo
    # exactly what was sent via ~/set_look, regardless of the (nonzero,
    # nonzero-z, 45-degree-heading) ego pose above.
    if not vcam_states:
        print("FAIL: no ~/vcam_state samples received", file=sys.stderr)
        return 1
    for s in vcam_states:
        if len(s) != 9:  # VM-037 Step (d) appended mux_mode at index 8
            print(f"FAIL: unexpected ~/vcam_state shape {s}", file=sys.stderr)
            return 1
        diffs = [abs(s[i] - LOOK[i]) for i in range(6)]
        if any(d > 1e-9 for d in diffs):
            print(f"FAIL: ~/vcam_state leaked ego composition into the offset "
                  f"frame: expected {LOOK}, got {s[:6]} (diffs={diffs})",
                  file=sys.stderr)
            return 1
        if int(s[6]) != 0:
            print(f"FAIL: expected free-look (active_preset==0) after set_look, "
                  f"got {s[6]}", file=sys.stderr)
            return 1
    print(f"INFO: {len(vcam_states)} ~/vcam_state samples all echo the sent "
          f"offset {LOOK} exactly -- anchoring did not leak into telemetry.")

    # (c) /rendering/image kept flowing (composition didn't stall the render
    # path): at least a handful of frames, no gap wider than a few ticks.
    if len(frame_times) < 5:
        print(f"FAIL: too few /rendering/image frames ({len(frame_times)}) "
              f"during the observation window", file=sys.stderr)
        return 1
    gaps = [b - a for a, b in zip(frame_times, frame_times[1:])]
    max_gap = max(gaps) if gaps else 0.0
    if max_gap > MAX_FRAME_GAP_SEC:
        print(f"FAIL: /rendering/image had a {max_gap:.3f}s gap "
              f"(> {MAX_FRAME_GAP_SEC}s) while ego-anchored -- render path stalled.",
              file=sys.stderr)
        return 1
    print(f"INFO: {len(frame_times)} frames, max inter-frame gap "
          f"{max_gap:.3f}s -- OK.")

    print("PASS: ego-anchored composition stays out of the vcam contract surface.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
