#!/usr/bin/env python3
"""TF adapter test: map->base_link TF -> EMA-smoothed ego speed (Epic 1 Task 4
/ VM-012, docs/superpowers/plans/2026-08-18-visual-mode-epic1.md Task 4 Step
7-9).

Publishes a recorded fixture -- straight-line motion at a known constant
speed (2.0 m/s) on a real tf2 buffer at 20 Hz, with one deliberately noisy
outlier sample partway through -- against the real, running
visualization_node, and samples its ~/ego_state debug topic (the
"~/vcam_state-adjacent ego telemetry" the plan step calls for: same
Float64MultiArray-per-tick pattern as ~/vcam_state, laid out as
[x, y, z, heading_rad, speed_mps, valid]).

Two things are asserted, matching the plan step's own two clauses:
  (a) the finite-differenced+smoothed speed converges to 2.0 m/s (within
      tolerance) once the EMA filter settles;
  (b) the single noisy outlier sample does not spike the *reported* speed
      anywhere near the outlier's own raw magnitude -- that's what the
      smoothing is for. (The raw finite-difference for that one sample is
      ~100 m/s; if the filter weren't smoothing, the reported speed would
      spike to roughly that. EMA at alpha=0.2 caps a single sample's
      contribution to (raw-prev)*0.2, so the ceiling below is a real,
      derived bound, not an arbitrary tolerance.)

Run (ROS + this repo's ros install sourced first, per plan Task 4 Step
11):
    source /opt/ros/humble/setup.bash
    source ros/install/setup.bash
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest \
        ros/src/micropilot_visualization_node/test/test_tf_adapter.py

Skips cleanly when this repo's ros install isn't built yet.
"""

import os
import subprocess
import sys
import time

import pytest

OUT_W, OUT_H = 160, 120
TF_RATE_HZ = 20.0
TF_DT = 1.0 / TF_RATE_HZ
SPEED_MPS = 2.0
N_STEPS = 100                # 5s of straight-line motion at 20Hz
OUTLIER_STEP = 40            # inject the bad sample partway through (t=2.0s)
OUTLIER_JUMP_M = 5.0         # instantaneous position jump for that one sample
SMOOTHING_ALPHA = 0.2        # matches the node's declared default (ego_speed_smoothing_alpha)
# EMA ceiling for the outlier tick: alpha*(raw~100 m/s - prev~2 m/s) bounds
# the single-step jump; 60 sits comfortably above that bound, far below raw.
OUTLIER_SMOOTHED_CEILING_MPS = 60.0

INSTALL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../../install"))


def _popen(cmd: str) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, start_new_session=True)


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


def _wait_for_start(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


def test_tf_adapter_speed_converges():
    if not os.path.isdir(INSTALL_DIR):
        pytest.skip("ros/install/ not found -- build with colcon_build.sh first")

    # Imports deferred past the skip check: only meaningful once the ROS
    # environment (sourced by the caller, per this file's own docstring) is
    # actually on the path.
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from tf2_ros import TransformBroadcaster

    class TfFixtureNode(Node):
        """Publishes the straight-line-motion TF fixture, samples ~/ego_state."""

        def __init__(self):
            super().__init__("tf_adapter_fixture")
            self.broadcaster = TransformBroadcaster(self)
            self.samples = []  # [x, y, z, heading_rad, speed_mps, valid], one per tick
            self.create_subscription(Float64MultiArray, "/visualization_node/ego_state",
                                     self._on_ego, 20)

        def _on_ego(self, msg: Float64MultiArray):
            self.samples.append(list(msg.data))

        def publish_tf(self, x: float):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node "
        f"--ros-args -p out_width:={OUT_W} -p out_height:={OUT_H} "
        f"-p initial_mode:=1 -p ego_speed_smoothing_alpha:={SMOOTHING_ALPHA}"
    )
    node_proc = _popen(cmd)
    fixture = None
    try:
        if not _wait_for_start(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            pytest.fail(f"node exited early (code {node_proc.returncode}):\n{stderr[-2000:]}")
        assert _lifecycle("configure"), "configure transition failed"
        assert _lifecycle("activate"), "activate transition failed"

        rclpy.init()
        fixture = TfFixtureNode()

        # Real-time-paced (not spin-once-as-fast-as-possible): finite-difference
        # math is dx/dt off real TF timestamps, so the loop must take ~TF_DT
        # per step for the "constant 2.0 m/s" fixture to actually measure as
        # 2.0 m/s once timestamped.
        for i in range(N_STEPS):
            x = SPEED_MPS * i * TF_DT
            if i == OUTLIER_STEP:
                x += OUTLIER_JUMP_M  # single noisy sample -- see module docstring
            fixture.publish_tf(x)
            rclpy.spin_once(fixture, timeout_sec=0.0)
            time.sleep(TF_DT)

        # Drain a few more of the node's own 33ms ticks past the last TF
        # sample so the tail-window assertion below isn't racing the last
        # publish.
        t_end = time.time() + 0.5
        while time.time() < t_end:
            rclpy.spin_once(fixture, timeout_sec=0.05)

        samples = fixture.samples
    finally:
        if fixture is not None:
            fixture.destroy_node()
            rclpy.shutdown()
        _kill(node_proc)

    assert len(samples) > 10, f"expected ~/ego_state samples, got {len(samples)}"

    # (b) outlier robustness: the *reported* (smoothed) speed must never
    # approach the outlier's raw magnitude (~100 m/s) -- see
    # OUTLIER_SMOOTHED_CEILING_MPS's derivation in the module docstring.
    max_speed = max(s[4] for s in samples)
    assert max_speed < OUTLIER_SMOOTHED_CEILING_MPS, (
        f"reported speed spiked to {max_speed:.1f} m/s -- smoothing did not "
        f"absorb the single noisy TF sample (ceiling {OUTLIER_SMOOTHED_CEILING_MPS})")

    # (a) convergence: the tail (well after the outlier and several EMA time
    # constants past the last real TF update) should sit close to 2.0 m/s,
    # with ego.valid == 1 throughout (TF was flowing).
    tail = samples[-20:]
    assert all(s[5] == 1.0 for s in tail), f"expected ego.valid==1 in tail, got {tail}"
    tail_avg = sum(s[4] for s in tail) / len(tail)
    assert abs(tail_avg - SPEED_MPS) < 0.4, (
        f"expected smoothed speed to converge near {SPEED_MPS} m/s, tail avg={tail_avg:.3f} "
        f"(tail speeds={[s[4] for s in tail]})")


def test_robot_speed_topic_preferred_over_tf_diff():
    """Spec §7 (docs/superpowers/specs/2026-08-18-visual-mode-design.md:264):
    ego speed PREFERS /robot/feedback/robot_speed_mps over the TF
    finite-difference fallback. TF motion here implies ~2.0 m/s (same
    fixture rate as above); the topic reports a deliberately different
    9.0 m/s -- the reported ~/ego_state speed must track the topic, not
    the TF-diff value, proving the preference (not just that both paths
    individually work, which the test above already covers).
    """
    if not os.path.isdir(INSTALL_DIR):
        pytest.skip("ros/install/ not found -- build with colcon_build.sh first")

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from std_msgs.msg import Float32, Float64MultiArray
    from tf2_ros import TransformBroadcaster

    TOPIC_SPEED_MPS = 9.0

    class SpeedFixtureNode(Node):
        def __init__(self):
            super().__init__("robot_speed_topic_fixture")
            self.broadcaster = TransformBroadcaster(self)
            self.speed_pub = self.create_publisher(
                Float32, "/robot/feedback/robot_speed_mps", 10)
            self.samples = []
            self.create_subscription(Float64MultiArray, "/visualization_node/ego_state",
                                     self._on_ego, 20)

        def _on_ego(self, msg: Float64MultiArray):
            self.samples.append(list(msg.data))

        def publish_tf(self, x: float):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)

        def publish_speed(self, v: float):
            m = Float32()
            m.data = v
            self.speed_pub.publish(m)

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node "
        f"--ros-args -p out_width:={OUT_W} -p out_height:={OUT_H} "
        f"-p initial_mode:=1 -p ego_speed_smoothing_alpha:={SMOOTHING_ALPHA}"
    )
    node_proc = _popen(cmd)
    fixture = None
    try:
        if not _wait_for_start(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            pytest.fail(f"node exited early (code {node_proc.returncode}):\n{stderr[-2000:]}")
        assert _lifecycle("configure"), "configure transition failed"
        assert _lifecycle("activate"), "activate transition failed"

        rclpy.init()
        fixture = SpeedFixtureNode()

        for i in range(30):  # 1.5s -- TF alone implies ~2.0 m/s, topic says 9.0
            fixture.publish_tf(SPEED_MPS * i * TF_DT)
            fixture.publish_speed(TOPIC_SPEED_MPS)
            rclpy.spin_once(fixture, timeout_sec=0.0)
            time.sleep(TF_DT)

        t_end = time.time() + 0.5
        while time.time() < t_end:
            rclpy.spin_once(fixture, timeout_sec=0.05)

        samples = fixture.samples
    finally:
        if fixture is not None:
            fixture.destroy_node()
            rclpy.shutdown()
        _kill(node_proc)

    assert len(samples) > 5, f"expected ~/ego_state samples, got {len(samples)}"
    tail = samples[-10:]
    tail_avg = sum(s[4] for s in tail) / len(tail)
    assert abs(tail_avg - TOPIC_SPEED_MPS) < 0.5, (
        f"expected reported speed to track /robot/feedback/robot_speed_mps "
        f"({TOPIC_SPEED_MPS} m/s), got tail avg={tail_avg:.3f} -- TF-diff "
        f"fallback used instead of the preferred topic (tail speeds="
        f"{[s[4] for s in tail]})")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
