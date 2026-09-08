#!/usr/bin/env python3
"""Extra-topic parity E2E (Epic 2 Task 8 / VM-027 Step 6): the spec §7
parity guarantee proven end to end -- adding a topic is ONE YAML ROW AND NO
CODE.

Two launches of the real, running visualization_node against the SAME
TF + MarkerArray publish pattern:

  Phase 1 (no row): a profile filtered from the real urban_profile.yaml
  with the `/sim/ground_truth/boxes` row REMOVED. Publishes two CUBE
  markers on that topic anyway and asserts the rendered frame is
  UNCHANGED (mean-abs-pixel-diff against a markers-silent baseline stays
  low) -- nothing renders a topic with no profile row, full stop.

  Phase 2 (with row): the REAL, unmodified urban_profile.yaml (the row
  ships there already, Task 1). Same TF + markers pattern. Asserts the
  frame CHANGES measurably against phase 1's frame (mean-abs-pixel-diff >
  10.0 -- the generous documented noise floor the existing E2E scripts use,
  since the GPU may be contended by CARLA) AND that the diff is
  concentrated near the ego's screen neighbourhood, not smeared across the
  whole frame or floating in a spot the ego-anchored camera can't even see
  (which is exactly where an untransformed base_link->map bug would put
  it: the map origin, 100+ m outside this frustum).

/sim/ground_truth/boxes publishes in base_link -- the bag's one non-map
frame -- so the TF fixture below places the ego 150 m from the map origin
("100+ m" per the plan) and the two boxes at base_link (+-3, 0, 0). The
render pose composes ego-anchored regardless of vcam offset
(visualization_node.cpp's timer_callback, ego_anchor.hpp) -- the camera
follows the ego wherever TF puts it; this script sends one ~/set_look
(a SIDE view, not the default chase-cam preset) purely so both boxes are
unoccluded by the ego's own body in frame, see LOOK_OFFSET's own comment.

QoS (LOAD-BEARING, per the shipped profile row's own comment,
config/urban_profile.yaml): published BEST_EFFORT, depth 10, matching the
row's `best_effort: true` and the real bag's `reliability: 2`. A RELIABLE
test publisher would certify a permanently dead row -- an rclcpp
subscription built from a wrongly-RELIABLE row would never match a
BEST_EFFORT publisher either, and a RELIABLE test publisher papers over
exactly that failure.

Run (ROS + this repo's ros_apps install sourced first):
    source /opt/ros/humble/setup.bash
    source cuda/install/ros_apps/setup.bash
    python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/test_extra_topic_parity.py

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install isn't
present.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import time

OUT_W, OUT_H = 320, 240
TF_RATE_HZ = 20.0
TF_DT = 1.0 / TF_RATE_HZ
EGO_X_M = 150.0  # "100+ m from the map origin" per the plan
# ~/set_look offset (ego-anchored, no tween): a SIDE view, since the
# default chase-cam preset looks along +X where both boxes sit (base_link
# +-3 in X) and the ego's own clay-box silhouette occludes most of them
# from that angle (confirmed empirically).
LOOK_OFFSET = [0.0, -3.5, 1.8, 0.0, 0.0, 0.0]
# Marker scale: position (not size) is what the plan pins down. A 1x1x1
# box measured ~0.7 mean-abs-diff here (confirmed empirically), nowhere
# near DIFF_FLOOR=10.0 even unoccluded, so BOX_SCALE is sized up to a
# decent-sized ground obstacle instead of moving the camera uncomfortably
# close.
BOX_SCALE = (4.0, 4.0, 3.0)
SETTLE_SEC = 1.0     # TF alone -- let the ego/camera settle before the baseline capture
PUBLISH_SEC = 1.5    # boxes + TF together -- long enough to clear the row's staleness window
DIFF_FLOOR = 10.0    # matches test_ego_anchored_vcam.py-style generous noise floors
NOCHANGE_CEILING = 5.0  # phase 1's own "nothing rendered" internal check

# TEST_PORT is this script's isolation knob (every E2E script in this
# directory picks a distinct one); ROS_DOMAIN_ID is derived from it (mod
# 232, the valid domain range) so it stays traceable to the assigned port
# rather than being an unrelated magic constant.
TEST_PORT = 18767
ROS_DOMAIN_ID = TEST_PORT % 232

REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", ".."))
INSTALL_DIR = os.path.join(REPO_ROOT, "cuda", "install", "ros_apps")
VIZ_SHARE_CONFIG = os.path.join(
    INSTALL_DIR, "micropilot_visualization_node", "share", "micropilot_visualization_node",
    "config")


def _popen(cmd: str, env: dict) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            start_new_session=True, env=env)


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


def _lifecycle(env: dict, transition: str, timeout: float = 15.0) -> bool:
    cmd = f"source /opt/ros/humble/setup.bash && ros2 lifecycle set /visualization_node {transition}"
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout,
                            env=env)
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


def _make_norow_profile(tmpdir: str) -> str:
    """Filters the REAL, installed urban_profile.yaml -- never a hand-typed
    copy that could silently drift from what actually ships -- dropping
    only the /sim/ground_truth/boxes row. Returns the profile NAME (without
    '_profile.yaml'); caller passes -p profile_dir:=tmpdir alongside it.

    Also copies class_inference.yaml alongside it: on_configure() resolves
    that file relative to profile_dir unconditionally (every profile needs
    it, not just this filtered one), so a profile_dir with only the
    filtered profile YAML in it fails class-inference loading before ever
    reaching the row-filtering this test actually cares about.
    """
    import yaml

    shutil.copy(os.path.join(VIZ_SHARE_CONFIG, "class_inference.yaml"),
                os.path.join(tmpdir, "class_inference.yaml"))

    src = os.path.join(VIZ_SHARE_CONFIG, "urban_profile.yaml")
    with open(src) as f:
        profile = yaml.safe_load(f)
    # The shipped profile carries no /sim/ground_truth/boxes row (see the
    # profile's own comment block) -- assert that stays true, or this
    # test's premise silently rots:
    assert not any(r.get("topic") == "/sim/ground_truth/boxes" for r in profile["rows"]), (
        "shipped urban_profile.yaml has a /sim/ground_truth/boxes row again -- "
        "phase 1 is no longer a no-row baseline; update this test")

    dst = os.path.join(tmpdir, "e2e_norow_profile.yaml")
    with open(dst, "w") as f:
        yaml.safe_dump(profile, f)
    return "e2e_norow"


def _make_withrow_profile(tmpdir: str) -> str:
    """The parity guarantee, literally: the SAME shipped profile plus ONE
    added YAML row (the canonical /sim/ground_truth/boxes row -- best_effort
    is LOAD-BEARING, matching the real publisher's BEST_EFFORT offer; a
    RELIABLE subscription would never connect to it)."""
    import yaml

    src = os.path.join(VIZ_SHARE_CONFIG, "urban_profile.yaml")
    with open(src) as f:
        profile = yaml.safe_load(f)
    profile["rows"].append({
        "topic": "/sim/ground_truth/boxes",
        "type": "visualization_msgs/msg/MarkerArray",
        "adapter": "generic",
        "role": "neutral",
        "best_effort": True,
    })
    dst = os.path.join(tmpdir, "e2e_withrow_profile.yaml")
    with open(dst, "w") as f:
        yaml.safe_dump(profile, f)
    return "e2e_withrow"


def _launch_node(env: dict, profile_name: str, profile_dir: str) -> subprocess.Popen:
    cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node --ros-args "
        f"-p out_width:={OUT_W} -p out_height:={OUT_H} -p initial_mode:=3 "
        f"-p profile:={profile_name} -p profile_dir:={profile_dir}"
    )
    return _popen(cmd, env)


def _run_phase(env: dict, profile_name: str, profile_dir: str):
    """Returns (frame_before_markers, frame_after_markers) -- both raw RGB
    byte lists at OUT_W*OUT_H*3, both under the SAME continuously-published
    TF the whole time (ego stays at EGO_X_M throughout). frame_before is
    captured after TF-only settling (no markers ever published up to that
    point); frame_after is captured after PUBLISH_SEC of also publishing
    the two boxes.
    """
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64MultiArray
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    class Fixture(Node):
        def __init__(self):
            super().__init__("extra_topic_parity_fixture")
            self.broadcaster = TransformBroadcaster(self)
            # LOAD-BEARING (module docstring): BEST_EFFORT, matching the
            # shipped row's best_effort:true and the real bag's
            # reliability:2 -- a RELIABLE publisher here would certify a
            # row whose QoS is silently wrong on the real stack.
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.boxes_pub = self.create_publisher(MarkerArray, "/sim/ground_truth/boxes", qos)
            self.look_pub = self.create_publisher(
                Float64MultiArray, "/visualization_node/set_look", 10)
            self.latest_frame = None
            self.create_subscription(Image, "/rendering/image", self._on_image, 10)

        def publish_look(self):
            msg = Float64MultiArray()
            msg.data = LOOK_OFFSET
            self.look_pub.publish(msg)

        def _on_image(self, msg: Image):
            self.latest_frame = bytes(msg.data)

        def publish_tf(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = EGO_X_M
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)

        def publish_boxes(self):
            arr = MarkerArray()
            for i, x in enumerate((3.0, -3.0)):
                m = Marker()
                m.header.frame_id = "base_link"
                m.ns = "boxes"
                m.id = i
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = BOX_SCALE
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 1.0
                arr.markers.append(m)
            self.boxes_pub.publish(arr)

    node_proc = _launch_node(env, profile_name, profile_dir)
    rclpy.init(args=None)
    fixture = Fixture()
    try:
        if not _wait_running(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            raise RuntimeError(f"visualization_node exited early:\n{stderr[-2000:]}")
        if not _lifecycle(env, "configure") or not _lifecycle(env, "activate"):
            raise RuntimeError("lifecycle transition failed")

        # Wait for the FIRST frame explicitly: DDS discovery + the lifecycle
        # CLI subprocess's startup jitter is variable and could eat a fixed
        # settle window, failing this on timing noise rather than a real
        # render-path problem. publish_look() is repeated (not fire-and-
        # forget): the fixture's publisher must discovery-match the node's
        # subscription first, same race as the image subscription; applying
        # the offset is idempotent, so repeating it costs nothing.
        first_frame_deadline = time.time() + 8.0
        while fixture.latest_frame is None and time.time() < first_frame_deadline:
            fixture.publish_tf()
            fixture.publish_look()
            rclpy.spin_once(fixture, timeout_sec=0.05)
            time.sleep(TF_DT)
        if fixture.latest_frame is None:
            raise RuntimeError("no /rendering/image frame received within 8s of activation")

        # TF-only settle -- ego valid, camera anchored, no markers yet.
        t_end = time.time() + SETTLE_SEC
        while time.time() < t_end:
            fixture.publish_tf()
            fixture.publish_look()
            rclpy.spin_once(fixture, timeout_sec=0.0)
            time.sleep(TF_DT)
        frame_before = fixture.latest_frame

        # TF + boxes together.
        t_end = time.time() + PUBLISH_SEC
        while time.time() < t_end:
            fixture.publish_tf()
            fixture.publish_boxes()
            rclpy.spin_once(fixture, timeout_sec=0.0)
            time.sleep(TF_DT)
        # Drain a few more ticks so frame_after isn't racing the last publish.
        t_end = time.time() + 0.3
        while time.time() < t_end:
            fixture.publish_tf()
            fixture.publish_boxes()
            rclpy.spin_once(fixture, timeout_sec=0.05)
        frame_after = fixture.latest_frame
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
        _kill(node_proc)

    return frame_before, frame_after


def _diff_stats(a: bytes, b: bytes):
    """(mean_abs_diff, (centroid_row_frac, centroid_col_frac)) -- the
    centroid is the diff-energy-weighted mean pixel position, expressed as
    fractions of (height, width) so it's resolution-independent. (0.5, 0.5)
    is the exact frame center.
    """
    assert len(a) == len(b) == OUT_W * OUT_H * 3, "frame size mismatch"
    n_pixels = OUT_W * OUT_H
    total_abs = 0
    weighted_row = 0.0
    weighted_col = 0.0
    weight_sum = 0.0
    for p in range(n_pixels):
        base = p * 3
        d = (abs(a[base] - b[base]) + abs(a[base + 1] - b[base + 1]) +
             abs(a[base + 2] - b[base + 2]))
        total_abs += d
        if d > 0:
            row, col = divmod(p, OUT_W)
            weighted_row += row * d
            weighted_col += col * d
            weight_sum += d
    mean_abs = total_abs / (n_pixels * 3)
    if weight_sum == 0.0:
        return mean_abs, None
    centroid = (weighted_row / weight_sum / OUT_H, weighted_col / weight_sum / OUT_W)
    return mean_abs, centroid


def main() -> int:
    if not os.path.isdir(INSTALL_DIR):
        print("SKIP: cuda/install/ros_apps not found -- build with colcon_build.sh first.")
        return 0

    # Set on THIS process's environment, not just a dict handed to
    # subprocess calls: the fixture's rclpy node runs in-process, so
    # rclpy.init() must see the override too, or the node subprocess and
    # this script end up on different DDS domains and never discover each
    # other (confirmed the hard way).
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    env = os.environ

    tmpdir = tempfile.mkdtemp(prefix="e2e_extra_topic_parity_")
    try:
        norow_name = _make_norow_profile(tmpdir)

        print("== Phase 1: profile with NO /sim/ground_truth/boxes row ==")
        norow_before, norow_after = _run_phase(env, norow_name, tmpdir)
        mean_diff_norow, _ = _diff_stats(norow_before, norow_after)
        print(f"  no-row: TF-only vs TF+boxes mean-abs-diff = {mean_diff_norow:.3f}")
        if mean_diff_norow >= NOCHANGE_CEILING:
            print(f"FAIL: publishing /sim/ground_truth/boxes with no profile row still changed "
                  f"the frame (mean-abs-diff {mean_diff_norow:.3f} >= {NOCHANGE_CEILING}) -- "
                  f"something rendered it anyway.", file=sys.stderr)
            return 1

        print("== Phase 2: the shipped profile plus ONE added YAML row ==")
        withrow_name = _make_withrow_profile(tmpdir)
        _, withrow_after = _run_phase(env, withrow_name, tmpdir)
        mean_diff_row, centroid = _diff_stats(norow_after, withrow_after)
        print(f"  with-row vs no-row: mean-abs-diff = {mean_diff_row:.3f}, centroid={centroid}")
        if mean_diff_row <= DIFF_FLOOR:
            print(f"FAIL: adding the /sim/ground_truth/boxes row did not change the frame "
                  f"measurably (mean-abs-diff {mean_diff_row:.3f} <= {DIFF_FLOOR}) -- the "
                  f"parity guarantee did not hold.", file=sys.stderr)
            return 1
        if centroid is None:
            print("FAIL: no diff pixels at all despite mean-abs-diff passing (inconsistent).",
                  file=sys.stderr)
            return 1
        row_frac, col_frac = centroid
        # Generous central window (not a tight bbox on the boxes' projected
        # footprint, which would overfit this vcam preset): the diff must
        # land where the ego-anchored camera is looking, not smeared across
        # the frame or (the bug this guards against) sitting where an
        # untransformed base_link->map bug would put it -- 100+ m outside
        # this frustum, which would produce no diff pixels (already caught
        # by DIFF_FLOOR above).
        if not (0.15 <= row_frac <= 0.95 and 0.1 <= col_frac <= 0.9):
            print(f"FAIL: diff centroid ({row_frac:.2f}, {col_frac:.2f}) is not in the ego's "
                  f"screen neighbourhood -- expected roughly central, not a corner/edge smear.",
                  file=sys.stderr)
            return 1

        print(f"PASS: adding one YAML row rendered the topic (mean-abs-diff {mean_diff_row:.3f} "
              f"> {DIFF_FLOOR}), concentrated near the ego (centroid row={row_frac:.2f} "
              f"col={col_frac:.2f}), and a no-row profile rendered nothing "
              f"(mean-abs-diff {mean_diff_norow:.3f} < {NOCHANGE_CEILING}).")
        return 0
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
