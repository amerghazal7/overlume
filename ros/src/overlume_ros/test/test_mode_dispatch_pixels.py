#!/usr/bin/env python3
"""Node-level live-pixel mode-dispatch check (Task 6 / VM-095 Step 0).

Closes the one signoff.md row Task 4 (VM-093) left "Not started": node-level
bowl-visible/bowl-hidden pixel assertions for the per-mode dispatch, plus a
node-level "autonomy layers do not render in BOWL with live data flowing"
check. `test_scene_assembly.cpp`'s `compose_layer_gates()` GTest coverage
already proves the MASK/predicate logic; this is the live-render proof that
was still missing, against a REAL running node with REAL camera frames (this
epic's own fixture bag), not a synthetic stand-in.

Mechanism: mean-abs-pixel-diff between short averaged capture windows (same
metric flicker_capture.py already established in this repo), not exact-color
matching -- object/theme render colors are class/theme-driven, not read back
from the published marker, so a diff-based check is the robust one:

  1. A synthetic MarkerArray object (`/perception/dynamic_objects_list`,
     ns=dynamic_objects_bbox, frame_id="map" so `FrameTransformer::lookup()`
     short-circuits to identity -- no TF tree needed) is toggled on/off while
     the node stays in one render_mode, and the two capture windows are
     diffed:
       - FREE_LOOK: object ON vs OFF differs a lot -- autonomy renders here.
       - BOWL:      object ON vs OFF differs ~nothing -- autonomy is
         suppressed here (the actual regression this check guards).
       - FREE_LOOK + Surround Stitching (bowl profile): object ON vs OFF
         differs a lot AGAIN -- the autonomy scene is back, alongside bowl.
  2. BOWL-vs-FREE_LOOK (object OFF in both) differs a lot -- bowl content
     replaces the autonomy scene's own look, not a no-op mode switch.

Run (ROS + this repo's ros install sourced first, from a box with a GPU
-- render_frame() needs one):
    source /opt/ros/humble/setup.bash
    source ros/install/setup.bash
    python3 ros/src/overlume_ros/test/test_mode_dispatch_pixels.py \\
        [--bag ~/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11]

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install or the
fixture bag isn't present -- same convention as this directory's other
standalone node-level tests (test_bowl_node_params.py, test_mode_dispatch.py).

NOT CMake-registered -- same convention as every other *.py file in this
test/ directory (none are ament_add_pytest_test targets).
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
INSTALL_DIR = os.path.join(REPO_ROOT, "ros", "install")
NODE_NAME = "/overlume_node"
# v2 (stack_v2_full_sensors_2026-09-09) no longer exists on this box (user
# decision, 2026-09-11) -- stack_v3_full_sensors_2026-09-11 is the new
# default fixture bag for this epic (67s, bm/br cameras under-deliver at
# 67%/77% of the best camera -- a known recording deficit, not a rig bug;
# harmless here since this check only needs cameras flowing, not full-rate
# parity across all six).
DEFAULT_BAG = os.path.expanduser("~/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11")
OUT_W, OUT_H = 320, 240

# render_mode_ int constants -- overlume_node.hpp's kRenderModeBowl/
# kRenderModeHybrid/kRenderModeFreeLook (1/2/3).
MODE_BOWL, MODE_FREE_LOOK = 1, 3

# Salience metric: COUNT of pixels whose per-pixel mean-abs-channel-diff
# exceeds PIXEL_DIFF_THRESH, not a whole-frame mean diff. A whole-frame mean
# (flicker_capture.py's own metric, fine for a carpet ribbon that covers a
# meaningful frame fraction) dilutes a car-class object's own footprint --
# measured directly against this rig's default virtual_pose/class rendering:
# a single "car"-classified box at this test's marker position changes
# ~1800 of 76800 px (320x240) by up to +28, for a whole-frame MEAN of only
# ~1.0 -- real, but too close to BOWL's own live-video frame-to-frame mean
# diff to threshold reliably. Counting only pixels that moved by a real
# amount is far more sensitive to "did an object's silhouette appear" while
# staying just as usable for the coarse BOWL-vs-FREE_LOOK content check.
PIXEL_DIFF_THRESH = 10.0    # per-pixel mean-abs-channel-diff to count as "changed"
MODE_CONTENT_COUNT = 2000   # BOWL vs FREE_LOOK changed-px floor (whole frame), object off in both
# ADDITIVE margins over each mode's own measured noise floor, not
# multiplicative -- Surround Stitching's own ROI noise floor (real bowl
# video showing through/around the object's screen position) measured
# ~1800px in this rig's own runs, the same ORDER as the object's own
# incremental signal (~3400px) -- a multiplicative factor big enough to
# tolerate BOWL's much larger noise floor (~70px, but seen up to ~4400px
# across runs) would never clear here, and one small enough to clear here
# would pass BOWL's noise as "visible" too. Additive margins, sized to each
# check's own actual measured gap (never the coin-flip a shared factor
# would be), separate the two cleanly.
VISIBLE_MARGIN = 500        # signal must clear noise_floor + this to count as "shown"
HIDDEN_MARGIN = 100         # signal must stay under noise_floor + this to count as "hidden"

# The object-ON-vs-OFF check is restricted to this ROI (y0, y1, x0, x1),
# not the whole frame: measured directly (a standalone bowl-disabled probe
# against this rig's default virtual_pose put the object's own footprint at
# y in [103,137], x in [139,201] out of 320x240) and padded generously. A
# whole-frame diff dilutes a ~2% -footprint object into noise once BOWL's
# own real six-camera video is added to the scene (that video changes tens
# of thousands of px between any two ~1s windows, regardless of the
# marker) -- restricting to this ROI keeps that same live-video noise a much
# smaller absolute count, without changing what's actually being asked
# ("did the object's own screen region change").
OBJ_ROI = (85, 155, 120, 220)  # y0, y1, x0, x1
OBJ_VISIBLE_COUNT = 100     # object ON vs OFF changed-px floor (ROI), in a mode that should show it
OBJ_HIDDEN_COUNT = 40       # object ON vs OFF changed-px floor (ROI), in a mode that should hide it


def _changed_px(a, b, roi=None):
    """Count of pixels whose per-pixel mean-abs-channel-diff between two
    (H,W,3) float32 frames exceeds PIXEL_DIFF_THRESH. `roi`, if given, is
    (y0, y1, x0, x1) and restricts the count to that crop."""
    import numpy as np
    if roi is not None:
        y0, y1, x0, x1 = roi
        a = a[y0:y1, x0:x1]
        b = b[y0:y1, x0:x1]
    per_px = np.abs(a - b).mean(axis=2)
    return int(np.count_nonzero(per_px > PIXEL_DIFF_THRESH))


def _popen(cmd: str) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd], start_new_session=True)


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


def _run(cmd: str, timeout: float = 15.0) -> subprocess.CompletedProcess:
    full = f"source /opt/ros/humble/setup.bash && {cmd}"
    return subprocess.run(["bash", "-c", full], capture_output=True, text=True, timeout=timeout)


def _lifecycle(transition: str, timeout: float = 20.0) -> bool:
    for _ in range(20):
        result = _run(f"ros2 lifecycle set {NODE_NAME} {transition}", timeout=timeout)
        if "Transitioning successful" in result.stdout:
            return True
        if "Transitioning failed" in result.stdout:
            print(f"  [lifecycle {transition}] FAILED: {result.stdout}\n{result.stderr}",
                  file=sys.stderr)
            return False
        time.sleep(1)
    return False


def _param_set(name: str, value_literal: str) -> bool:
    result = _run(f"ros2 param set {NODE_NAME} {name} {value_literal}")
    ok = result.returncode == 0 and "Set parameter successful" in result.stdout
    if not ok:
        print(f"  [param set {name}={value_literal}] stdout={result.stdout!r} "
              f"stderr={result.stderr!r}", file=sys.stderr)
    return ok


def _wait_running(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


class FrameAvg:
    """Subscribes /rendering/image, averages the next N frames it sees."""

    def __init__(self, node):
        import rclpy.qos as qos
        from sensor_msgs.msg import Image
        self._Image = Image
        self.count = 0
        self._sum = None
        self._collecting = False
        node.create_subscription(
            Image, "/rendering/image", self._on_image,
            qos.QoSProfile(depth=5, reliability=qos.ReliabilityPolicy.RELIABLE))

    def _on_image(self, msg):
        import numpy as np
        if not self._collecting:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self._sum is None:
            self._sum = arr.astype(np.float64).copy()
        else:
            self._sum += arr.astype(np.float64)
        self.count += 1

    def capture(self, executor, n_frames: int = 8, timeout_s: float = 10.0):
        import numpy as np
        self._sum = None
        self.count = 0
        self._collecting = True
        t0 = time.time()
        while self.count < n_frames and time.time() - t0 < timeout_s:
            executor.spin_once(timeout_sec=0.5)
        self._collecting = False
        if self.count == 0:
            raise RuntimeError("no frames captured within timeout")
        return (self._sum / self.count).astype(np.float32)


def _publish_marker(pub, add: bool):
    from builtin_interfaces.msg import Duration
    from visualization_msgs.msg import Marker, MarkerArray
    obj_id = 90001  # far outside any real bag track-id range

    def _base(ns: str) -> Marker:
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = obj_id
        m.action = Marker.ADD if add else Marker.DELETE
        # Empirically confirmed in view of this rig's default virtual_pose
        # (eye=(-4,0,3.5), target=(2,0,-0.5)): a 2m cube centered here lands
        # on-screen (a standalone bowl-disabled probe measured its exact
        # screen footprint, OBJ_ROI below).
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 2.0, 0.0, -0.5
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(sec=0, nanosec=0)  # forever, this test manages ADD/DELETE itself
        return m

    bbox = _base("dynamic_objects_bbox")
    bbox.type = Marker.CUBE
    bbox.scale.x = bbox.scale.y = bbox.scale.z = 2.0
    bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1.0, 0.05, 0.05, 1.0

    # A bbox with no matching (non-empty) TEXT marker of the same track id
    # is dropped as malformed (dynamic_objects.cpp's ingest(): "bbox with no
    # matching text" -- has_bbox && !has_text at the end of the message).
    # "car" hits the class_inference keyword table for a real, visible clay
    # asset/color instead of the dimension-fallback default.
    text = _base("dynamic_objects_text")
    text.type = Marker.TEXT_VIEW_FACING
    text.text = "car"
    text.scale.z = 0.5

    arr = MarkerArray()
    arr.markers.append(bbox)
    arr.markers.append(text)
    pub.publish(arr)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default=DEFAULT_BAG)
    args = ap.parse_args()

    if not os.path.isdir(INSTALL_DIR):
        print("SKIP: ros/install not built -- run colcon_build.sh first.")
        return 0
    if not os.path.isdir(args.bag):
        print(f"SKIP: fixture bag not found at {args.bag} -- this check needs the real "
              f"six-camera bag to genuinely configure the bowl.")
        return 0

    log_fd, log_path = tempfile.mkstemp(prefix="viz_mode_dispatch_pixels_", suffix=".log")
    os.close(log_fd)

    vparams = _run(
        "ros2 pkg prefix overlume_ros"
    ).stdout.strip() + "/share/overlume_ros/config/default_params.yaml"

    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"--params-file {vparams} "
        f"-p out_width:={OUT_W} -p out_height:={OUT_H} -p use_sim_time:=true "
        f"-p profile:=urban -p bowl_enabled:=true -p render_mode:={MODE_FREE_LOOK} "
        # initial_mode:=3 is harmless/ignored post-cutover (the param no
        # longer exists once Task 6 Step 3 lands) and required pre-cutover
        # (the still-live mux gate, active_mode_ != 3, would otherwise
        # suppress every frame regardless of render_mode_) -- this launch
        # line works unmodified on either side of the cutover commit.
        f"-p initial_mode:=3 "
        f"> {log_path} 2>&1")
    viz_proc = _popen(viz_cmd)
    bag_proc = None

    try:
        if not _wait_running(viz_proc):
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: overlume_node exited early:\n{tail}", file=sys.stderr)
            return 1
        if not _lifecycle("configure") or not _lifecycle("activate"):
            print("FAIL: configure/activate failed.", file=sys.stderr)
            return 1

        # --topics restricts playback to ONLY the six cameras' raw_images/
        # camera_info -- this bag (stack_v2_full_sensors_2026-09-09) is
        # richer than the epic's own camera-bag framing suggests: it ALSO
        # carries ~30 real MarkerArray/Path topics (including a REAL
        # /perception/dynamic_objects_list at ~9 Hz, 1889 msgs), which would
        # otherwise swamp this test's own synthetic marker with real,
        # constantly-changing object content on the very topic this test
        # publishes to -- confirmed empirically (a first draft of this test
        # measured FREE_LOOK's own back-to-back noise floor at ~30, higher
        # than the synthetic marker's own on/off signal). Camera-only
        # playback makes this test's marker the SOLE autonomy-content
        # driver in FREE_LOOK, and isolates BOWL's noise floor to genuine
        # camera/video motion, which is what that mode's own check needs.
        camera_topics = " ".join(
            f"/{cam}_camera/{kind}"
            for cam in ("fl", "fm", "fr", "bl", "bm", "br")
            for kind in ("raw_images", "camera_info"))
        bag_cmd = (f"source /opt/ros/humble/setup.bash && "
                   f"ros2 bag play {args.bag} --clock --rate 1.0 "
                   f"--topics {camera_topics} < /dev/null")
        bag_proc = _popen(bag_cmd)

        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        import numpy as np

        rclpy.init()
        node = Node("mode_dispatch_pixel_probe")
        marker_pub = node.create_publisher(
            __import__("visualization_msgs.msg", fromlist=["MarkerArray"]).MarkerArray,
            "/perception/dynamic_objects_list", 10)
        avg = FrameAvg(node)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def publish_for(add: bool, seconds: float = 1.0):
            t0 = time.time()
            while time.time() - t0 < seconds:
                _publish_marker(marker_pub, add)
                executor.spin_once(timeout_sec=0.1)
                time.sleep(0.1)

        def capture_mode(mode_label: str):
            """no_obj_1 -> no_obj_2 -> obj, back to back. Returns (noise_floor,
            signal) where noise_floor = diff(no_obj_1, no_obj_2) [pure bag/
            render motion, marker untouched throughout] and signal =
            diff(no_obj_2, obj) [the SAME kind of gap, but with the marker
            toggled on] -- isolates the marker's own effect from ordinary
            live-video motion between two capture windows."""
            publish_for(add=False, seconds=1.0)
            no_obj_1 = avg.capture(executor)
            publish_for(add=False, seconds=1.0)
            no_obj_2 = avg.capture(executor)
            publish_for(add=True, seconds=1.0)
            obj = avg.capture(executor)
            noise_floor = _changed_px(no_obj_2, no_obj_1, roi=OBJ_ROI)
            signal = _changed_px(obj, no_obj_2, roi=OBJ_ROI)
            print(f"INFO: {mode_label}: noise_floor={noise_floor}px "
                  f"object-on-vs-off signal={signal}px (>{PIXEL_DIFF_THRESH} per-px)")
            return noise_floor, signal, no_obj_2

        try:
            # Let the bag reach steady playback + all cameras' first
            # CameraInfo/image -- same warm-up bowl_perf_gate.sh uses.
            print("INFO: warm-up (15s) -- bag reaching steady playback, bowl configuring ...")
            t0 = time.time()
            while time.time() - t0 < 15.0:
                executor.spin_once(timeout_sec=0.5)

            # ---- FREE_LOOK ----
            free_noise, d_free, free_ref = capture_mode("FREE_LOOK")
            if d_free < max(OBJ_VISIBLE_COUNT, free_noise + VISIBLE_MARGIN):
                print(f"FAIL: FREE_LOOK should show the autonomy object (signal {d_free}px "
                      f"did not clear its own noise floor {free_noise}px + {VISIBLE_MARGIN} "
                      f"nor the {OBJ_VISIBLE_COUNT}px floor) -- "
                      f"see {log_path}", file=sys.stderr)
                return 1
            print("PASS (1/3): FREE_LOOK renders the autonomy object (live pixel diff, "
                  "noise-floor-normalized).")

            # ---- switch to BOWL ----
            if not _param_set("render_mode", str(MODE_BOWL)):
                print("FAIL: render_mode -> BOWL was rejected.", file=sys.stderr)
                return 1
            time.sleep(1.0)  # let the mode switch's next tick land

            bowl_noise, d_bowl, bowl_ref = capture_mode("BOWL")
            if d_bowl >= max(OBJ_HIDDEN_COUNT, bowl_noise + HIDDEN_MARGIN):
                print(f"FAIL: BOWL must NOT render the autonomy object (signal {d_bowl}px "
                      f"exceeds its own noise floor {bowl_noise}px + {HIDDEN_MARGIN} "
                      f"and the {OBJ_HIDDEN_COUNT}px floor) -- live data was flowing on "
                      f"/perception/dynamic_objects_list the whole time; this bowl-mode "
                      f"noise floor is REAL six-camera video motion from the fixture bag, "
                      f"measured fresh in this same run, not an assumed constant; "
                      f"see {log_path}", file=sys.stderr)
                return 1
            print("PASS (2/3): BOWL suppresses the autonomy object while live data flows "
                  "(node-level pixel proof, not just the compose_layer_gates() mask unit "
                  "test) -- object-on-vs-off signal stays within this mode's own measured "
                  "live-video noise floor.")

            d_mode = _changed_px(bowl_ref, free_ref)
            print(f"INFO: BOWL-vs-FREE_LOOK (object off in both) changed px = {d_mode}")
            if d_mode < MODE_CONTENT_COUNT:
                print(f"FAIL: BOWL's own content should visibly differ from FREE_LOOK's "
                      f"(expected >= {MODE_CONTENT_COUNT}px changed, got {d_mode})",
                      file=sys.stderr)
                return 1
            print("PASS (3/3, part a): BOWL content visibly differs from FREE_LOOK "
                  "(bowl replaces the autonomy scene's own look).")

            # ---- FREE_LOOK + Surround Stitching (bowl profile): object back ----
            if not _param_set("render_mode", str(MODE_FREE_LOOK)):
                print("FAIL: render_mode -> FREE_LOOK was rejected.", file=sys.stderr)
                return 1
            if not _param_set("layer_surround_stitching", "true"):
                print("FAIL: layer_surround_stitching -> true was rejected.", file=sys.stderr)
                return 1
            time.sleep(1.0)

            ss_noise, d_ss_obj, ss_ref = capture_mode("FREE_LOOK+SurroundStitching")
            if d_ss_obj < max(OBJ_VISIBLE_COUNT, ss_noise + VISIBLE_MARGIN):
                print(f"FAIL: FREE_LOOK+Surround Stitching should still show the autonomy "
                      f"object alongside bowl content (signal {d_ss_obj}px did not clear "
                      f"noise floor {ss_noise}px + {VISIBLE_MARGIN} nor the "
                      f"{OBJ_VISIBLE_COUNT}px floor)", file=sys.stderr)
                return 1
            d_ss_bowl = _changed_px(ss_ref, free_ref)
            print(f"INFO: FREE_LOOK+SurroundStitching-vs-plain-FREE_LOOK (object off in "
                  f"both) changed px = {d_ss_bowl}")
            if d_ss_bowl < MODE_CONTENT_COUNT:
                print(f"FAIL: Surround Stitching should visibly add bowl content on top of "
                      f"FREE_LOOK (expected >= {MODE_CONTENT_COUNT}px changed, got "
                      f"{d_ss_bowl})", file=sys.stderr)
                return 1
            print("PASS (3/3, part b): FREE_LOOK+Surround Stitching (bowl profile) shows "
                  "the autonomy object ALONGSIDE bowl content -- neither hides the other.")

            print("PASS: node-level mode-dispatch live-pixel check passed "
                  "(signoff.md's deferred row, closed).")
            return 0
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        if bag_proc is not None:
            _kill(bag_proc)
        _kill(viz_proc)
        try:
            os.remove(log_path)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
