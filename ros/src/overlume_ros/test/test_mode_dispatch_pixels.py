#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

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
        [--bag ~/overlume-fixtures/stack_v3_full_sensors_2026-09-11]

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
DEFAULT_BAG = os.path.expanduser("~/overlume-fixtures/stack_v3_full_sensors_2026-09-11")
OUT_W, OUT_H = 320, 240

MODE_BOWL, MODE_HYBRID, MODE_FREE_LOOK = 1, 2, 3

# Hybrid composite checks: mean-abs-diff (0..255) of mode 2 vs mode 1, and of the `hybrid` vs
# `bowl` stitching profile. Before the fix hybrid was "visually indistinguishable" from bowl
# (VM-094 runbook: 0.493). Calibration (this box, GPU, fixture bag; see the B2 commit body):
#   pre-fix (B2 node feed reverted, 3 runs): (i) 5.28-5.49  (ii) 5.87-6.43  (== the noise)
#   fixed (3 runs):                          (i) 13.17-13.75 (ii) 18.82-19.06 (noise 5.4 / 5.9)
# HYBRID_DELTA_FLOOR = 8.0
HYBRID_ATTEMPTS = 3  # best of N: the fixture bag is live video (one run in ~6 dipped to 9.5) (below the geometric means 8.4 / 10.8, above the pre-fix max 6.43):
# it FAILS pre-fix and PASSES fixed. The `noise * 2` term in hybrid_delta() scales it with the
# bag's own video motion (windows are diffed back to back via an in-process param set).
HYBRID_DELTA_FLOOR = 8.0
HYBRID_ATTEMPTS = 3  # best of N: the fixture bag is live video (one run in ~6 dipped to 9.5)

PIXEL_DIFF_THRESH = 10.0
MODE_CONTENT_COUNT = 2000
VISIBLE_MARGIN = 500
HIDDEN_MARGIN = 100

OBJ_ROI = (85, 155, 120, 220)
OBJ_VISIBLE_COUNT = 100
OBJ_HIDDEN_COUNT = 40

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

def _mean_abs(a, b):
    import numpy as np
    return float(np.abs(a - b).mean())

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
    obj_id = 90001

    def _base(ns: str) -> Marker:
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = obj_id
        m.action = Marker.ADD if add else Marker.DELETE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 2.0, 0.0, -0.5
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(sec=0, nanosec=0)
        return m

    bbox = _base("dynamic_objects_bbox")
    bbox.type = Marker.CUBE
    bbox.scale.x = bbox.scale.y = bbox.scale.z = 2.0
    bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1.0, 0.05, 0.05, 1.0

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
    ap.add_argument("--dump-dir", default=None,
                    help="write each averaged hybrid-check window as a PNG here")
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
        f"-p initial_mode:=3 "
        f"-p pointcloud_topic:=/iv_points_fusion -p hybrid_enabled:=true "
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

        camera_topics = " ".join(
            f"/{cam}_camera/{kind}"
            for cam in ("fl", "fm", "fr", "bl", "bm", "br")
            for kind in ("raw_images", "camera_info"))
        camera_topics += " /iv_points_fusion"
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
            print("INFO: warm-up (15s) -- bag reaching steady playback, bowl configuring ...")
            t0 = time.time()
            while time.time() - t0 < 15.0:
                executor.spin_once(timeout_sec=0.5)

            free_noise, d_free, free_ref = capture_mode("FREE_LOOK")
            if d_free < max(OBJ_VISIBLE_COUNT, free_noise + VISIBLE_MARGIN):
                print(f"FAIL: FREE_LOOK should show the autonomy object (signal {d_free}px "
                      f"did not clear its own noise floor {free_noise}px + {VISIBLE_MARGIN} "
                      f"nor the {OBJ_VISIBLE_COUNT}px floor) -- "
                      f"see {log_path}", file=sys.stderr)
                return 1
            print("PASS (1/3): FREE_LOOK renders the autonomy object (live pixel diff, "
                  "noise-floor-normalized).")

            if not _param_set("render_mode", str(MODE_BOWL)):
                print("FAIL: render_mode -> BOWL was rejected.", file=sys.stderr)
                return 1
            time.sleep(1.0)

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

            # ---- hybrid composite: lidar splats must visibly differ from the bare bowl ----
            failed = False  # all hybrid checks run (calibration needs every number)
            publish_for(add=False, seconds=1.0)

            from rcl_interfaces.msg import Parameter as RclParam, ParameterType, ParameterValue
            from rcl_interfaces.srv import SetParameters
            set_cli = node.create_client(SetParameters, f"{NODE_NAME}/set_parameters")
            if not set_cli.wait_for_service(timeout_sec=5.0):
                print("FAIL: set_parameters service unavailable.", file=sys.stderr)
                return 1

            def fast_set(name, value):
                """In-process parameter set (ms, vs ~1-2 s for `ros2 param set`): the bag keeps
                moving, so the gap between the windows being diffed must stay tiny."""
                pv = ParameterValue()
                if isinstance(value, str):
                    pv.type, pv.string_value = ParameterType.PARAMETER_STRING, value
                elif isinstance(value, bool):
                    pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, value
                else:
                    pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, value
                req = SetParameters.Request()
                req.parameters = [RclParam(name=name, value=pv)]
                fut = set_cli.call_async(req)
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 5.0:
                    executor.spin_once(timeout_sec=0.05)
                return fut.done() and all(r.successful for r in fut.result().results)

            def dump(name, img):
                if args.dump_dir:
                    from PIL import Image
                    os.makedirs(args.dump_dir, exist_ok=True)
                    Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(
                        os.path.join(args.dump_dir, name + ".png"))

            def hybrid_delta(label, base_a, base_b, hyb):
                noise = _mean_abs(base_a, base_b)
                delta = _mean_abs(hyb, base_b)
                need = max(HYBRID_DELTA_FLOOR, noise * 2)
                print(f"INFO: {label}: noise={noise:.3f} hybrid-vs-bowl={delta:.3f} "
                      f"(need >= {need:.3f})")
                return delta >= need

            if not _param_set("render_mode", str(MODE_BOWL)):
                print("FAIL: render_mode -> BOWL was rejected.", file=sys.stderr)
                return 1
            time.sleep(1.0)
            # The bag keeps moving, so "noise" must span the same protocol as the signal:
            # window, one param set, window. The second set is a no-op for the noise pair.
            ok_i = False
            for attempt in range(HYBRID_ATTEMPTS):  # live video: any attempt clearing the bar
                fast_set("render_mode", MODE_BOWL)
                b1 = avg.capture(executor)
                fast_set("render_mode", MODE_BOWL)
                b2 = avg.capture(executor)
                if not fast_set("render_mode", MODE_HYBRID):
                    print("FAIL: render_mode -> HYBRID was rejected.", file=sys.stderr)
                    return 1
                h = avg.capture(executor)
                for n_, im in (("mode1_a", b1), ("mode1_b", b2), ("mode2_hybrid", h)):
                    dump(n_, im)
                if hybrid_delta("mode 2 vs mode 1", b1, b2, h):
                    ok_i = True
                    break
            if not ok_i:
                print("FAIL: mode 2 (hybrid) is not visibly different from mode 1 (bowl) -- "
                      f"lidar splats are not composited over the bowl; see {log_path}",
                      file=sys.stderr)
                failed = True
            else:
                print("PASS (hybrid i): mode 2 lidar splats visibly differ from the bare bowl.")

            if not _param_set("render_mode", str(MODE_FREE_LOOK)):
                print("FAIL: render_mode -> FREE_LOOK was rejected.", file=sys.stderr)
                return 1
            if not _param_set("layer_surround_stitching", "true") or \
               not _param_set("surround_stitching_profile", "bowl"):
                print("FAIL: stitching bowl profile was rejected.", file=sys.stderr)
                return 1
            time.sleep(1.0)
            ok_ii = False
            for attempt in range(HYBRID_ATTEMPTS):
                fast_set("surround_stitching_profile", "bowl")
                s1 = avg.capture(executor)
                fast_set("surround_stitching_profile", "bowl")  # no-op: matches the signal gap
                s2 = avg.capture(executor)
                if not fast_set("surround_stitching_profile", "hybrid"):
                    print("FAIL: surround_stitching_profile -> hybrid was rejected.",
                          file=sys.stderr)
                    return 1
                sh = avg.capture(executor)
                for n_, im in (("stitch_bowl_a", s1), ("stitch_bowl_b", s2),
                               ("stitch_hybrid", sh)):
                    dump(n_, im)
                if hybrid_delta("stitching hybrid vs bowl profile", s1, s2, sh):
                    ok_ii = True
                    break
            if not ok_ii:
                print("FAIL: FREE_LOOK + stitching profile 'hybrid' is not visibly different "
                      "from the 'bowl' profile", file=sys.stderr)
                failed = True
            with open(log_path) as f:
                node_log = f.read()
            if "hybrid: profile row /iv_points_fusion suppressed" not in node_log:
                hy = [l for l in node_log.splitlines() if "hybrid" in l or "point_cloud" in l]
                print("FAIL: node never logged the profile-row de-dup "
                      "('hybrid: profile row /iv_points_fusion suppressed'); hybrid log lines:\n"
                      + "\n".join(hy[-15:]), file=sys.stderr)
                failed = True
            if failed:
                return 1
            print("PASS (hybrid ii): FREE_LOOK + stitching 'hybrid' profile differs from "
                  "'bowl' and the duplicate profile row is suppressed.")

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
