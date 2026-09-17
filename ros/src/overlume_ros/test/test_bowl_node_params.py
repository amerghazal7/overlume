#!/usr/bin/env python3
"""Bowl-config node-level regression test (VM-091 gate close-out, findings 1/2/4).

Exercises three things that need a REAL running node (no in-process rclcpp
node-harness convention exists in this package -- see test_camera_ingest.cpp's
own comment on why the CameraIngest wrapper itself is only exercised end to
end by running the node), so this follows the same standalone-subprocess
pattern test_theme_ws.py already established rather than inventing a second
harness shape:

  1. **Finding 1's regression check**: on_configure()/on_activate() with
     DEFAULT params (no --params-file, so image_topics/info_topics are the
     declared empty-vector defaults and bowl_enabled defaults false) must
     succeed and the node must stay alive -- this is the reviewer's own
     repro for the segfault finding 1 fixed (CameraIngest's ctor used to
     index image_topics[i]/info_topics[i] for i in [0, n_cameras)
     UNCONDITIONALLY, regardless of bowl_enabled_, so a default-params
     configure walked off the end of two empty vectors before construction
     was gated on bowl_enabled_ + validated sizes).
  2. **Finding 2/Decision 3's clamp**: `ros2 param set fill_blind_zone true`
     and `exposure_match true` must be ACCEPTED (not rejected outright --
     the old node's on_params() falls through unmatched names as
     successful, and this clamp is enforced by forcing the internal member
     false + a WARN, not by rejecting the set_parameters() call) and must
     each log the "forced to false" WARN naming Decision 3.
  3. **Finding 4's layer_* round-trip**: a `layer_objects` param set must
     still succeed with the bowl on_params() handling installed in the SAME
     callback (rclcpp invokes every registered callback; a second one that
     doesn't fall through unmatched names would reintroduce the
     reject-unknowns bug this node's own on_params() already avoids).

Run (ROS + this repo's ros install sourced first):
    source /opt/ros/humble/setup.bash
    source ros/install/setup.bash
    python3 ros/src/overlume_ros/test/test_bowl_node_params.py

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install isn't
present, same convention as test_theme_ws.py.

NOT CMake-registered (`ament_add_gtest`/`ament_add_pytest_test` etc.) -- same
as test_theme_ws.py/test_vcam_contract.py/test_ego_anchored_vcam.py/
test_extra_topic_parity.py/test_tf_adapter.py in this same directory, none of
which appear in this package's CMakeLists.txt (grepped: no ament_add_test /
ament_add_pytest_test / add_test entry names any of them). Following that
existing (undocumented-elsewhere) convention rather than introducing this
package's first CMake-registered launch test.
"""

import os
import subprocess
import sys
import tempfile
import time

REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
INSTALL_DIR = os.path.join(REPO_ROOT, "ros", "install")
NODE_NAME = "/overlume_node"


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


def _lifecycle(transition: str, timeout: float = 15.0) -> bool:
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


def main() -> int:
    if not os.path.isdir(INSTALL_DIR):
        print("SKIP: ros/install not built -- run colcon_build.sh first.")
        return 0

    log_fd, log_path = tempfile.mkstemp(prefix="viz_bowl_node_params_", suffix=".log")
    os.close(log_fd)

    # Deliberately NO --params-file: image_topics/info_topics/bowl_enabled
    # are the declared defaults (empty vectors / false) -- finding 1's own
    # repro shape ("node with default params must configure cleanly").
    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"-p out_width:=160 -p out_height:=120 -p initial_mode:=3 "
        f"> {log_path} 2>&1")
    viz_proc = _popen(viz_cmd)

    try:
        if not _wait_running(viz_proc):
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: overlume_node exited early (default params):\n{tail}",
                  file=sys.stderr)
            return 1

        if not _lifecycle("configure"):
            print("FAIL: on_configure() with DEFAULT params (bowl_enabled=false, "
                  "image_topics/info_topics empty) should succeed -- this is finding 1's "
                  "own regression check.", file=sys.stderr)
            return 1
        if not _lifecycle("activate"):
            print("FAIL: on_activate() with default params failed.", file=sys.stderr)
            return 1

        time.sleep(1.0)
        if viz_proc.poll() is not None:
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: overlume_node crashed after activate with default params "
                  f"(the segfault finding 1 fixed):\n{tail}", file=sys.stderr)
            return 1
        print("PASS (1/3): on_configure()/on_activate() succeeded with default params, "
              "node still alive -- no segfault (finding 1).")

        # ---- Finding 2 / Decision 3: fill_blind_zone/exposure_match clamp ----
        if not _param_set("fill_blind_zone", "true"):
            print("FAIL: `ros2 param set fill_blind_zone true` was rejected outright -- "
                  "expected ACCEPTED (clamped internally to false + WARN, not rejected).",
                  file=sys.stderr)
            return 1
        if not _param_set("exposure_match", "true"):
            print("FAIL: `ros2 param set exposure_match true` was rejected outright -- "
                  "expected ACCEPTED (clamped internally to false + WARN, not rejected).",
                  file=sys.stderr)
            return 1
        time.sleep(0.5)

        # ---- Finding 4: layer_* param set still round-trips ----
        if not _param_set("layer_objects", "false"):
            print("FAIL: `ros2 param set layer_objects false` was rejected -- the bowl "
                  "on_params() handling must fall through unmatched names as successful, "
                  "same as every other extension of this callback.", file=sys.stderr)
            return 1
        time.sleep(0.5)

        with open(log_path) as f:
            log_text = f.read()
        if "fill_blind_zone: true requested but forced to false" not in log_text:
            print(f"FAIL: no fill_blind_zone clamp WARN found in the node's log:\n"
                  f"{log_text[-3000:]}", file=sys.stderr)
            return 1
        if "exposure_match: true requested but forced to false" not in log_text:
            print(f"FAIL: no exposure_match clamp WARN found in the node's log:\n"
                  f"{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (2/3): fill_blind_zone/exposure_match true requests were accepted and "
              "each logged the Decision-3 clamp-to-false WARN.")
        print("PASS (3/3): layer_objects param set still succeeded with bowl on_params() "
              "handling installed.")

        print("PASS: overlume_node bowl-param node-level regression test passed.")
        return 0
    finally:
        _kill(viz_proc)
        try:
            os.remove(log_path)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
