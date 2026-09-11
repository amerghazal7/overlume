#!/usr/bin/env python3
"""Local render-mode / Surround Stitching param regression test (Task 4 /
VM-093).

Exercises what the mode-content-mask GTest cases (test_scene_assembly.cpp)
can't reach -- the actual ROS param surface -- with a REAL running node, same
standalone-subprocess pattern test_bowl_node_params.py/test_theme_ws.py
already established for this package (no in-process rclcpp node-harness
convention exists here):

  1. **render_mode accepts 1/2/3, rejects out of range.** `ros2 param set
     render_mode 1|2|3` must succeed (BOWL/HYBRID/FREE_LOOK); `render_mode 0`
     and `render_mode 4` must be REJECTED (res.successful=false in on_params()
     -- a bad mode request, not a value to silently clamp the way the
     declare-time default does).
  2. **layer_surround_stitching round-trips like every other layer_* bool.**
  3. **surround_stitching_profile accepts 'bowl'/'hybrid', rejects anything
     else** (e.g. 'lidar') -- same reject-not-clamp shape as render_mode.
  4. **render_mode is genuinely separate from the mux's active_mode_:** none
     of the above ever touches /rendering/set_mode, and the node stays alive
     throughout (this file's own repro for "no ROS message exchanged for a
     mode switch").

Run (ROS + this repo's ros_apps install sourced first):
    source /opt/ros/humble/setup.bash
    source cuda/install/ros_apps/setup.bash
    python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/test_mode_dispatch.py

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install isn't
present, same convention as test_bowl_node_params.py/test_theme_ws.py.

NOT CMake-registered -- same as test_bowl_node_params.py/test_theme_ws.py/
test_vcam_contract.py/test_ego_anchored_vcam.py/test_extra_topic_parity.py/
test_tf_adapter.py in this same directory.
"""

import os
import subprocess
import sys
import tempfile
import time

REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", ".."))
CUDA_ROOT = os.path.join(REPO_ROOT, "cuda")
INSTALL_DIR = os.path.join(CUDA_ROOT, "install", "ros_apps")
NODE_NAME = "/visualization_node"


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
    return result.returncode == 0 and "Set parameter successful" in result.stdout


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

    log_fd, log_path = tempfile.mkstemp(prefix="viz_mode_dispatch_", suffix=".log")
    os.close(log_fd)

    # initial_mode:=3 so this node is the mux-authoritative renderer
    # throughout (Decision 7 untouched) -- every check below flips ONLY the
    # local render_mode/layer_surround_stitching/surround_stitching_profile
    # params, never /rendering/set_mode.
    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node --ros-args "
        f"-p out_width:=160 -p out_height:=120 -p initial_mode:=3 "
        f"> {log_path} 2>&1")
    viz_proc = _popen(viz_cmd)

    try:
        if not _wait_running(viz_proc):
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: visualization_node exited early:\n{tail}", file=sys.stderr)
            return 1

        if not _lifecycle("configure") or not _lifecycle("activate"):
            print("FAIL: configure/activate failed.", file=sys.stderr)
            return 1

        # ---- 1. render_mode: accepts 1/2/3 ----
        for mode in (1, 2, 3):
            if not _param_set("render_mode", str(mode)):
                print(f"FAIL: `ros2 param set render_mode {mode}` was rejected -- "
                      f"expected accepted.", file=sys.stderr)
                return 1
        print("PASS (1a/4): render_mode accepts 1, 2, 3.")

        # ---- 1b. render_mode: rejects out of range ----
        for bad in (0, 4):
            if _param_set("render_mode", str(bad)):
                print(f"FAIL: `ros2 param set render_mode {bad}` was ACCEPTED -- "
                      f"expected rejected (out of range 1-3).", file=sys.stderr)
                return 1
        print("PASS (1b/4): render_mode rejects 0 and 4.")

        # ---- 2. layer_surround_stitching round-trips ----
        if not _param_set("layer_surround_stitching", "true"):
            print("FAIL: `ros2 param set layer_surround_stitching true` was rejected.",
                  file=sys.stderr)
            return 1
        if not _param_set("layer_surround_stitching", "false"):
            print("FAIL: `ros2 param set layer_surround_stitching false` was rejected.",
                  file=sys.stderr)
            return 1
        print("PASS (2/4): layer_surround_stitching round-trips true/false.")

        # ---- 3. surround_stitching_profile: accepts bowl/hybrid, rejects other ----
        for profile in ("bowl", "hybrid"):
            if not _param_set("surround_stitching_profile", profile):
                print(f"FAIL: `ros2 param set surround_stitching_profile {profile}` was "
                      f"rejected -- expected accepted.", file=sys.stderr)
                return 1
        if _param_set("surround_stitching_profile", "lidar"):
            print("FAIL: `ros2 param set surround_stitching_profile lidar` was ACCEPTED -- "
                  "expected rejected (only 'bowl'/'hybrid' are valid).", file=sys.stderr)
            return 1
        print("PASS (3/4): surround_stitching_profile accepts bowl/hybrid, rejects lidar.")

        # ---- 4. node stayed alive throughout, no /rendering/set_mode touched ----
        time.sleep(0.5)
        if viz_proc.poll() is not None:
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: visualization_node crashed during the param cycle:\n{tail}",
                  file=sys.stderr)
            return 1
        print("PASS (4/4): node stayed alive throughout -- no ROS message on any topic was "
              "needed for any of the above.")

        print("PASS: visualization_node local render-mode/Surround Stitching param test passed.")
        return 0
    finally:
        _kill(viz_proc)
        try:
            os.remove(log_path)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
