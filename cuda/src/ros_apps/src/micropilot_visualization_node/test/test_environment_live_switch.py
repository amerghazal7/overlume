#!/usr/bin/env python3
"""Live environment_enabled/environment_source_uri param regression test
(this task -- vcam GUI Environment Tiles toggle, Epic 6 follow-up).

Same standalone-subprocess pattern test_mode_dispatch.py/test_bowl_node_params.py
already established for this package (no in-process rclcpp node-harness
convention exists here). Runs with DEFAULT params -- no
environment_chunks_dir/environment_source_uri override, no geo_datum_*
override -- so the geo-anchor never solves (no NavSatFix/TF traffic is fed
to this node) and renderer_->environmentSource stays null the whole run.
That is deliberate: it is the cheapest way to exercise the two NEW on_params()
branches without needing a GPU-rendered scene, a solved anchor, or network
access, and it is exactly the precondition design decision (b) says must be
guarded honestly:

  1. **environment_own_asset_uri is a real declared param, default ""**
     (design decision (c) -- the "clipped" preset's own asset id, never a
     fabricated value).
  2. **environment_enabled round-trips live** (true/false both ACCEPTED --
     same "config for every rendered element" shape as every other layer_*
     disable knob) AND logs an honest WARN when toggled with no
     environment source configured (the "view: pointcloud button does
     nothing" precedent this repo's own CLAUDE.md-adjacent history names:
     a control that silently does nothing must say so).
  3. **environment_source_uri is REJECTED while the geo-anchor has not
     solved** (res.successful=false, reason names the anchor) -- the same
     precondition on_activate()'s own environment-arming branch already
     enforces, now also enforced on the LIVE path. This is the one check
     that FAILS against the pre-this-task node (no case for this param name
     existed, so it fell through the on_params() catch-all as accepted).
  4. **node stays alive throughout.**

Honest scope: this file does NOT exercise the on_params() SUCCESS path for
environment_source_uri (that needs a solved geo-anchor, i.e. real
NavSatFix+TF traffic, or a GeoDatumOverride -- out of this lightweight
subprocess test's budget) or environment_enabled actually hiding a rendered
building (that is the library-level proof, test_environment.cpp's
SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown/
ChunkLoadedWhileHiddenDoesNotPopIntoView, and the streaming-backend twin in
test_environment_stream.cpp). What this file proves is the PARAM-SURFACE
contract: accept/reject shape and the honest-WARN discipline.

Run (ROS + this repo's ros_apps install sourced first):
    source /opt/ros/humble/setup.bash
    source cuda/install/ros_apps/setup.bash
    python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/test_environment_live_switch.py

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install isn't
present, same convention as test_mode_dispatch.py/test_bowl_node_params.py.

NOT CMake-registered -- same as test_mode_dispatch.py/test_bowl_node_params.py/
test_theme_ws.py/test_vcam_contract.py/test_ego_anchored_vcam.py/
test_extra_topic_parity.py/test_tf_adapter.py in this same directory.
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


def _param_set(name: str, value_literal: str) -> subprocess.CompletedProcess:
    return _run(f"ros2 param set {NODE_NAME} {name} {value_literal}")


def _param_set_ok(name: str, value_literal: str) -> bool:
    result = _param_set(name, value_literal)
    return result.returncode == 0 and "Set parameter successful" in result.stdout


def _param_get(name: str) -> str:
    result = _run(f"ros2 param get {NODE_NAME} {name}")
    return result.stdout.strip()


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

    log_fd, log_path = tempfile.mkstemp(prefix="viz_environment_live_switch_", suffix=".log")
    os.close(log_fd)

    # Deliberately NO environment_chunks_dir/environment_source_uri and NO
    # geo_datum_* override: the geo-anchor never solves this run (no
    # NavSatFix/TF is published), so renderer_->environmentSource stays
    # null and on_activate()'s own environment-arming branch never fires
    # either -- this is the cheapest way to reach "geo-anchor not solved"
    # for check 3 below without a live GPS/TF feed.
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

        # ---- 1. environment_own_asset_uri: declared, defaults to "" ----
        got = _param_get("environment_own_asset_uri")
        if "String value is:" not in got:
            print(f"FAIL: environment_own_asset_uri is not a declared string param -- "
                  f"`ros2 param get` returned {got!r} (expected a param declaration, even if "
                  f"the value itself is empty).", file=sys.stderr)
            return 1
        print(f"PASS (1/4): environment_own_asset_uri is declared ({got!r}).")

        # ---- 2. environment_enabled round-trips live + honest WARN when unarmed ----
        if not _param_set_ok("environment_enabled", "false"):
            print("FAIL: `ros2 param set environment_enabled false` was rejected -- "
                  "expected accepted (STANDING disable-knob shape).", file=sys.stderr)
            return 1
        if not _param_set_ok("environment_enabled", "true"):
            print("FAIL: `ros2 param set environment_enabled true` was rejected -- "
                  "expected accepted.", file=sys.stderr)
            return 1
        time.sleep(0.3)
        with open(log_path) as f:
            log_text = f.read()
        if "environment_enabled" not in log_text or "no environment source" not in log_text:
            print(f"FAIL: no honest WARN found for toggling environment_enabled with no "
                  f"environment source configured -- a control that silently does nothing "
                  f"must say so (this repo's own precedent: the pointcloud view-button "
                  f"incident). Log tail:\n{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (2/4): environment_enabled round-trips true/false live and WARNs "
              "honestly when no source is configured to show/hide.")

        # ---- 3. environment_source_uri REJECTED while geo-anchor unsolved ----
        result = _param_set("environment_source_uri", "ion://96188")
        if "Set parameter successful" in result.stdout:
            print("FAIL: `ros2 param set environment_source_uri ion://96188` was ACCEPTED "
                  "with the geo-anchor never solved -- expected rejected (same precondition "
                  "on_activate()'s own environment-arming branch enforces).", file=sys.stderr)
            return 1
        # `ros2 param set` prints a REJECTED result's reason to stderr, not
        # stdout (confirmed empirically -- only the success message goes to
        # stdout).
        if "anchor" not in result.stderr.lower():
            print(f"FAIL: environment_source_uri was rejected (good) but the reason doesn't "
                  f"name the geo-anchor precondition. stdout={result.stdout!r} "
                  f"stderr={result.stderr!r}", file=sys.stderr)
            return 1
        print("PASS (3/4): environment_source_uri live switch is REJECTED while the "
              "geo-anchor has not solved, with a reason naming the anchor.")

        # ---- 4. node stayed alive throughout ----
        if viz_proc.poll() is not None:
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: visualization_node crashed during the param cycle:\n{tail}",
                  file=sys.stderr)
            return 1
        print("PASS (4/4): node stayed alive throughout.")

        print("PASS: visualization_node environment live-switch param test passed.")
        return 0
    finally:
        _kill(viz_proc)


if __name__ == "__main__":
    sys.exit(main())
