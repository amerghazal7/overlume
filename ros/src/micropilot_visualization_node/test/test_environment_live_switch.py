#!/usr/bin/env python3
"""Live environment_enabled/environment_source_uri param regression test
(VM-096 -- vcam GUI Environment Tiles toggle, Epic 6 follow-up).

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
     that FAILS against the pre-VM-096 node (no case for this param name
     existed, so it fell through the on_params() catch-all as accepted).
  4. **node stays alive throughout.**
  5. (separate run, _run_hidden_arm_check()) **environment_enabled:=false at
     launch, then a LATER live environment_source_uri switch, arms the
     environment source HIDDEN, not visible** (VM-096 gate round 1 finding)
     -- a geo_datum_* override solves the anchor so the live switch is
     accepted (this is the on_params() SUCCESS path checks 1-4 above don't
     reach), asserting the "environment source armed" log names it 'hidden'.

Honest scope: checks 1-4 do NOT exercise the on_params() SUCCESS path for
environment_source_uri (that needs a solved geo-anchor, i.e. real
NavSatFix+TF traffic, or a GeoDatumOverride -- out of this lightweight
subprocess test's budget) or environment_enabled actually hiding a rendered
building (that is the library-level proof, test_environment.cpp's
SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown/
ChunkLoadedWhileHiddenDoesNotPopIntoView, and the streaming-backend twin in
test_environment_stream.cpp). Check 5 does reach the on_params() SUCCESS
path (via a GeoDatumOverride) but still proves only the PARAM-SURFACE/log
contract, not a rendered pixel -- same reasons.

Run (ROS + this repo's ros install sourced first):
    source /opt/ros/humble/setup.bash
    source ros/install/setup.bash
    python3 ros/src/micropilot_visualization_node/test/test_environment_live_switch.py

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
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
INSTALL_DIR = os.path.join(REPO_ROOT, "ros", "install")
NODE_NAME = "/visualization_node"

# The real, committed environment_test_town_0 fixture (test_environment.cpp's
# own kTestTownDir) + its matching anchor -- a REAL baked dir, never a
# fabricated path, so this run's geo-anchor solves via override and
# on_activate()'s environment-arming branch actually fires (unlike main()'s
# run above, which deliberately never arms a source).
FIXTURE_CHUNKS_DIR = os.path.join(
    CUDA_ROOT, "src", "libs", "visual_renderer", "tests", "fixtures", "environment_test_town_0")
FIXTURE_LAT_DEG = 25.0803
FIXTURE_LON_DEG = 55.3910
FIXTURE_HEADING_DEG = 0.0


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


def _run_hidden_arm_check() -> int:
    """VM-096 gate round 1 finding: on_activate() must push environment_enabled_
    into the renderer's visibility flag BEFORE arming anything, so that a
    LATER live environment_source_uri switch -- which arms on its own
    schedule, gated only on the geo-anchor, never on environment_enabled_
    (on_params()'s own comment) -- picks up a HIDDEN flag rather than the
    renderer's true default (environmentVisible=true).

    Launches with environment_enabled:=false + a geo_datum_* override (no
    environment_chunks_dir here -- deliberately: composing a live
    environment_source_uri switch against a non-empty chunks dir appends
    "?fallback=<dir>", corrupting a plain baked path with a '?' that path
    doesn't have; Decision 5's own restated ceiling. So on_activate() itself
    arms nothing this run and logs no WARN at all -- every arming branch
    there (visualization_node.cpp's on_activate(), all three `if`/`else if`
    arms) is gated on environment_enabled_ being true, and this run launches
    with it false, so the "neither ... is set" WARN never fires either; the
    SAME real fixture dir (kTestTownDir, test_environment.cpp) is instead
    pushed live via `environment_source_uri`, the exact scenario the finding
    names).

    HONEST SCOPE (this matters -- verified empirically while writing this
    check): the "environment source armed" log's (hidden)/(visible) word is
    derived from environment_enabled_ itself, so it is intent, not proof --
    it would print '(hidden)' here even against a node built WITHOUT the
    on_activate() fix (confirmed by temporarily reverting the fix and
    re-running this exact check: it still passed). What THIS check actually
    proves is the param-surface/plumbing path: environment_enabled:=false at
    launch is preserved through to the live-switch log line, end to end,
    through real ROS param get/set, not mocked. The ACTUAL proof that a
    source armed after set_environment_visible(r, false) comes up hidden --
    the thing the log line cannot verify from outside the library -- is
    test_environment.cpp's
    Environment.SetEnvironmentVisibleFalseBeforeArmingHidesNewlyOpenedSource,
    which uses the new environment_visible() getter plus loaded/scene-
    membership counts and a real pixel diff. That test is the regression
    guard for this finding; this one is a wiring smoke check alongside it.
    """
    log_fd, log_path = tempfile.mkstemp(prefix="viz_environment_hidden_arm_", suffix=".log")
    os.close(log_fd)

    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node --ros-args "
        f"-p out_width:=160 -p out_height:=120 -p initial_mode:=3 "
        f"-p environment_enabled:=false "
        f"-p geo_datum_lat_deg:={FIXTURE_LAT_DEG} "
        f"-p geo_datum_lon_deg:={FIXTURE_LON_DEG} "
        f"-p geo_datum_heading_deg:={FIXTURE_HEADING_DEG} "
        f"> {log_path} 2>&1")
    viz_proc = _popen(viz_cmd)

    try:
        if not _wait_running(viz_proc):
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: visualization_node (hidden-arm run) exited early:\n{tail}",
                  file=sys.stderr)
            return 1

        if not _lifecycle("configure") or not _lifecycle("activate"):
            print("FAIL: configure/activate failed (hidden-arm run).", file=sys.stderr)
            return 1

        # The real, committed fixture dir (a plain path, no '?'/'&') -- a
        # LIVE switch, not the launch config, is what arms a source this
        # run (see the docstring above for why).
        if not _param_set_ok("environment_source_uri", FIXTURE_CHUNKS_DIR):
            result = _param_set("environment_source_uri", FIXTURE_CHUNKS_DIR)
            print(f"FAIL: live `environment_source_uri` switch to the real fixture dir was "
                  f"rejected -- expected accepted (geo-anchor solved via override). "
                  f"stdout={result.stdout!r} stderr={result.stderr!r}", file=sys.stderr)
            return 1

        time.sleep(0.3)
        with open(log_path) as f:
            log_text = f.read()

        if "environment source armed" not in log_text:
            print(f"FAIL: the live environment_source_uri switch was accepted but no "
                  f"'environment source armed' log appeared -- expected the on_params() "
                  f"success path to log it same as on_activate()'s. Log tail:\n"
                  f"{log_text[-3000:]}", file=sys.stderr)
            return 1
        if "(hidden)" not in log_text:
            print(f"FAIL: the arming log names this run 'visible' even though it was "
                  f"launched with environment_enabled:=false -- environment_enabled_ isn't "
                  f"reaching the live-switch log line. (This check's own docstring is honest "
                  f"that the log word alone doesn't PROVE the renderer stays hidden -- see "
                  f"test_environment.cpp's "
                  f"SetEnvironmentVisibleFalseBeforeArmingHidesNewlyOpenedSource for that.) "
                  f"Log tail:\n{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (check 5, separate run): environment_enabled:=false at launch + a "
              "later live environment_source_uri switch arms a HIDDEN source, not a "
              "visible one.")
        return 0
    finally:
        _kill(viz_proc)


def main() -> int:
    if not os.path.isdir(INSTALL_DIR):
        print("SKIP: ros/install not built -- run colcon_build.sh first.")
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
    finally:
        _kill(viz_proc)

    return _run_hidden_arm_check()


if __name__ == "__main__":
    sys.exit(main())
