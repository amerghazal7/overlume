#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

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
  5. (separate run, _run_hidden_arm_check()) **launched disabled -> armed
     hidden -> switch shows** (FOLLOW-UP 2, maintainer decision 2026-09-18):
     environment_enabled:=false at launch, then a live environment_source_uri
     switch arms the environment source HIDDEN, not visible (VM-096 gate
     round 1 finding, still true) -- a geo_datum_* override solves the anchor
     so the live switch is accepted (this is the on_params() SUCCESS path
     checks 1-4 above don't reach). A LATER `environment_enabled:=true` then
     shows it (apply_environment_visibility()'s own log line). Same run also
     covers FOLLOW-UP 9 (per-render_mode gating): `render_mode:=1` (BOWL)
     hides the now-visible source again even though environment_enabled
     stayed true, and `render_mode:=3` (FREE_LOOK) restores it.

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
    python3 ros/src/overlume_ros/test/test_environment_live_switch.py

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
NODE_NAME = "/overlume_node"

# The real, committed environment_test_town_0 fixture (test_environment.cpp's
# own kTestTownDir) + its matching anchor -- a REAL baked dir, never a
# fabricated path, so this run's geo-anchor solves via override and
# on_activate()'s environment-arming branch actually fires (unlike main()'s
# run above, which deliberately never arms a source).
FIXTURE_CHUNKS_DIR = os.path.join(
    REPO_ROOT, "overlume", "tests", "fixtures", "environment_test_town_0")
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
    """FOLLOW-UP 2 + FOLLOW-UP 9 (maintainer decision, 2026-09-18): one run,
    four live param changes, each checked against apply_environment_visibility()'s
    own log line ("environment visibility -> visible|hidden ..." --
    overlume_node.cpp) or the arming log it feeds ("environment source
    armed: '...' (visible|hidden)").

    Launches with environment_enabled:=false + a geo_datum_* override (no
    environment_chunks_dir here -- deliberately: composing a live
    environment_source_uri switch against a non-empty chunks dir appends
    "?fallback=<dir>", corrupting a plain baked path with a '?' that path
    doesn't have; Decision 5's own restated ceiling. So on_activate() itself
    arms nothing this run -- with neither chunks dir nor source_uri
    configured at launch it logs the "neither ... is set" WARN regardless of
    environment_enabled -- and the SAME real fixture dir (kTestTownDir,
    test_environment.cpp) is instead pushed live via `environment_source_uri`
    below, the exact scenario FOLLOW-UP 2 names).

    Sequence, each step checked against the log before the next fires:
      (a) launched disabled -- live `environment_source_uri` switch arms the
          real fixture dir HIDDEN (VM-096 gate round 1 finding, still true:
          the switch doesn't gate on environment_enabled at all, only
          visibility does).
      (b) `environment_enabled:=true` -- the now-armed source SHOWS
          (apply_environment_visibility() logs "-> visible"; this is
          FOLLOW-UP 2's "switch shows" half).
      (c) `render_mode:=1` (BOWL) -- HIDES it again even though
          environment_enabled stayed true (FOLLOW-UP 9: BOWL/HYBRID always
          hide buildings, mode overrides the switch).
      (d) `render_mode:=3` (FREE_LOOK) -- SHOWS it again, no re-arm needed.

    HONEST SCOPE: this is a param-surface/log-contract smoke check, not a
    rendered-pixel proof (the log line cannot itself verify the renderer's
    scene membership) -- the pixel-level regression guard is
    test_environment.cpp's
    Environment.SetEnvironmentVisibleFalseBeforeArmingHidesNewlyOpenedSource,
    and the pure-function truth table (all six mode x enabled combinations)
    is test_scene_assembly.cpp's Environment*/EnvironmentHidden*/
    EnvironmentInFreeLook* gtests.
    """
    log_fd, log_path = tempfile.mkstemp(prefix="viz_environment_hidden_arm_", suffix=".log")
    os.close(log_fd)

    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"-p out_width:=160 -p out_height:=120 -p initial_mode:=3 "
        f"-p environment_enabled:=false "
        f"-p geo_datum_lat_deg:={FIXTURE_LAT_DEG} "
        f"-p geo_datum_lon_deg:={FIXTURE_LON_DEG} "
        f"-p geo_datum_heading_deg:={FIXTURE_HEADING_DEG} "
        f"> {log_path} 2>&1")
    viz_proc = _popen(viz_cmd)

    def _read_log() -> str:
        with open(log_path) as f:
            return f.read()

    try:
        if not _wait_running(viz_proc):
            print(f"FAIL: overlume_node (hidden-arm run) exited early:\n{_read_log()[-2000:]}",
                  file=sys.stderr)
            return 1

        if not _lifecycle("configure") or not _lifecycle("activate"):
            print("FAIL: configure/activate failed (hidden-arm run).", file=sys.stderr)
            return 1

        # (a) The real, committed fixture dir (a plain path, no '?'/'&') --
        # a LIVE switch, not the launch config, is what arms a source this
        # run (see the docstring above for why). FIX ROUND 1 finding: every
        # step below captures `before` right before its own _param_set_ok()
        # and asserts only against the log written AFTER it -- on_activate()
        # (which ran before step (a) even starts) already logs its own
        # "environment visibility -> hidden" line (want=false against the
        # renderer's default-true), so a step that greped the WHOLE
        # cumulative log for that bare substring could pass without its own
        # action doing anything.
        before = len(_read_log())
        if not _param_set_ok("environment_source_uri", FIXTURE_CHUNKS_DIR):
            result = _param_set("environment_source_uri", FIXTURE_CHUNKS_DIR)
            print(f"FAIL: live `environment_source_uri` switch to the real fixture dir was "
                  f"rejected -- expected accepted (geo-anchor solved via override). "
                  f"stdout={result.stdout!r} stderr={result.stderr!r}", file=sys.stderr)
            return 1
        time.sleep(0.3)
        log_text = _read_log()[before:]
        if "environment source armed" not in log_text:
            print(f"FAIL: the live environment_source_uri switch was accepted but no "
                  f"'environment source armed' log appeared -- expected the on_params() "
                  f"success path to log it same as on_activate()'s. New log:\n"
                  f"{log_text[-3000:]}", file=sys.stderr)
            return 1
        if "environment source armed: '" + FIXTURE_CHUNKS_DIR + "' (hidden)" not in log_text:
            print(f"FAIL (a): the arming log names this run 'visible' even though it was "
                  f"launched with environment_enabled:=false -- expected '(hidden)'. "
                  f"New log:\n{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (a): launched disabled + a live environment_source_uri switch arms a "
              "HIDDEN source, not a visible one.")

        # (b) FOLLOW-UP 2's "switch shows" half: enabling now shows the
        # already-armed source.
        before = len(_read_log())
        if not _param_set_ok("environment_enabled", "true"):
            print("FAIL: `ros2 param set environment_enabled true` was rejected.",
                  file=sys.stderr)
            return 1
        time.sleep(0.3)
        log_text = _read_log()[before:]
        if "environment visibility -> visible" not in log_text:
            print(f"FAIL (b): environment_enabled:=true did not show the already-armed "
                  f"source -- expected an 'environment visibility -> visible' log line "
                  f"(apply_environment_visibility()). New log:\n{log_text[-3000:]}",
                  file=sys.stderr)
            return 1
        print("PASS (b): environment_enabled:=true shows the already-armed source "
              "(FOLLOW-UP 2's 'switch shows').")

        # (c) FOLLOW-UP 9: BOWL hides buildings even with environment_enabled
        # still true. Slicing to only the log written after THIS param set is
        # what makes this check bite -- on_activate()'s own launch-time
        # "-> hidden" line (want=false, logged long before step (a)) would
        # otherwise satisfy a whole-log substring search with zero help from
        # this step's actual render_mode:=1 change.
        before = len(_read_log())
        if not _param_set_ok("render_mode", "1"):
            print("FAIL: `ros2 param set render_mode 1` was rejected.", file=sys.stderr)
            return 1
        time.sleep(0.3)
        log_text = _read_log()[before:]
        if "environment visibility -> hidden" not in log_text:
            print(f"FAIL (c): render_mode:=1 (BOWL) did not hide buildings even though "
                  f"environment_enabled stayed true -- expected an "
                  f"'environment visibility -> hidden' log line. New log:\n"
                  f"{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (c): render_mode:=1 (BOWL) hides buildings regardless of "
              "environment_enabled.")

        # (d) FOLLOW-UP 9: FREE_LOOK restores them, no re-arm needed. Same
        # after-this-action slicing as (b)/(c) -- the fragile
        # `.count(...) >= 2` idiom this replaced only worked by coincidence
        # (nothing earlier could produce a second "-> visible" line).
        before = len(_read_log())
        if not _param_set_ok("render_mode", "3"):
            print("FAIL: `ros2 param set render_mode 3` was rejected.", file=sys.stderr)
            return 1
        time.sleep(0.3)
        log_text = _read_log()[before:]
        if "environment visibility -> visible" not in log_text:
            print(f"FAIL (d): render_mode:=3 (FREE_LOOK) did not re-show buildings -- "
                  f"expected an 'environment visibility -> visible' log line. "
                  f"New log:\n{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (d): render_mode:=3 (FREE_LOOK) re-shows buildings, no re-arm needed.")

        print("PASS (check 5, separate run): launched disabled -> armed hidden -> switch "
              "shows -> BOWL hides -> FREE_LOOK shows.")
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
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"-p out_width:=160 -p out_height:=120 -p initial_mode:=3 "
        f"> {log_path} 2>&1")
    viz_proc = _popen(viz_cmd)

    try:
        if not _wait_running(viz_proc):
            with open(log_path) as f:
                tail = f.read()[-2000:]
            print(f"FAIL: overlume_node exited early:\n{tail}", file=sys.stderr)
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
            print(f"FAIL: overlume_node crashed during the param cycle:\n{tail}",
                  file=sys.stderr)
            return 1
        print("PASS (4/4): node stayed alive throughout.")

        print("PASS: overlume_node environment live-switch param test passed.")
    finally:
        _kill(viz_proc)

    return _run_hidden_arm_check()


if __name__ == "__main__":
    sys.exit(main())
