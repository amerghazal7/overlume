#!/usr/bin/env python3
"""Asset-resolution regression test (VM-044 review round 2, blocking finding).

B05.4.2's own acceptance criterion is literally "Param test" and none
existed -- every prior verification of theme_assets_dir/hud_font_path/
ego_model_path/initial_theme resolution was manual, in commit prose only.
The failure mode VM-044 closes is a SILENT fallback (wrong theme dir /
clay-box ego with no error), so a regression here has to be visible to a
test, not just to someone reading a log by hand.

Same standalone-subprocess pattern test_bowl_node_params.py/test_theme_ws.py
already established (no in-process rclcpp node-harness convention exists in
this package), reusing that file's _popen/_kill/_lifecycle-style helpers.

NOTE: the node must be built Release. A plain `colcon build` (empty
CMAKE_BUILD_TYPE, the colcon default) SIGSEGVs inside on_configure() before
theme resolution completes -- a pre-existing, not-VM-044 gap (see
docs/plans/2026-08-18-visual-mode-epic3.md's Debug-build crash
note, widened by the same review round to cover this default-build case
too). tools/ci_visual_mode.sh builds -DCMAKE_BUILD_TYPE=Release; do the same
before running this test, or every case below fails with no useful signal.

Three cases:
  1. Launch with --params-file <share>/config/default_params.yaml (the
     installed one, resolved via `ros2 pkg prefix`). `ros2 lifecycle set
     configure` must succeed. `ros2 param get` for theme_assets_dir,
     hud_font_path and ego_model_path must each return a path that starts
     with the installed share dir -- EXCEPT ego_model_path, which may
     legitimately come back "" (scripts/provision_ego_model.sh not run on
     this install): assert exactly one of the two so an unprovisioned box
     still passes honestly instead of the test silently accepting either
     without checking. No "theme assets failed to load" WARN in the log,
     and (when the ego path resolved) no "set_ego_model: failed to load"
     clay-box fallback WARN either.
  2. Relaunch with -p initial_theme:=light_clay -- configure succeeds,
     `ros2 param get initial_theme` reads back light_clay, still no
     fallback WARN.
  3. Relaunch with -p initial_theme:=definitely_not_a_theme -- the
     configure transition must FAIL (on_configure()'s own validation against
     what this install actually shipped), the process must stay alive (a
     failed transition is not a crash), and the log must carry the
     "initial_theme '...' not found under theme_assets_dir" ERROR.

Run (ROS + this repo's ros install sourced first, node built Release):
    source /opt/ros/humble/setup.bash
    source ros/install/setup.bash
    python3 ros/src/overlume_ros/test/test_asset_paths.py

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install isn't
present, same convention as test_theme_ws.py/test_bowl_node_params.py.

NOT CMake-registered (`ament_add_gtest`/`ament_add_pytest_test` etc.) -- same
undocumented-elsewhere convention as test_theme_ws.py/test_bowl_node_params.py/
test_vcam_contract.py/test_ego_anchored_vcam.py/test_extra_topic_parity.py/
test_tf_adapter.py in this same directory (grepped: none of them appear in
this package's CMakeLists.txt as an ament_add_test/ament_add_pytest_test/
add_test entry).
"""

import os
import re
import subprocess
import sys
import tempfile
import time

REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
INSTALL_DIR = os.path.join(REPO_ROOT, "ros", "install")
NODE_NAME = "/overlume_node"
PKG = "overlume_ros"


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
    full = f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && {cmd}"
    return subprocess.run(["bash", "-c", full], capture_output=True, text=True, timeout=timeout)


def _lifecycle(transition: str, timeout: float = 15.0) -> bool:
    for _ in range(20):
        result = _run(f"ros2 lifecycle set {NODE_NAME} {transition}", timeout=timeout)
        if "Transitioning successful" in result.stdout:
            return True
        if "Transitioning failed" in result.stdout:
            return False
        time.sleep(1)
    return False


def _param_get(name: str) -> str:
    result = _run(f"ros2 param get {NODE_NAME} {name} --hide-type")
    return result.stdout.strip()


def _wait_running(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


def _pkg_share_dir() -> str:
    result = _run(f"ros2 pkg prefix {PKG}")
    prefix = result.stdout.strip()
    if not prefix:
        raise RuntimeError(f"`ros2 pkg prefix {PKG}` returned nothing "
                            f"(stderr: {result.stderr!r})")
    return os.path.join(prefix, "share", PKG)


def _launch(extra_args: str, log_path: str) -> subprocess.Popen:
    cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run {PKG} overlume_node --ros-args "
        f"--params-file {SHARE_DIR}/config/default_params.yaml "
        f"-p out_width:=160 -p out_height:=120 -p initial_mode:=3 {extra_args} "
        f"> {log_path} 2>&1")
    return _popen(cmd)


def _fresh_log() -> str:
    fd, path = tempfile.mkstemp(prefix="viz_asset_paths_", suffix=".log")
    os.close(fd)
    return path


def _read(path: str) -> str:
    with open(path) as f:
        return f.read()


SHARE_DIR = ""  # set in main() once we know INSTALL_DIR is real


def main() -> int:
    global SHARE_DIR
    if not os.path.isdir(INSTALL_DIR):
        print("SKIP: ros/install not built -- run colcon build first.")
        return 0

    SHARE_DIR = _pkg_share_dir()
    if not os.path.isdir(SHARE_DIR):
        print(f"SKIP: resolved share dir '{SHARE_DIR}' does not exist -- "
              "package not actually installed under this prefix.")
        return 0

    # ---- Case 1: default params, no theme override ----
    log1 = _fresh_log()
    proc = _launch("", log1)
    try:
        if not _wait_running(proc):
            print(f"FAIL: node exited early (case 1):\n{_read(log1)[-2000:]}", file=sys.stderr)
            return 1
        if not _lifecycle("configure"):
            print(f"FAIL: configure failed with default params:\n{_read(log1)[-2000:]}",
                  file=sys.stderr)
            return 1

        theme_dir = _param_get("theme_assets_dir")
        hud_font = _param_get("hud_font_path")
        ego_path = _param_get("ego_model_path")

        if not theme_dir.startswith(SHARE_DIR):
            print(f"FAIL: theme_assets_dir '{theme_dir}' does not start with "
                  f"installed share dir '{SHARE_DIR}'.", file=sys.stderr)
            return 1
        if not hud_font.startswith(SHARE_DIR):
            print(f"FAIL: hud_font_path '{hud_font}' does not start with "
                  f"installed share dir '{SHARE_DIR}'.", file=sys.stderr)
            return 1
        # ego_model_path is allowed to be exactly "" (unprovisioned box) --
        # assert exactly one of the two honest outcomes, not "whichever".
        ego_resolved = ego_path.startswith(SHARE_DIR)
        ego_empty = ego_path == ""
        if ego_resolved == ego_empty:  # both true or both false: neither is right
            print(f"FAIL: ego_model_path '{ego_path}' is neither a resolved "
                  f"installed path nor the honest empty-default.", file=sys.stderr)
            return 1

        log_text = _read(log1)
        if "theme assets failed to load" in log_text:
            print(f"FAIL: unexpected theme-fallback WARN in log:\n{log_text[-3000:]}",
                  file=sys.stderr)
            return 1
        # A resolved ego path that then silently clay-boxes is the exact
        # failure mode VM-044 exists to kill (gate minor 1): the WARN string
        # is real and reachable (verified against a bogus ego_model_path).
        if ego_resolved and "set_ego_model: failed to load" in log_text:
            print(f"FAIL: ego path resolved but the model fell back to the "
                  f"clay box:\n{log_text[-3000:]}", file=sys.stderr)
            return 1
        print(f"PASS (1/3): theme_assets_dir='{theme_dir}', hud_font_path='{hud_font}', "
              f"ego_model_path='{ego_path}' (resolved={ego_resolved}), no fallback WARN.")
    finally:
        _kill(proc)
        os.remove(log1)

    # ---- Case 2: initial_theme:=light_clay ----
    log2 = _fresh_log()
    proc = _launch("-p initial_theme:=light_clay", log2)
    try:
        if not _wait_running(proc):
            print(f"FAIL: node exited early (case 2):\n{_read(log2)[-2000:]}", file=sys.stderr)
            return 1
        if not _lifecycle("configure"):
            print(f"FAIL: configure failed with initial_theme:=light_clay:\n"
                  f"{_read(log2)[-2000:]}", file=sys.stderr)
            return 1
        theme = _param_get("initial_theme")
        if theme != "light_clay":
            print(f"FAIL: `ros2 param get initial_theme` returned '{theme}', expected "
                  "'light_clay'.", file=sys.stderr)
            return 1
        log_text = _read(log2)
        if "theme assets failed to load" in log_text:
            print(f"FAIL: unexpected theme-fallback WARN with light_clay:\n"
                  f"{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (2/3): initial_theme:=light_clay configured cleanly, param reads back "
              "light_clay, no fallback WARN.")
    finally:
        _kill(proc)
        os.remove(log2)

    # ---- Case 3: initial_theme:=definitely_not_a_theme ----
    log3 = _fresh_log()
    proc = _launch("-p initial_theme:=definitely_not_a_theme", log3)
    try:
        if not _wait_running(proc):
            print(f"FAIL: node exited early before configure attempt (case 3):\n"
                  f"{_read(log3)[-2000:]}", file=sys.stderr)
            return 1
        if _lifecycle("configure"):
            print("FAIL: configure SUCCEEDED with an unknown initial_theme -- expected the "
                  "transition to FAIL.", file=sys.stderr)
            return 1
        time.sleep(0.5)
        if proc.poll() is not None:
            print(f"FAIL: node process died (expected: alive, transition failed cleanly) "
                  f"after a bad initial_theme:\n{_read(log3)[-2000:]}", file=sys.stderr)
            return 1
        log_text = _read(log3)
        if not re.search(r"initial_theme '.*' not found under theme_assets_dir", log_text):
            print(f"FAIL: no 'initial_theme ... not found under theme_assets_dir' ERROR in "
                  f"the log:\n{log_text[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (3/3): bad initial_theme FAILS the configure transition, node stays "
              "alive, ERROR names the missing theme.")
    finally:
        _kill(proc)
        os.remove(log3)

    print("PASS: overlume_node asset-path resolution test passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
