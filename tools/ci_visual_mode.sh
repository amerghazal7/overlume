#!/usr/bin/env bash
# ci_visual_mode.sh — VM-041 (Epic 5): the repo's one repo-local pre-merge
# gate for visual mode. This repo has NO hosted CI (no .github/workflows, no
# .gitlab-ci.yml) — this script IS "CI wiring" until a hosted platform
# exists. Run it from anywhere; it cd's off its own path.
#
# Stages (each labeled, each can fail the whole run):
#   1. POD header check      (visual_renderer/scripts/check_pod_header.sh)
#   2. Library ctest suite   (visual_renderer's full ctest run)
#   3. Node gtests           (colcon test, micropilot_visualization_node)
#   4. WS bridge pytest      (tools/test_vcam_ws_bridge.py, 53 tests)
#   5. Golden suite          (GPU-skip breakdown, honestly reported)
#
# GPU/EGL required -- a GPU-less box fails at stage 2/3 by design; see
# docs/visual_mode/README.md (the single home of the GPU/skip rationale).
#
# What this script deliberately does NOT do (validate_visual_mode.sh's own
# --live lesson): no bag is ever played, and nothing here touches a rig this
# script didn't itself start. Stage 4's two E2E tests DO start
# visualization_node and vcam_ws_bridge.py as their own
# child processes -- but as their own isolated processes on an isolated
# ROS_DOMAIN_ID, killed via killpg of the session they themselves started
# (never a process the script did not start).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

LOG_DIR="/tmp/ci_visual_mode.$$"
mkdir -p "${LOG_DIR}"
echo "logs: ${LOG_DIR}/"

# ── stage bookkeeping ───────────────────────────────────────────────────────
declare -a STAGE_NAMES=()
declare -a STAGE_RESULTS=()   # PASS | FAIL
declare -a STAGE_NOTES=()     # extra context for the summary line
OVERALL_FAIL=0

record_stage() {
    STAGE_NAMES+=("$1")
    STAGE_RESULTS+=("$2")
    STAGE_NOTES+=("${3:-}")
    if [[ "$2" == "FAIL" ]]; then
        OVERALL_FAIL=1
    fi
}

banner() {
    echo "=============================================================="
    echo "[$1] $2"
    echo "=============================================================="
}

# ── stage 1: POD header check ───────────────────────────────────────────────
banner 1/5 "POD header check"
POD_LOG="${LOG_DIR}/pod_header.log"
if bash "${REPO_ROOT}/cuda/src/libs/visual_renderer/scripts/check_pod_header.sh" \
        > "${POD_LOG}" 2>&1; then
    echo "PASS  POD header check"
    record_stage "POD header check" PASS
else
    echo "FAIL  POD header check (see ${POD_LOG})"
    cat "${POD_LOG}"
    record_stage "POD header check" FAIL
fi

# ── stage 2: library ctest suite (also feeds stage 5's golden breakdown) ───
banner 2/5 "library ctest suite"
LIB_DIR="${REPO_ROOT}/cuda/src/libs/visual_renderer"
LIB_BUILD_DIR="${LIB_DIR}/build"
LIB_CONFIGURE_LOG="${LOG_DIR}/lib_configure.log"
LIB_BUILD_LOG="${LOG_DIR}/lib_build.log"
LIB_CTEST_LOG="${LOG_DIR}/lib_ctest.log"
LIB_STAGE_OK=1

[[ -n "${CI_VISUAL_MODE_CLEAN:-}" ]] && rm -rf "${LIB_BUILD_DIR}"
mkdir -p "${LIB_BUILD_DIR}"
if ! cmake --toolchain "${LIB_DIR}/cmake/toolchain-clang-libcxx.cmake" \
        -S "${LIB_DIR}" -B "${LIB_BUILD_DIR}" > "${LIB_CONFIGURE_LOG}" 2>&1; then
    echo "FAIL  visual_renderer configure (see ${LIB_CONFIGURE_LOG})"
    tail -40 "${LIB_CONFIGURE_LOG}"
    LIB_STAGE_OK=0
fi

if [[ "${LIB_STAGE_OK}" == "1" ]]; then
    if ! cmake --build "${LIB_BUILD_DIR}" -j"$(nproc)" > "${LIB_BUILD_LOG}" 2>&1; then
        echo "FAIL  visual_renderer build (see ${LIB_BUILD_LOG})"
        tail -60 "${LIB_BUILD_LOG}"
        LIB_STAGE_OK=0
    fi
fi

if [[ "${LIB_STAGE_OK}" == "1" ]]; then
    # -V (not --output-on-failure): stage 5 below needs every test's own
    # gtest "[ OK ]"/"[ SKIPPED ]" line, not just the failing ones.
    if ctest --test-dir "${LIB_BUILD_DIR}" -V > "${LIB_CTEST_LOG}" 2>&1; then
        LIB_SUMMARY="$(grep -E '^[0-9]+% tests passed' "${LIB_CTEST_LOG}" || true)"
        # Suite-wide ok/skipped, not just goldens -- ctest's own "100% tests
        # passed" line folds every GTEST_SKIP() (~125 GPU-gated guard sites)
        # into "passed", which reads as full coverage on a degraded box.
        # Same inline-line anchor as stage 5, no Golden filter.
        SUITE_OK=$(grep -cE '\[ *OK *\].*\([0-9]+ ms\)$' "${LIB_CTEST_LOG}" || true)
        SUITE_SKIPPED=$(grep -cE '\[ *SKIPPED *\].*\([0-9]+ ms\)$' "${LIB_CTEST_LOG}" || true)
        LIB_SUMMARY="${LIB_SUMMARY} (${SUITE_OK} ok / ${SUITE_SKIPPED} skipped)"
        echo "PASS  library ctest suite  (${LIB_SUMMARY:-see log})"
        record_stage "library ctest suite" PASS "${LIB_SUMMARY}"
    else
        echo "FAIL  library ctest suite (see ${LIB_CTEST_LOG})"
        grep -E "tests passed|Failed" "${LIB_CTEST_LOG}" | tail -20 || true
        record_stage "library ctest suite" FAIL
    fi
else
    echo "FAIL  library ctest suite (configure/build failed above)"
    record_stage "library ctest suite" FAIL
fi

# ── stage 3: node gtests (colcon test, micropilot_visualization_node) ──────
# Never silently skipped: missing ROS/colcon infra is a loud FAIL here, not
# a skip — this is the pre-merge gate, not an optional convenience check.
banner 3/5 "node gtests (colcon test)"
ROS_SETUP="/opt/ros/humble/setup.bash"
# Post-cutover (VM-095): the node has no sibling ROS package dependency --
# SetVirtualCam.srv is generated in-package. The install space sourced here
# is the node's OWN prior install (needed for the srv typesupport at test
# time). CI_VISUAL_MODE_ROS_APPS_INSTALL still overrides for a worktree
# borrowing another checkout's install, read-only.
MAIN_INSTALL="${CI_VISUAL_MODE_ROS_APPS_INSTALL:-${REPO_ROOT}/cuda/install/ros_apps/setup.bash}"
NODE_WS="${REPO_ROOT}/cuda/src/ros_apps"
NODE_LOG="${LOG_DIR}/node_colcon.log"
NODE_STAGE_OK=1

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "FAIL  node gtests: ${ROS_SETUP} not found (no ROS install) -- colcon infra unavailable"
    record_stage "node gtests" FAIL
    NODE_STAGE_OK=0
elif [[ ! -f "${MAIN_INSTALL}" ]]; then
    echo "FAIL  node gtests: ${MAIN_INSTALL} not found (this checkout's install/ros_apps," \
         "needed for the node's own generated interfaces) -- build it first"
    record_stage "node gtests" FAIL
    NODE_STAGE_OK=0
elif ! command -v colcon > /dev/null 2>&1; then
    echo "FAIL  node gtests: colcon not on PATH -- colcon infra unavailable"
    record_stage "node gtests" FAIL
    NODE_STAGE_OK=0
fi

if [[ "${NODE_STAGE_OK}" == "1" ]]; then
    (
        set +u
        source "${ROS_SETUP}"
        source "${MAIN_INSTALL}"
        set -u
        cd "${NODE_WS}"
        # desktop_notification-: this box has no working dbus notification
        # daemon (confirmed: notify2 throws a GDBus timeout) -- harmless but
        # noisy in a log meant to be read for pass/fail, not desktop popups.
        colcon build --packages-select micropilot_visualization_node \
            --event-handlers desktop_notification- \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            && colcon test --packages-select micropilot_visualization_node \
                --event-handlers desktop_notification- \
            && colcon test-result --all --verbose
    ) > "${NODE_LOG}" 2>&1 && NODE_RC=0 || NODE_RC=$?
    if [[ "${NODE_RC}" -eq 0 ]]; then
        NODE_SUMMARY="$(grep -E 'tests?,.*errors?,.*failures?' "${NODE_LOG}" | tail -1 || true)"
        echo "PASS  node gtests  (${NODE_SUMMARY:-see log})"
        record_stage "node gtests" PASS "${NODE_SUMMARY}"
    else
        echo "FAIL  node gtests (colcon exit ${NODE_RC}, see ${NODE_LOG})"
        tail -60 "${NODE_LOG}"
        record_stage "node gtests" FAIL
    fi
fi

# ── stage 4: WS bridge pytest suite ─────────────────────────────────────────
banner 4/5 "WS bridge pytest suite"
WS_LOG="${LOG_DIR}/ws_bridge_pytest.log"
# -p no:anyio: this box's installed anyio pytest plugin is incompatible with
# the system pytest (ModuleNotFoundError: _pytest.scope) and aborts
# collection entirely before a single test runs -- confirmed the hard way.
# The WS bridge tests need no anyio fixture, so disabling that one autoload
# plugin is the whole fix.
#
# ROS_DOMAIN_ID: two of these tests (test_bridge_e2e_*) start real
# visualization_node / vcam_ws_bridge.py processes and
# drive them over ROS 2 by node name (ros2 lifecycle set, ros2 param
# get/set). _ros_env() in test_vcam_ws_bridge.py copies this process's
# environment into every one of those child processes, so pinning the
# domain here is what keeps this stage off whatever ROS_DOMAIN_ID an
# operator's shell already exports for a live rig -- CI_VISUAL_MODE_DOMAIN_ID
# overrides it, but the default must never be a domain a real rig uses
# (this repo's own tools default to 93/94, see validate_visual_mode.sh and
# flicker_measure.sh).
if ROS_DOMAIN_ID="${CI_VISUAL_MODE_DOMAIN_ID:-77}" \
        python3 -m pytest "${REPO_ROOT}/tools/test_vcam_ws_bridge.py" -q -p no:anyio \
        > "${WS_LOG}" 2>&1; then
    WS_SUMMARY="$(tail -1 "${WS_LOG}")"
    echo "PASS  WS bridge pytest suite  (${WS_SUMMARY})"
    record_stage "WS bridge pytest suite" PASS "${WS_SUMMARY}"
else
    echo "FAIL  WS bridge pytest suite (see ${WS_LOG})"
    tail -40 "${WS_LOG}"
    record_stage "WS bridge pytest suite" FAIL
fi

# ── stage 5: golden suite, GPU-skip reported honestly ───────────────────────
banner 5/5 "golden suite (GPU-skip)"
if [[ -f "${LIB_CTEST_LOG}" ]]; then
    # `.*\([0-9]+ ms\)$` anchors each grep to gtest's inline per-test line
    # only -- ctest -V also reprints every SKIPPED/FAILED name in its
    # end-of-run summary list (no "(N ms)" suffix there), so without this
    # anchor every skip/fail is counted twice. Golden-NAMED subset only;
    # see docs/visual_mode/README.md for what sits outside it.
    GOLDEN_OK=$(grep -cE '\[ *OK *\].*Golden.*\([0-9]+ ms\)$' "${LIB_CTEST_LOG}" || true)
    GOLDEN_SKIPPED=$(grep -cE '\[ *SKIPPED *\].*Golden.*\([0-9]+ ms\)$' "${LIB_CTEST_LOG}" || true)
    GOLDEN_FAILED=$(grep -cE '\[ *FAILED *\].*Golden.*\([0-9]+ ms\)$' "${LIB_CTEST_LOG}" || true)
    if [[ "${GOLDEN_FAILED}" -gt 0 ]]; then
        echo "FAIL  golden suite: ${GOLDEN_OK} ok, ${GOLDEN_SKIPPED} skipped (no GPU/EGL)," \
             "${GOLDEN_FAILED} FAILED"
        record_stage "golden suite" FAIL "${GOLDEN_OK} ok / ${GOLDEN_SKIPPED} skipped / ${GOLDEN_FAILED} failed"
    elif [[ "${GOLDEN_OK}" -eq 0 && "${GOLDEN_SKIPPED}" -eq 0 ]]; then
        echo "FAIL  golden suite: no golden tests found in ${LIB_CTEST_LOG} -- stage 2 didn't run them"
        record_stage "golden suite" FAIL
    else
        # PASS whether every golden ran (GPU present) or every golden skipped
        # (no GPU/EGL) -- skipped is reported as its own count, never as "ok".
        SKIP_NOTE=""
        [[ "${GOLDEN_SKIPPED}" -gt 0 ]] && SKIP_NOTE=" (no GPU/EGL)"
        echo "PASS  golden suite: ${GOLDEN_OK} ok, ${GOLDEN_SKIPPED} skipped${SKIP_NOTE}"
        record_stage "golden suite" PASS "${GOLDEN_OK} ok / ${GOLDEN_SKIPPED} skipped${SKIP_NOTE}"
    fi
else
    echo "FAIL  golden suite: stage 2's ctest log is missing (library stage never ran)"
    record_stage "golden suite" FAIL
fi

# ── summary ──────────────────────────────────────────────────────────────────
banner SUMMARY "ci_visual_mode.sh"
for i in "${!STAGE_NAMES[@]}"; do
    printf '%-4s %-28s %s\n' "${STAGE_RESULTS[$i]}" "${STAGE_NAMES[$i]}" "${STAGE_NOTES[$i]}"
done
echo "--------------------------------------------------------------"
if [[ "${OVERALL_FAIL}" == "0" ]]; then
    echo "OVERALL: PASS"
else
    echo "OVERALL: FAIL"
fi
echo "logs: ${LOG_DIR}/"

[[ "${OVERALL_FAIL}" == "0" ]]
