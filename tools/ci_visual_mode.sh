#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

LOG_DIR="/tmp/ci_visual_mode.$$"
mkdir -p "${LOG_DIR}"
echo "logs: ${LOG_DIR}/"

declare -a STAGE_NAMES=()
declare -a STAGE_RESULTS=()
declare -a STAGE_NOTES=()
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

banner 1/6 "POD header check"
POD_LOG="${LOG_DIR}/pod_header.log"
if bash "${REPO_ROOT}/overlume/scripts/check_pod_header.sh" \
        > "${POD_LOG}" 2>&1; then
    echo "PASS  POD header check"
    record_stage "POD header check" PASS
else
    echo "FAIL  POD header check (see ${POD_LOG})"
    cat "${POD_LOG}"
    record_stage "POD header check" FAIL
fi

banner 2/6 "library ctest suite"
LIB_DIR="${REPO_ROOT}/overlume"
LIB_BUILD_DIR="${LIB_DIR}/build"
LIB_CONFIGURE_LOG="${LOG_DIR}/lib_configure.log"
LIB_BUILD_LOG="${LOG_DIR}/lib_build.log"
LIB_CTEST_LOG="${LOG_DIR}/lib_ctest.log"
LIB_STAGE_OK=1

[[ -n "${CI_VISUAL_MODE_CLEAN:-}" ]] && rm -rf "${LIB_BUILD_DIR}"
mkdir -p "${LIB_BUILD_DIR}"
if ! cmake --toolchain "${LIB_DIR}/cmake/toolchain-clang-libcxx.cmake" \
        -S "${LIB_DIR}" -B "${LIB_BUILD_DIR}" > "${LIB_CONFIGURE_LOG}" 2>&1; then
    echo "FAIL  overlume configure (see ${LIB_CONFIGURE_LOG})"
    tail -40 "${LIB_CONFIGURE_LOG}"
    LIB_STAGE_OK=0
fi

if [[ "${LIB_STAGE_OK}" == "1" ]]; then
    if ! cmake --build "${LIB_BUILD_DIR}" -j"$(nproc)" > "${LIB_BUILD_LOG}" 2>&1; then
        echo "FAIL  overlume build (see ${LIB_BUILD_LOG})"
        tail -60 "${LIB_BUILD_LOG}"
        LIB_STAGE_OK=0
    fi
fi

if [[ "${LIB_STAGE_OK}" == "1" ]]; then
    if ctest --test-dir "${LIB_BUILD_DIR}" -V > "${LIB_CTEST_LOG}" 2>&1; then
        LIB_SUMMARY="$(grep -E '^[0-9]+% tests passed' "${LIB_CTEST_LOG}" || true)"
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

banner 3/6 "node gtests (colcon test)"
ROS_SETUP="/opt/ros/humble/setup.bash"
MAIN_INSTALL="${CI_VISUAL_MODE_ROS_APPS_INSTALL:-${REPO_ROOT}/ros/install/setup.bash}"
NODE_WS="${REPO_ROOT}/ros"
NODE_LOG="${LOG_DIR}/node_colcon.log"
NODE_STAGE_OK=1

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "FAIL  node gtests: ${ROS_SETUP} not found (no ROS install) -- colcon infra unavailable"
    record_stage "node gtests" FAIL
    NODE_STAGE_OK=0
elif [[ ! -f "${MAIN_INSTALL}" ]]; then
    echo "FAIL  node gtests: ${MAIN_INSTALL} not found (this checkout's ros/install," \
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
        colcon build --packages-select overlume_ros \
            --event-handlers desktop_notification- \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            && colcon test --packages-select overlume_ros \
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

banner 4/6 "WS bridge pytest suite"
WS_LOG="${LOG_DIR}/ws_bridge_pytest.log"
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

banner 5/6 "golden suite (GPU-skip)"
if [[ -f "${LIB_CTEST_LOG}" ]]; then
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
        SKIP_NOTE=""
        [[ "${GOLDEN_SKIPPED}" -gt 0 ]] && SKIP_NOTE=" (no GPU/EGL)"
        echo "PASS  golden suite: ${GOLDEN_OK} ok, ${GOLDEN_SKIPPED} skipped${SKIP_NOTE}"
        record_stage "golden suite" PASS "${GOLDEN_OK} ok / ${GOLDEN_SKIPPED} skipped${SKIP_NOTE}"
    fi
else
    echo "FAIL  golden suite: stage 2's ctest log is missing (library stage never ran)"
    record_stage "golden suite" FAIL
fi

banner 6/6 "examples"
EXAMPLES_BUILD_DIR="${LIB_BUILD_DIR}/examples"
EXAMPLES_TMP_DIR="${LOG_DIR}/examples_out"
EXAMPLES_LOG="${LOG_DIR}/examples.log"
mkdir -p "${EXAMPLES_TMP_DIR}"
EXAMPLES_STAGE_OK=1
EXAMPLES_RAN=0

if [[ ! -d "${EXAMPLES_BUILD_DIR}" ]]; then
    echo "FAIL  examples: ${EXAMPLES_BUILD_DIR} not found -- stage 2's build didn't produce it" \
        | tee -a "${EXAMPLES_LOG}"
    EXAMPLES_STAGE_OK=0
else
    for _example_bin in "${EXAMPLES_BUILD_DIR}"/*; do
        [[ -f "${_example_bin}" && -x "${_example_bin}" ]] || continue
        _example_name="$(basename "${_example_bin}")"
        _out_png="${EXAMPLES_TMP_DIR}/${_example_name}.png"
        EXAMPLES_RAN=$((EXAMPLES_RAN + 1))
        if ! env -u CESIUM_ION_TOKEN "${_example_bin}" "${_out_png}" >> "${EXAMPLES_LOG}" 2>&1; then
            echo "FAIL  examples: ${_example_name} exited non-zero (see ${EXAMPLES_LOG})"
            EXAMPLES_STAGE_OK=0
            continue
        fi
        if [[ ! -s "${_out_png}" ]]; then
            echo "FAIL  examples: ${_example_name} produced no non-empty output at ${_out_png}"
            EXAMPLES_STAGE_OK=0
        fi
    done
fi

EXAMPLES_EXPECTED=$(ls "${REPO_ROOT}"/examples/*.cpp 2>/dev/null | wc -l)
if [[ "${EXAMPLES_STAGE_OK}" == "1" && "${EXAMPLES_RAN}" -gt 0 \
        && "${EXAMPLES_RAN}" -eq "${EXAMPLES_EXPECTED}" ]]; then
    echo "PASS  examples  (${EXAMPLES_RAN} run, all exited 0 with non-empty output)"
    record_stage "examples" PASS "${EXAMPLES_RAN} run"
else
    echo "FAIL  examples: ran ${EXAMPLES_RAN}, expected ${EXAMPLES_EXPECTED}" \
         "(examples/*.cpp on disk) -- see ${EXAMPLES_LOG}"
    tail -40 "${EXAMPLES_LOG}" 2>/dev/null || true
    record_stage "examples" FAIL "${EXAMPLES_RAN} run / ${EXAMPLES_EXPECTED} expected"
fi

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
