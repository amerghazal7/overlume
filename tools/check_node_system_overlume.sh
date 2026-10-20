#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# Proves the ROS node builds against an INSTALLED overlume, the way a machine with only the apt
# package has it. Stages this checkout's shared library as a package, builds overlume_ros against
# that package with -DOVERLUME_ROS_LINK=system, asserts the result links the staged
# liboverlume.so.1 (no static Filament, no checkout headers), and runs the node tests that touch
# the library. It also configures the node with overlume hidden and expects the node's own "not
# found" message, the only guidance a machine with neither the package nor a library build gets.
# The work directory (OVERLUME_NODE_SYSTEM_DIR, default ros/system-overlume) persists between
# runs, so later builds are incremental. Run by tools/ci_visual_mode.sh after the node's own build.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
W="${OVERLUME_NODE_SYSTEM_DIR:-${REPO_ROOT}/ros/system-overlume}"
ROS_SETUP="/opt/ros/humble/setup.bash"
TESTS=(test_profile test_hud_overlay test_callouts test_point_cloud_adapter test_camera_ingest
       test_lidar_colorize test_exported_symbols)

fail() {
    echo "FAIL  node against an installed overlume: $1"
    exit 1
}

[[ -e "${REPO_ROOT}/overlume/build/liboverlume.so.1" ]] \
    || fail "overlume/build/liboverlume.so.1 is missing (build the library first)"
[[ -f "${ROS_SETUP}" ]] || fail "${ROS_SETUP} is missing (no ROS 2 Humble install)"

mkdir -p "${W}"
W="$(cd "${W}" && pwd)"
# colcon run from ros/ (colcon_build.sh, the gate) would otherwise find the CMake sub-projects
# the probe's configure generates under nofind/. Its build, install and log dirs mark themselves.
touch "${W}/COLCON_IGNORE"
LOG="${W}/colcon.log"
LIB="${W}/build/overlume_ros/liboverlume_node_lib.so"

cmake --install "${REPO_ROOT}/overlume/build" --prefix "${W}/prefix" --component overlume > /dev/null \
    || fail "cmake --install of overlume/build into ${W}/prefix failed"

# Only the ROS underlay is sourced, not the checkout's ros/install: the build needs nothing from it,
# and a package from there could shadow what is built here.
set +u
source "${ROS_SETUP}"
set -u

# A configure with overlume hidden must fail with the node's own "not found" message. Hiding works
# whether or not this machine has the package, and the FATAL_ERROR stops before the node downloads
# anything. With a REQUIRED find_package CMake would reject the hiding instead ("A REQUIRED package
# cannot be disabled"), so this fails if the message is reverted.
if NOFIND="$(cmake -S "${REPO_ROOT}/ros/src/overlume_ros" -B "${W}/nofind" \
        -DOVERLUME_ROS_LINK=system -DCMAKE_DISABLE_FIND_PACKAGE_overlume=ON 2>&1)"; then
    fail "configure without overlume succeeded"
fi
if ! grep -qF 'overlume 1.1 or later was not found' <<< "${NOFIND}"; then
    tail -20 <<< "${NOFIND}"
    fail "configure without overlume does not name the remedies"
fi

# Runs a colcon step with its output in the log; on failure prints the log's tail and fails. The
# tokens are unset because colcon snapshots its environment into the build tree and nothing here
# needs one.
colcon_step() {
    local what="$1"
    shift
    if ! env -u MAPBOX_TOKEN -u CESIUM_ION_TOKEN colcon --log-base "${W}/log" "$@" \
            >> "${LOG}" 2>&1; then
        tail -40 "${LOG}"
        fail "${what} failed (full output in ${LOG})"
    fi
}

: > "${LOG}"
colcon_step "colcon build" build --packages-select overlume_ros --base-paths "${REPO_ROOT}/ros/src" \
    --build-base "${W}/build" --install-base "${W}/install" --event-handlers desktop_notification- \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DOVERLUME_ROS_LINK=system "-Doverlume_DIR=${W}/prefix/lib/cmake/overlume"

[[ -f "${LIB}" ]] || fail "${LIB} was not built"
DYNAMIC="$(readelf -d "${LIB}" 2>/dev/null)"
grep -qF 'Shared library: [liboverlume.so.1]' <<< "${DYNAMIC}" \
    || fail "liboverlume_node_lib.so does not list NEEDED liboverlume.so.1"
RESOLVED="$(ldd "${LIB}" | awk '$1 == "liboverlume.so.1" {print $3}')"
[[ "${RESOLVED}" == "${W}/prefix/"* ]] \
    || fail "ldd resolves liboverlume.so.1 to '${RESOLVED:-nothing}', not into ${W}/prefix/"
FILAMENT_SYMBOLS="$(nm -C "${LIB}" 2>/dev/null | grep -cF 'filament::' || true)"
[[ "${FILAMENT_SYMBOLS}" -eq 0 ]] \
    || fail "liboverlume_node_lib.so holds ${FILAMENT_SYMBOLS} filament:: symbols (a static Filament link)"
if grep -qF "${REPO_ROOT}/overlume/include" "${W}/build/overlume_ros/compile_commands.json"; then
    fail "compile_commands.json uses the checkout's headers (${REPO_ROOT}/overlume/include)"
fi

# Stale results from an earlier, different selection would break the exactly-these-tests check.
rm -rf "${W}/build/overlume_ros/test_results"
colcon_step "colcon test" test --packages-select overlume_ros --base-paths "${REPO_ROOT}/ros/src" \
    --build-base "${W}/build" --install-base "${W}/install" --event-handlers desktop_notification- \
    --ctest-args -R "^($(IFS='|'; echo "${TESTS[*]}"))\$"
# --all lists each result file, so the test set can be checked, not only the totals.
RESULT="$(colcon --log-base "${W}/log" test-result --test-result-base "${W}/build/overlume_ros" \
    --all --verbose 2>&1)" || true
printf '%s\n' "${RESULT}" >> "${LOG}"
RAN="$(sed -nE 's|.*/test_results/overlume_ros/([^/]+)\.xml: .*|\1|p' <<< "${RESULT}" \
    | sed 's|\.gtest$||' | sort | tr '\n' ' ')"
WANT="$(printf '%s\n' "${TESTS[@]}" | sort | tr '\n' ' ')"
if [[ "${RAN}" != "${WANT}" ]]; then
    tail -40 "${LOG}"
    fail "colcon test-result covers [${RAN% }], expected [${WANT% }]"
fi
if ! grep -qE '^Summary: [0-9]+ tests?, 0 errors, 0 failures' <<< "${RESULT}"; then
    tail -40 "${LOG}"
    fail "colcon test-result reports errors or failures (see ${LOG})"
fi

echo "PASS  node against an installed overlume (${#TESTS[@]} tests; liboverlume.so.1 from the staged prefix)"
