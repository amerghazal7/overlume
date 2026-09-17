#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# flicker_measure.sh — VM-077 carpet-flicker rig (2026-09-10).
#
# Stands up ONE continuous overlume_node + tf_flatten_fixture.py +
# ros2 bag play session (same invocation pattern validate_visual_mode.sh
# uses) and runs flicker_capture.py against it, which toggles
# layer_trajectory_carpet live to sample both conditions from the SAME
# bag window (see flicker_capture.py's own docstring for why that matters).
#
# Usage: tools/flicker_measure.sh [--out-dir PATH] [--bag PATH] [--qos PATH]
#
# Committed so the next measurement pass is reproducible from git, not just
# from /tmp (2026-09-10 code review finding).
set -euo pipefail
set -m  # own process group per background job -- see validate_visual_mode.sh's
        # own comment on this; same teardown hazard applies here.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${HOME}/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11"
QOS="${HOME}/TPSProjector-fixtures/qos_full.yaml"
OUT_DIR="/tmp/overlume_flicker_measure_after"
ROS_DOMAIN_ID_RIG="94"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-dir) OUT_DIR="$2"; shift 2 ;;
        --bag) BAG="$2"; shift 2 ;;
        --qos) QOS="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

LOG_DIR="${OUT_DIR}/logs"
mkdir -p "${LOG_DIR}"

if [[ ! -e "${BAG}" ]]; then echo "bag not found: ${BAG}" >&2; exit 1; fi
if [[ ! -e "${QOS}" ]]; then echo "qos override file not found: ${QOS}" >&2; exit 1; fi

set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/ros/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_RIG}"

declare -a CHILD_PIDS=()
declare -a CHILD_PGIDS=()
RIG_DOWN=0
track_child() {
    local pid="$1"
    CHILD_PIDS+=("${pid}")
    CHILD_PGIDS+=("$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')")
}
teardown() {
    [[ "${RIG_DOWN}" == "1" ]] && return
    RIG_DOWN=1
    local self_pid=$$ pid pgid
    local -a targets=()
    for pgid in "${CHILD_PGIDS[@]:-}"; do
        [[ -z "${pgid}" ]] && continue
        for pid in $(pgrep -g "${pgid}" 2>/dev/null || true); do
            [[ "${pid}" == "${self_pid}" ]] && continue
            targets+=("${pid}")
        done
    done
    targets+=("${CHILD_PIDS[@]:-}")
    if [[ "${#targets[@]}" -gt 0 ]]; then
        kill "${targets[@]}" 2>/dev/null || true
        sleep 1
        kill -9 "${targets[@]}" 2>/dev/null || true
    fi
    echo "rig down"
}
trap teardown INT TERM EXIT

echo "[launch] overlume_node (ROS_DOMAIN_ID=${ROS_DOMAIN_ID_RIG}, log: ${LOG_DIR}/overlume_node.log)"
ros2 run overlume_ros overlume_node --ros-args \
    --params-file "$(ros2 pkg prefix overlume_ros)/share/overlume_ros/config/default_params.yaml" \
    -p initial_mode:=3 -p use_sim_time:=true \
    > "${LOG_DIR}/overlume_node.log" 2>&1 &
track_child "$!"

lifecycle_set_retry() {
    local transition="$1" tries=0 out
    while true; do
        out="$(ros2 lifecycle set /overlume_node "${transition}" 2>&1)" || true
        if grep -q "Transitioning successful" <<<"${out}"; then return 0; fi
        if grep -q "Transitioning failed" <<<"${out}"; then
            echo "${transition} failed: see ${LOG_DIR}/overlume_node.log" >&2
            return 1
        fi
        tries=$((tries + 1))
        [[ "${tries}" -ge 15 ]] && { echo "${transition} failed after ${tries} tries" >&2; return 1; }
        sleep 1
    done
}
echo "[lifecycle] configure + activate ..."
lifecycle_set_retry configure
lifecycle_set_retry activate

echo "[launch] tf_flatten_fixture.py (log: ${LOG_DIR}/tf_flatten.log)"
python3 "${REPO_ROOT}/tools/tf_flatten_fixture.py" > "${LOG_DIR}/tf_flatten.log" 2>&1 &
track_child "$!"

echo "[launch] ros2 bag play --loop (log: ${LOG_DIR}/bag_play.log)"
ros2 bag play "${BAG}" --loop --clock \
    --qos-profile-overrides-path "${QOS}" --remap /tf:=/tf_raw \
    < /dev/null > "${LOG_DIR}/bag_play.log" 2>&1 &
track_child "$!"

echo "[settle] 3s for topics to stabilize ..."
sleep 3

echo "[capture] flicker_capture.py -> ${OUT_DIR}"
python3 "${REPO_ROOT}/tools/flicker_capture.py" --out-dir "${OUT_DIR}" \
    2>&1 | tee "${LOG_DIR}/capture.log"

echo "done: ${OUT_DIR}"
