#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# validate_visual_mode.sh — one command to stand up the full visual-mode
# validation rig (overlume_node + fixture bag + tf flattener + vcam
# bridge/GUI) against the recorded fixture bag, and report PASS/FAIL.
#
# Usage: tools/validate_visual_mode.sh [--bag PATH] [--qos PATH] [--no-gui]
#                                       [--build] [--profile NAME] [--live]
#
# --live: validate against the LIVE autonomy stack instead of the fixture
#         bag -- skips bag playback and the tf flattener (a live stack
#         publishes /tf itself), runs the node on wall time (override with
#         LIVE_SIM_TIME=true when the live source, e.g. CARLA, publishes
#         /clock), and relaxes the ego z==0.0 health check (that asserts
#         the flattener's output, a bag-rig invariant). A FAIL in live mode
#         can also mean "the stack just isn't publishing yet" -- the rig
#         stays up for inspection either way.
#
# ==========================================================================
set -euo pipefail
set -m  # each backgrounded job gets its OWN process group (job leader = its
        # own pid), even when this script itself is not a process-group
        # leader (piped from a wrapper, `bash -c`, CI, an agent harness).
        # Without this, all of this script's background jobs inherit
        # whatever pgid the invoking shell happened to have, and teardown()
        # below -- which kills by recorded child pgid -- ends up killing the
        # wrapper and its unrelated siblings instead of just the rig.

# ---------------------------------------------------------------------- args
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FIXTURES="${OVERLUME_FIXTURES:-${HOME}/overlume-fixtures}"
BAG="${FIXTURES}/stack_v3_full_sensors_2026-09-11"
QOS="${FIXTURES}/qos_full.yaml"
NO_GUI=0
DO_BUILD=0
PROFILE=""
PROFILE_DIR=""
EXTRA_PARAMS=()
LIVE=0
BAG_SET=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag) BAG="$2"; BAG_SET=1; shift 2 ;;
        --qos) QOS="$2"; shift 2 ;;
        --no-gui) NO_GUI=1; shift ;;
        --build) DO_BUILD=1; shift ;;
        --profile) PROFILE="$2"; shift 2 ;;
        --profile-dir) PROFILE_DIR="$2"; shift 2 ;;
        --param) EXTRA_PARAMS+=("-p" "$2"); shift 2 ;;
        --live) LIVE=1; shift ;;
        -h|--help)
            grep '^# ' "${BASH_SOURCE[0]}" | head -6 | sed 's/^# //'
            exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

if [[ "${LIVE}" == "1" && "${BAG_SET}" == "1" ]]; then
    echo "--live and --bag are mutually exclusive (live mode plays no bag)" >&2
    exit 1
fi

LOG_DIR=/tmp/overlume_validate
mkdir -p "${LOG_DIR}"


RIG_PATTERNS=(
    "ros2 run overlume_ros"
    "lib/overlume_ros/overlume_node"
    "ros2 bag play"
    "tools/tf_flatten_fixture\.py"
    "tools/vcam_ws_bridge\.py"
    "tools/vcam_gui\.py"
)

rig_candidate_pids() {
    local pattern
    for pattern in "${RIG_PATTERNS[@]}"; do
        pgrep -f "${pattern}" 2>/dev/null || true
    done | sort -un
}

kill_prior_rig() {
    local self_pid=$$
    local self_pgid
    self_pgid="$(ps -o pgid= -p "${self_pid}" 2>/dev/null | tr -d ' ')"
    local pid pgid killed=0
    for pid in $(rig_candidate_pids); do
        [[ "${pid}" == "${self_pid}" ]] && continue
        pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
        [[ -n "${pgid}" && "${pgid}" == "${self_pgid}" ]] && continue
        kill "${pid}" 2>/dev/null && killed=1 || true
    done
    if [[ "${killed}" == "1" ]]; then
        sleep 1
        # second pass, force, for anything that ignored SIGTERM
        for pid in $(rig_candidate_pids); do
            [[ "${pid}" == "${self_pid}" ]] && continue
            pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
            [[ -n "${pgid}" && "${pgid}" == "${self_pgid}" ]] && continue
            kill -9 "${pid}" 2>/dev/null || true
        done
        echo "[teardown-first] killed leftover rig process(es) from a prior run"
    fi
}
kill_prior_rig

if [[ "${LIVE}" == "1" ]]; then
    for _pass in 1 2; do
        while read -r _pid; do
            [[ -z "${_pid}" || "${_pid}" == "$$" ]] && continue
            kill -9 "${_pid}" 2>/dev/null || true
        done < <(pgrep -f "ros2 bag play" 2>/dev/null || true)
        sleep 0.5
    done
    if pgrep -f "ros2 bag play" >/dev/null 2>&1; then
        echo "[live] FATAL: a 'ros2 bag play' process is still running and could" >&2
        echo "       not be killed -- live mode will not fight a bag. Offender:" >&2
        pgrep -af "ros2 bag play" >&2
        exit 1
    fi
    
    if [[ "${LIVE_SIM_TIME:-false}" != "true" ]]; then
        set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u
        _clock_pubs="$(timeout -k 2 5 ros2 topic info /clock 2>/dev/null | sed -n 's/^Publisher count: //p' || true)"
        if [[ -n "${_clock_pubs}" && "${_clock_pubs}" != "0" ]]; then
            echo "[live] FATAL: /clock has ${_clock_pubs} publisher(s) on this ROS domain." >&2
            echo "       Something is playing a bag or publishing sim time. Stop it, or" >&2
            echo "       run with LIVE_SIM_TIME=true if the live source owns /clock." >&2
            exit 1
        fi
    fi
    echo "[live] verified: no bag player, no unexpected /clock publisher"
fi

# ---------------------------------------------------------------- prereqs
if [[ ! -d "${REPO_ROOT}/ros/install" ]]; then
    if [[ "${DO_BUILD}" == "1" ]]; then
        :  # built below
    else
        echo "ros/install not found. Run ros/colcon_build.sh" \
             "or re-run with --build." >&2
        exit 1
    fi
fi

if [[ "${DO_BUILD}" == "1" ]]; then
    echo "[build] running colcon_build.sh ..."
    ( cd "${REPO_ROOT}/ros" && ./colcon_build.sh )
fi

if [[ "${LIVE}" != "1" ]]; then
    if [[ ! -e "${BAG}" ]]; then
        echo "bag not found: ${BAG}" >&2
        exit 1
    fi
    if [[ ! -e "${QOS}" ]]; then
        echo "qos override file not found: ${QOS}" >&2
        exit 1
    fi
fi

# ------------------------------------------------------------------- ROS env
set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/ros/install/setup.bash"
set -u

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

echo "[launch] overlume_node (log: ${LOG_DIR}/overlume_node.log)"
PROFILE_ARGS=()
if [[ -n "${PROFILE}" ]]; then
    PROFILE_ARGS=(-p "profile:=${PROFILE}")
fi

if [[ -n "${PROFILE_DIR}" ]]; then
    PROFILE_ARGS+=(-p "profile_dir:=${PROFILE_DIR}")
fi

if [[ ${#EXTRA_PARAMS[@]} -gt 0 ]]; then
    PROFILE_ARGS+=("${EXTRA_PARAMS[@]}")
fi

USE_SIM_TIME=true
if [[ "${LIVE}" == "1" ]]; then
    USE_SIM_TIME="${LIVE_SIM_TIME:-false}"
fi

if [[ "${LIVE}" == "1" ]]; then
    env -u CYCLONEDDS_URI python3 "${REPO_ROOT}/tools/tf_static_relay.py" \
        > "${LOG_DIR}/tf_static_relay.log" 2>&1 &
    track_child "$!"
fi

ros2 run overlume_ros overlume_node --ros-args \
    --params-file "$(ros2 pkg prefix overlume_ros)/share/overlume_ros/config/default_params.yaml" \
    -p initial_mode:=3 -p use_sim_time:="${USE_SIM_TIME}" -p bowl_enabled:=true -p hybrid_enabled:=true "${PROFILE_ARGS[@]}" \
    > "${LOG_DIR}/overlume_node.log" 2>&1 &
track_child "$!"

echo "[lifecycle] configure + activate (retrying while the node registers)..."

lifecycle_set_retry() {
    local transition="$1" tries=0 out upper
    upper="$(printf '%s' "${transition}" | tr '[:lower:]' '[:upper:]')"
    while true; do
        out="$(ros2 lifecycle set /overlume_node "${transition}" 2>&1)" || true
        if grep -q "Transitioning successful" <<<"${out}"; then
            return 0
        fi
        if grep -q "Transitioning failed" <<<"${out}"; then
            echo "${upper} failed: on_${transition} callback rejected the transition." >&2
            echo "  see node log: ${LOG_DIR}/overlume_node.log" >&2
            return 1
        fi
        tries=$((tries + 1))
        if [[ "${tries}" -ge 15 ]]; then
            echo "${upper} failed after ${tries} tries (node never became reachable)" >&2
            echo "  see node log: ${LOG_DIR}/overlume_node.log" >&2
            return 1
        fi
        sleep 1
    done
}
lifecycle_set_retry configure
lifecycle_set_retry activate

if [[ "${LIVE}" == "1" ]]; then
    # Live mode: the stack publishes /tf itself (no /tf_raw remap to bridge,
    # and flattening z would be WRONG against real TF), and there is no bag.
    echo "[live] skipping tf_flatten_fixture.py and bag playback -- reading live topics"
else
    echo "[launch] tf_flatten_fixture.py (log: ${LOG_DIR}/tf_flatten.log)"
    python3 "${REPO_ROOT}/tools/tf_flatten_fixture.py" \
        > "${LOG_DIR}/tf_flatten.log" 2>&1 &
    track_child "$!"

    echo "[launch] ros2 bag play --loop (log: ${LOG_DIR}/bag_play.log)"
    # stdin MUST be /dev/null: `set -m` (line 38) puts this job in its own
    # BACKGROUND process group, and rosbag2 with a TTY on stdin enables keyboard
    # controls and reads the terminal -- which SIGTTIN-stops a background group
    # before it prints a single byte. Symptom: 0-byte bag_play.log, no /clock,
    # no ego/map, only when launched from an interactive terminal (2026-08-20).
    ros2 bag play "${BAG}" --loop --clock \
        --qos-profile-overrides-path "${QOS}" --remap /tf:=/tf_raw \
        < /dev/null > "${LOG_DIR}/bag_play.log" 2>&1 &
    track_child "$!"
fi

echo "[launch] vcam_ws_bridge.py (log: ${LOG_DIR}/vcam_ws_bridge.log)"

python3 "${REPO_ROOT}/tools/vcam_ws_bridge.py" --local-mode \
    > "${LOG_DIR}/vcam_ws_bridge.log" 2>&1 &
track_child "$!"

if [[ "${NO_GUI}" != "1" && -n "${DISPLAY:-}" ]]; then
    echo "[launch] vcam_gui.py (log: ${LOG_DIR}/vcam_gui.log)"
    python3 "${REPO_ROOT}/tools/vcam_gui.py" \
        > "${LOG_DIR}/vcam_gui.log" 2>&1 &
    track_child "$!"
else
    echo "[viewer] GUI skipped (--no-gui or no DISPLAY). To view the stream:"
    echo "  rqt_image_view /rendering/image"
fi

# -------------------------------------------------------------- health gate
if [[ "${LIVE}" == "1" ]]; then
    echo "[health] waiting up to 20s for >=25 Hz on /rendering/image," \
         "valid ego_state, and a live /hd_map_local_elements feed ..."
else
    echo "[health] waiting up to 20s for >=25 Hz on /rendering/image," \
         "valid ego_state with z==0.0, and a live /hd_map_local_elements feed ..."
fi

read_hz() {
    timeout -k 2 4 ros2 topic hz /rendering/image 2>/dev/null \
        | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}'
}

# ego_state is std_msgs/Float64MultiArray: data = [x, y, z, heading, speed, valid]
read_ego_z_valid() {
    local out z valid
    out="$(timeout -k 2 3 ros2 topic echo --once /overlume_node/ego_state 2>/dev/null || true)"
    z="$(printf '%s\n' "${out}" | awk '/^data:/{f=1;next} f&&/^- /{n++; if(n==3){print $2; exit}}')"
    valid="$(printf '%s\n' "${out}" | awk '/^data:/{f=1;next} f&&/^- /{n++; if(n==6){print $2; exit}}')"
    printf '%s %s\n' "${z:-}" "${valid:-}"
}

read_hd_map_hz() {
    timeout -k 2 4 ros2 topic hz /hd_map_local_elements 2>/dev/null \
        | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}'
}


read_diagnostics_hz() {
    timeout -k 2 4 ros2 topic hz /overlume_node/diagnostics 2>/dev/null \
        | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}'
}

DEADLINE=$((SECONDS + 40))
HZ=""
EGO_Z=""
EGO_VALID=""
HD_MAP_HZ=""
DIAG_HZ=""
PASS=0
while [[ "${SECONDS}" -lt "${DEADLINE}" ]]; do
    HZ="$(read_hz || true)"
    read -r EGO_Z EGO_VALID < <(read_ego_z_valid)
    HD_MAP_HZ="$(read_hd_map_hz || true)"
    DIAG_HZ="$(read_diagnostics_hz || true)"
    HZ_OK=0
    if [[ -n "${HZ}" ]] && awk -v h="${HZ}" 'BEGIN{exit !(h>=25)}'; then
        HZ_OK=1
    fi
    EGO_OK=0
    # z==0.0 asserts the tf flattener's output -- a bag-rig invariant. Live
    # TF carries real z, so live mode checks validity only.
    if [[ "${LIVE}" == "1" ]]; then
        [[ "${EGO_VALID}" == "1.0" ]] && EGO_OK=1
    elif [[ "${EGO_VALID}" == "1.0" && "${EGO_Z}" == "0.0" ]]; then
        EGO_OK=1
    fi

    HD_MAP_OK=0
    if [[ -n "${HD_MAP_HZ}" ]] && awk -v h="${HD_MAP_HZ}" 'BEGIN{exit !(h>=1)}'; then
        HD_MAP_OK=1
    fi

    DIAG_OK=0
    if [[ -n "${DIAG_HZ}" ]] && awk -v h="${DIAG_HZ}" 'BEGIN{exit !(h>=1)}'; then
        DIAG_OK=1
    fi
    if [[ "${HZ_OK}" == "1" && "${EGO_OK}" == "1" && "${HD_MAP_OK}" == "1" && "${DIAG_OK}" == "1" ]]; then
        PASS=1
        break
    fi
done

echo "=============================================================="
if [[ "${PASS}" == "1" ]]; then
    echo "PASS  /rendering/image @ ${HZ} Hz  |  ego_state valid=${EGO_VALID} z=${EGO_Z}" \
         " |  /hd_map_local_elements @ ${HD_MAP_HZ} Hz  |  ~/diagnostics @ ${DIAG_HZ} Hz"
else
    echo "FAIL  /rendering/image @ ${HZ:-no-data} Hz  |  ego_state valid=${EGO_VALID:-?} z=${EGO_Z:-?}" \
         " |  /hd_map_local_elements @ ${HD_MAP_HZ:-no-data} Hz  |  ~/diagnostics @ ${DIAG_HZ:-no-data} Hz"
    if [[ "${LIVE}" == "1" ]]; then
        echo "  live mode: a FAIL can also mean the autonomy stack is not" \
             "publishing (yet) -- check TF and /hd_map_local_elements on the stack side."
    fi
    echo "  logs: ${LOG_DIR}/"
fi
echo "=============================================================="

echo "Rig is up (PASS/FAIL above). Ctrl-C to tear down."
set +e
wait
set -e

[[ "${PASS}" == "1" ]] || exit 1
