#!/usr/bin/env bash
# validate_visual_mode.sh — one command to stand up the full visual-mode
# validation rig (visualization_node + fixture bag + tf flattener + vcam
# bridge/GUI) against the recorded fixture bag, and report PASS/FAIL.
#
# Usage: tools/validate_visual_mode.sh [--bag PATH] [--qos PATH] [--no-gui]
#                                       [--build] [--profile NAME]
#
# ==========================================================================
# MILESTONE UPDATE LOG (Epic 2 — append one line per task as it lands; this
# script is a living deliverable, keep it in sync with what is visually
# checkable after each milestone. User directive 2026-08-20.)
#
#   2026-08-20  Rig created (pre-Task-1). Visually validatable: mode 3,
#               dark_adas ego-following ground+grid (Epic 1), against the
#               recorded fixture bag with TF flattened to z=0 for playback.
#               --profile is accepted but not yet forwarded to anything the
#               node understands (profile.yaml loading is Task 1/VM-020).
#   2026-08-20  Task 1/VM-020: --profile now selects config/<name>_profile.yaml
#               on the node side; a bad/missing name is a fatal on_configure
#               (node stays unconfigured, script reports CONFIGURE failed).
#               Nothing new is visually checkable yet -- no adapter subscribes.
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
BAG="${HOME}/TPSProjector-fixtures/epic2_fixtures_full"
QOS="${HOME}/TPSProjector-fixtures/qos_full.yaml"
NO_GUI=0
DO_BUILD=0
PROFILE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag) BAG="$2"; shift 2 ;;
        --qos) QOS="$2"; shift 2 ;;
        --no-gui) NO_GUI=1; shift ;;
        --build) DO_BUILD=1; shift ;;
        --profile) PROFILE="$2"; shift 2 ;;
        -h|--help)
            grep '^# ' "${BASH_SOURCE[0]}" | head -6 | sed 's/^# //'
            exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

LOG_DIR=/tmp/mpviz_validate
mkdir -p "${LOG_DIR}"

# ------------------------------------------------------------- teardown-first
# Kill any prior rig this script (or a previous run of it, or a hand-run
# session) left behind. pgrep -f matches full command lines; a naive
# unanchored pattern like "visualization_node" matches ANY process whose
# argv merely contains that text -- a `tail -f .../visualization_node.cpp`
# bystander, a colcon build's cc1plus compiling visualization_node.cpp, or
# (worst) the shell that is invoking this very script when it's wrapped by
# something that echoes its own command line into argv. Every pattern below
# is anchored to a path/phrase that only the actual rig member's argv
# contains -- the compiled binary is matched by its unique install path
# (not the bare binary name: process `comm` is truncated to 15 bytes by the
# kernel, so `pgrep -x visualization_node` never matches "visualization_n").
# We also still walk pgrep's PID list by hand and skip our own pid and our
# own process group, belt-and-suspenders. This bug bit us for real in the
# 2026-08-19/20 sessions; do not "simplify" back to bare-word pkill -f.
RIG_PATTERNS=(
    "ros2 run micropilot_visualization_node"
    "lib/micropilot_visualization_node/visualization_node"
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

# ---------------------------------------------------------------- prereqs
if [[ ! -d "${REPO_ROOT}/cuda/install/ros_apps" ]]; then
    if [[ "${DO_BUILD}" == "1" ]]; then
        :  # built below
    else
        echo "cuda/install/ros_apps not found. Run cuda/scripts/ros_apps_build/colcon_build.sh" \
             "or re-run with --build." >&2
        exit 1
    fi
fi

if [[ "${DO_BUILD}" == "1" ]]; then
    echo "[build] running colcon_build.sh ..."
    ( cd "${REPO_ROOT}/cuda/scripts/ros_apps_build" && ./colcon_build.sh )
fi

if [[ ! -e "${BAG}" ]]; then
    echo "bag not found: ${BAG}" >&2
    exit 1
fi
if [[ ! -e "${QOS}" ]]; then
    echo "qos override file not found: ${QOS}" >&2
    exit 1
fi

# ------------------------------------------------------------------- ROS env
set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/cuda/install/ros_apps/setup.bash"
set -u

# ---------------------------------------------------------------- launch rig
# Track both the backgrounded PID and its process group. `ros2 run`/`ros2 bag
# play` do not always exec-replace themselves -- they can fork a real child
# (verified live: the visualization_node binary ran under a DIFFERENT pid
# than the `ros2 run` job's own $!, reparented to pid 1 once the launcher
# exited). Killing only the recorded $! then leaves that child running. A
# forked child always inherits its parent's process group though, and the
# group id is stable even after the group's leader exits -- so teardown()
# kills by recorded PGID (with the raw PIDs as a belt-and-suspenders
# fallback), not by chasing individual descendant PIDs.
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

echo "[launch] visualization_node (log: ${LOG_DIR}/visualization_node.log)"
PROFILE_ARGS=()
if [[ -n "${PROFILE}" ]]; then
    PROFILE_ARGS=(-p "profile:=${PROFILE}")
fi
ros2 run micropilot_visualization_node visualization_node --ros-args \
    -p initial_mode:=3 -p use_sim_time:=true "${PROFILE_ARGS[@]}" \
    > "${LOG_DIR}/visualization_node.log" 2>&1 &
track_child "$!"

echo "[lifecycle] configure + activate (retrying while the node registers)..."
# `ros2 lifecycle set` exits 0 even when the transition CALLBACK fails -- it
# only prints "Transitioning failed" to stdout/stderr and leaves the node in
# its old state. So we grep the actual transition result instead of trusting
# the CLI's exit status. A real "Transitioning failed" is deterministic
# (on_configure/on_activate rejected it) -- retrying won't change that, so
# fail immediately and name the transition + node log. Only a CLI/service
# error (node hasn't registered its lifecycle service yet) is worth retrying.
lifecycle_set_retry() {
    local transition="$1" tries=0 out upper
    upper="$(printf '%s' "${transition}" | tr '[:lower:]' '[:upper:]')"
    while true; do
        out="$(ros2 lifecycle set /visualization_node "${transition}" 2>&1)" || true
        if grep -q "Transitioning successful" <<<"${out}"; then
            return 0
        fi
        if grep -q "Transitioning failed" <<<"${out}"; then
            echo "${upper} failed: on_${transition} callback rejected the transition." >&2
            echo "  see node log: ${LOG_DIR}/visualization_node.log" >&2
            return 1
        fi
        tries=$((tries + 1))
        if [[ "${tries}" -ge 15 ]]; then
            echo "${upper} failed after ${tries} tries (node never became reachable)" >&2
            echo "  see node log: ${LOG_DIR}/visualization_node.log" >&2
            return 1
        fi
        sleep 1
    done
}
lifecycle_set_retry configure
lifecycle_set_retry activate

echo "[launch] tf_flatten_fixture.py (log: ${LOG_DIR}/tf_flatten.log)"
python3 "${REPO_ROOT}/tools/tf_flatten_fixture.py" \
    > "${LOG_DIR}/tf_flatten.log" 2>&1 &
track_child "$!"

echo "[launch] ros2 bag play --loop (log: ${LOG_DIR}/bag_play.log)"
ros2 bag play "${BAG}" --loop --clock \
    --qos-profile-overrides-path "${QOS}" --remap /tf:=/tf_raw \
    > "${LOG_DIR}/bag_play.log" 2>&1 &
track_child "$!"

echo "[launch] vcam_ws_bridge.py (log: ${LOG_DIR}/vcam_ws_bridge.log)"
python3 "${REPO_ROOT}/tools/vcam_ws_bridge.py" \
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
echo "[health] waiting up to 20s for >=25 Hz on /rendering/image and" \
     "valid ego_state with z==0.0 ..."

read_hz() {
    timeout 4 ros2 topic hz /rendering/image 2>/dev/null \
        | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}'
}

# ego_state is std_msgs/Float64MultiArray: data = [x, y, z, heading, speed, valid]
read_ego_z_valid() {
    local out z valid
    out="$(timeout 3 ros2 topic echo --once /visualization_node/ego_state 2>/dev/null || true)"
    z="$(printf '%s\n' "${out}" | awk '/^data:/{f=1;next} f&&/^- /{n++; if(n==3){print $2; exit}}')"
    valid="$(printf '%s\n' "${out}" | awk '/^data:/{f=1;next} f&&/^- /{n++; if(n==6){print $2; exit}}')"
    printf '%s %s\n' "${z:-}" "${valid:-}"
}

DEADLINE=$((SECONDS + 20))
HZ=""
EGO_Z=""
EGO_VALID=""
PASS=0
while [[ "${SECONDS}" -lt "${DEADLINE}" ]]; do
    HZ="$(read_hz || true)"
    read -r EGO_Z EGO_VALID < <(read_ego_z_valid)
    HZ_OK=0
    if [[ -n "${HZ}" ]] && awk -v h="${HZ}" 'BEGIN{exit !(h>=25)}'; then
        HZ_OK=1
    fi
    EGO_OK=0
    if [[ "${EGO_VALID}" == "1.0" && "${EGO_Z}" == "0.0" ]]; then
        EGO_OK=1
    fi
    if [[ "${HZ_OK}" == "1" && "${EGO_OK}" == "1" ]]; then
        PASS=1
        break
    fi
done

echo "=============================================================="
if [[ "${PASS}" == "1" ]]; then
    echo "PASS  /rendering/image @ ${HZ} Hz  |  ego_state valid=${EGO_VALID} z=${EGO_Z}"
else
    echo "FAIL  /rendering/image @ ${HZ:-no-data} Hz  |  ego_state valid=${EGO_VALID:-?} z=${EGO_Z:-?}"
    echo "  logs: ${LOG_DIR}/"
fi
echo "=============================================================="

echo "Rig is up (PASS/FAIL above). Ctrl-C to tear down."
set +e
wait
set -e
# PASS is recorded above; a caller (orchestrator/milestone script) chaining
# on this script's exit status must see non-zero on a failed health gate --
# printing "FAIL" and then exiting 0 makes a broken rig indistinguishable
# from a working one. Keeping the rig up for inspection until Ctrl-C is
# still fine; only the final exit code changes.
[[ "${PASS}" == "1" ]] || exit 1
