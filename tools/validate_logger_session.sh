#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${OVERLUME_SESSION:-${HOME}/session_2026-09-01_13-57-00}"
COLLECTOR_INSTALL="${MP_COLLECTOR_INSTALL:-${HOME}/micropilot/micropilot_data_collector/install/setup.bash}"
PROFILE=replay
GPS_TOPIC=/fixposition/odometry_llh
ARM=""
VALIDATE_ARGS=()
LOG_DIR=/tmp/overlume_validate

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-gui) VALIDATE_ARGS+=(--no-gui); shift ;;
        --arm) ARM="$2"; shift 2 ;;
        --anchor-height-offset) VALIDATE_ARGS+=(--param "geo_anchor_height_offset_m:=$2"); shift 2 ;;
        --profile) PROFILE="$2"; shift 2 ;;
        --gps-topic) GPS_TOPIC="$2"; shift 2 ;;
        --collector-install) COLLECTOR_INSTALL="$2"; shift 2 ;;
        --) shift; VALIDATE_ARGS+=("$@"); break ;;
        -h|--help) sed -n '5,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown arg: $1" >&2; exit 1 ;;
        *) SESSION="$1"; shift ;;
    esac
done
SESSION="${SESSION%/}"

for f in config/player.yaml config/topics.yaml config/sensors_extrinsic_calib.yaml; do
    [[ -f "${SESSION}/${f}" ]] || { echo "not a logger session (missing ${f}): ${SESSION}" >&2; exit 1; }
done
[[ -f "${COLLECTOR_INSTALL}" ]] || { echo "data collector install not found: ${COLLECTOR_INSTALL}" >&2; exit 1; }
[[ -d "${REPO_ROOT}/ros/install" ]] || { echo "ros/install not found. Run ros/colcon_build.sh" >&2; exit 1; }
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${COLLECTOR_INSTALL}"
set -u

publisher_count() {
    timeout -k 2 5 ros2 topic info "$1" 2>/dev/null | sed -n 's/^Publisher count: //p'
}

if [[ "$(publisher_count /clock)" =~ ^[1-9] ]]; then
    echo "FATAL: /clock already has a publisher on this ROS domain -- stop the other player first." >&2
    exit 1
fi

PLAYER_YAML="${LOG_DIR}/player_$(basename "${SESSION}").yaml"
TF_LIST="${LOG_DIR}/static_tfs_$(basename "${SESSION}").txt"
python3 - "${SESSION}" "${PLAYER_YAML}" "${TF_LIST}" <<'EOF'
import math, sys, yaml
session, player_out, tf_out = sys.argv[1:4]

p = yaml.safe_load(open(f"{session}/config/player.yaml"))
p["mode"] = "asynchronous"
p.setdefault("playback", {}).update(start_paused=False, loop=True, publish_clock=True)
p.setdefault("sync", {})["require_all_topics"] = False
p.setdefault("dashboard", {})["enable"] = False
p.setdefault("controls", {})["keyboard"] = False
yaml.safe_dump(p, open(player_out, "w"), sort_keys=False)

c = yaml.safe_load(open(f"{session}/config/sensors_extrinsic_calib.yaml"))
renames = dict(c.get("frame_renames") or {})
renames.setdefault("fixposition", "FP_POI")
with open(tf_out, "w") as out:
    for key, frame in renames.items():
        blk = c.get(key) or {}
        m = next((v for k, v in blk.items() if k.endswith("_to_ego")), None)
        if m is None:
            continue
        (r00, r01, r02, tx), (r10, r11, r12, ty), (r20, r21, r22, tz) = [row[:4] for row in m[:3]]
        tr = r00 + r11 + r22  # rotation matrix -> quaternion (Shepperd)
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2; qw, qx, qy, qz = 0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s
        elif r00 > r11 and r00 > r22:
            s = math.sqrt(1.0 + r00 - r11 - r22) * 2; qw, qx, qy, qz = (r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s
        elif r11 > r22:
            s = math.sqrt(1.0 + r11 - r00 - r22) * 2; qw, qx, qy, qz = (r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s
        else:
            s = math.sqrt(1.0 + r22 - r00 - r11) * 2; qw, qx, qy, qz = (r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s
        out.write(f"{frame} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
EOF
echo "[config] player: ${PLAYER_YAML}"
echo "[config] static TFs (base_link -> frame  x y z  qx qy qz qw):"
sed 's/^/    /' "${TF_LIST}"

PGIDS=()
VALIDATE_PID=""
teardown() {
    trap - INT TERM EXIT
    if [[ -n "${VALIDATE_PID}" ]]; then
        kill -TERM "${VALIDATE_PID}" 2>/dev/null || true
        wait "${VALIDATE_PID}" 2>/dev/null || true
    fi
    for g in "${PGIDS[@]:-}"; do
        [[ -n "${g}" ]] && kill -TERM -- "-${g}" 2>/dev/null || true
    done
    sleep 1
    for g in "${PGIDS[@]:-}"; do
        [[ -n "${g}" ]] && kill -KILL -- "-${g}" 2>/dev/null || true
    done
    echo "player + static TFs down"
}
trap teardown INT TERM EXIT

echo "[launch] mp_play (log: ${LOG_DIR}/mp_play.log)"
setsid ros2 run data_logger mp_play --session "${SESSION}" --player "${PLAYER_YAML}" \
    < /dev/null > "${LOG_DIR}/mp_play.log" 2>&1 &
PGIDS+=("$!")
PLAYER_PID=$!

while read -r frame x y z qx qy qz qw; do
    setsid ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id "${frame}" \
        --x "${x}" --y "${y}" --z "${z}" --qx "${qx}" --qy "${qy}" --qz "${qz}" --qw "${qw}" \
        < /dev/null > "${LOG_DIR}/static_tf_${frame}.log" 2>&1 &
    PGIDS+=("$!")
done < "${TF_LIST}"

echo "[wait] for /clock and ${GPS_TOPIC} from the player (up to 90 s) ..."
DEADLINE=$((SECONDS + 90))
until [[ "$(publisher_count /clock)" =~ ^[1-9] && "$(publisher_count "${GPS_TOPIC}")" =~ ^[1-9] ]]; do
    if ! kill -0 "${PLAYER_PID}" 2>/dev/null; then
        echo "FATAL: mp_play exited. Tail of ${LOG_DIR}/mp_play.log:" >&2; tail -20 "${LOG_DIR}/mp_play.log" >&2; exit 1
    fi
    if [[ "${SECONDS}" -ge "${DEADLINE}" ]]; then
        echo "FATAL: player published no /clock + ${GPS_TOPIC} within 90 s. See ${LOG_DIR}/mp_play.log" >&2; exit 1
    fi
    sleep 2
done
echo "[wait] player is publishing"

echo "[launch] validate_visual_mode.sh --live --profile ${PROFILE} --param gps_topic:=${GPS_TOPIC} ${VALIDATE_ARGS[*]:-}"
: > "${LOG_DIR}/validate_logger_session.log"
LIVE_SIM_TIME=true "${REPO_ROOT}/tools/validate_visual_mode.sh" --live --profile "${PROFILE}" \
    --param "gps_topic:=${GPS_TOPIC}" ${VALIDATE_ARGS[@]+"${VALIDATE_ARGS[@]}"} \
    > >(tee "${LOG_DIR}/validate_logger_session.log") 2>&1 &
VALIDATE_PID=$!

if [[ -n "${ARM}" ]]; then
    until grep -q '^Rig is up' "${LOG_DIR}/validate_logger_session.log" 2>/dev/null; do
        kill -0 "${VALIDATE_PID}" 2>/dev/null || { wait "${VALIDATE_PID}"; exit $?; }
        sleep 2
    done
    echo "[arm] set_environment_source preset=${ARM} (retrying until the geo anchor solves, 120 s) ..."
    python3 - "${ARM}" <<'EOF' || true
import asyncio, json, sys, time, websockets
preset, deadline = sys.argv[1], time.time() + 120
async def arm():
    while time.time() < deadline:
        try:
            async with websockets.connect("ws://localhost:8765", open_timeout=5) as ws:
                await ws.send(json.dumps({"cmd": "set_environment_source", "preset": preset}))
                async for raw in ws:
                    msg = json.loads(raw)
                    if msg.get("type") == "ack" and msg.get("cmd") == "set_environment_source":
                        if msg.get("success"):
                            print(f"[arm] OK: environment source = {preset}", flush=True); return 0
                        print(f"[arm] refused: {msg.get('reason', '?')}", flush=True); break
        except (OSError, websockets.WebSocketException) as e:
            print(f"[arm] bridge not ready ({type(e).__name__})", flush=True)
        await asyncio.sleep(5)
    print("[arm] FAIL: not armed within 120 s", flush=True); return 1
sys.exit(asyncio.run(arm()))
EOF
fi

set +e
wait "${VALIDATE_PID}"
STATUS=$?
set -e
exit "${STATUS}"
