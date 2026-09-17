#!/usr/bin/env bash
# VM-091 Task 2 Step 5 perf gate. Real fixture bag (six live cameras), not
# epic2_fixtures_full (no camera topics -- budget_probe.md's own existing
# rows are not a valid baseline here, see the plan's Step 5).
set -o pipefail
REPO=/home/ag7/Documents/TPSProjector
BAG=$HOME/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11
# VM-091 gate close-out finding 5: OUT defaults to a fresh mktemp -d dir (the
# usual ${OUT:-...} pattern) instead of a hardcoded session-scratchpad path
# outside the repo -- override with OUT=... to keep results. SAMPLER is the
# committed copy next to this script, never a scratchpad path.
OUT="${OUT:-$(mktemp -d)}"
SAMPLER="$(dirname "$0")/sample_diagnostics.py"
mkdir -p "$OUT"
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
source "$REPO/ros/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
VPARAMS="$(ros2 pkg prefix overlume_ros)/share/overlume_ros/config/default_params.yaml"

cleanup(){
  for pat in "visualization_[n]ode" "bag pla[y]" "synthetic_odom_publishe[r]"; do
    for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill "$p" 2>/dev/null; done
  done
  sleep 2
  for pat in "visualization_[n]ode" "bag pla[y]" "synthetic_odom_publishe[r]"; do
    for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done
  done
}
trap cleanup EXIT
set -m

lc(){ local n=$1 t=$2 i; for i in $(seq 1 20); do out=$(ros2 lifecycle set "$n" "$t" 2>&1); grep -q "Transitioning successful" <<<"$out" && return 0; grep -q "Transitioning failed" <<<"$out" && { echo "$n $t FAILED: $out"; return 1; }; sleep 1; done; echo "$n $t timeout"; return 1; }
hz(){ timeout 14 ros2 topic hz "$1" --window 100 2>/dev/null | grep -oE "average rate: [0-9.]+" | tail -1 | awk '{print $3}'; }
gpu(){ nvidia-smi dmon -s um -c 8 2>/dev/null | awk 'NR>2 && $1!~/#/ {s+=$2; m+=$4; n++} END{ if(n) printf "%.0f %.0f", s/n, m/n; else print "na na"}'; }
cpu(){ top -b -n 3 -d 2 -p "$1" 2>/dev/null | awk -v p="$1" '$1==p {c=$9} END{print c+0}'; }

run_case(){ # name bowl_enabled [odom_topic]
  local name=$1 bowl=$2 odom=${3:-} vpid
  echo "=== case $name (bowl_enabled=$bowl odom_topic=${odom:-<none>})"
  cleanup >/dev/null 2>&1
  ros2 run overlume_ros overlume_node --ros-args --params-file "$VPARAMS" \
    -p initial_mode:=3 -p use_sim_time:=true -p quality:=1 -p out_width:=1280 -p out_height:=720 \
    -p profile:=urban -p bowl_enabled:=$bowl ${odom:+-p odom_topic:=$odom} \
    > "$OUT/$name.viz.log" 2>&1 &
  sleep 3
  vpid=$(pgrep -f "lib/overlume_ros/visualization_[n]ode" | head -1)
  if ! lc /overlume_node configure || ! lc /overlume_node activate; then cleanup; return 1; fi

  # Fresh single-pass playback (no --loop -- a looping bag's clock jump
  # stalls sim-time timers, per this task's own instructions). --clock so
  # use_sim_time timers advance; sensor topics are SensorDataQoS
  # (best_effort) on both the recorder side and this node's subscriptions.
  ros2 bag play "$BAG" --clock --rate 1.0 < /dev/null > "$OUT/$name.bag.log" 2>&1 &
  sleep 15  # let the bag reach steady playback + camera info/first frames arrive

  # bowl_on_driving case (finding 6): the fixture bag carries no odometry at
  # all, so a real rig_delta()/twist_at() per-tick integration cost never
  # runs without this synthetic constant-vx/wz publisher on $odom.
  if [ -n "$odom" ]; then
    python3 "$(dirname "$0")/synthetic_odom_publisher.py" "$odom" 2.0 0.1 \
      > "$OUT/$name.odom.log" 2>&1 &
  fi

  # Live re-bake check (VM-091 close-out review, minor 1): a bowl-param edit
  # mid-run must actually re-bake -- asserted against the node's own INFO
  # line, only on the plain bowl_on case (bowl active, no other churn).
  if [ "$name" = "bowl_on" ]; then
    ros2 param set /overlume_node bowl_R0 12.0 > /dev/null 2>&1 || true
    sleep 1
    if grep -q "bowl: re-baked (live param change)" "$OUT/$name.viz.log"; then
      echo "  live re-bake: OK" | tee -a "$OUT/results.txt"
    else
      echo "  live re-bake: FAILED (no re-bake log line after ros2 param set)" | tee -a "$OUT/results.txt"
    fi
  fi

  python3 "$SAMPLER" 14 > "$OUT/$name.diag.log" 2>&1 &
  local diag_pid=$!
  local h g c
  h=$(hz /rendering/image)
  g=$(gpu)
  c=$(cpu "$vpid")
  wait "$diag_pid"
  local diag; diag=$(cat "$OUT/$name.diag.log")
  echo "RESULT $name bowl_enabled=$bowl odom_topic=${odom:-<none>} image_hz=${h:-0} gpu_sm%_mem%=$g viz_cpu%=$c $diag" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/$name.viz.log" | sed 's/^/  viz log warn+err lines: /'
  cleanup; sleep 2
}

cleanup >/dev/null 2>&1
: > "$OUT/results.txt"
run_case bowl_off false
run_case bowl_on true
run_case bowl_on_driving true /synthetic/odom
echo DONE
echo "OUT=$OUT"
