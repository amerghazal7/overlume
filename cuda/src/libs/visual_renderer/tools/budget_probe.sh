#!/usr/bin/env bash
# Proxy budget probe: visualization_node @1280x720 per quality preset, bag replay,
# optionally beside micropilot_rendering_node (idle on mode 3 / active mode 2, no camera input on bag).
set -o pipefail
REPO=/home/ag7/Documents/TPSProjector
BAG=$HOME/TPSProjector-fixtures/epic2_fixtures_full
QOS=$HOME/TPSProjector-fixtures/qos_full.yaml
OUT=/tmp/claude-1000/-home-ag7-Documents-TPSProjector/6c5b4a8f-2dca-473f-b964-298e33a2bed7/scratchpad/probe
mkdir -p "$OUT"
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
source "$REPO/cuda/install/ros_apps/setup.bash"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
VPARAMS="$(ros2 pkg prefix micropilot_visualization_node)/share/micropilot_visualization_node/config/default_params.yaml"
RPARAMS="$(ros2 pkg prefix micropilot_rendering_node)/share/micropilot_rendering_node/config/default_params.yaml"
PIDS=()
cleanup(){ for pat in "visualization_[n]ode" "rendering_[n]ode" "bag pla[y]" "tf_flatten_fixtur[e]"; do for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill "$p" 2>/dev/null; done; done; sleep 2; for pat in "visualization_[n]ode" "rendering_[n]ode" "bag pla[y]"; do for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done; done; PIDS=(); }
trap cleanup EXIT
set -m
lc(){ local n=$1 t=$2 i; for i in $(seq 1 20); do out=$(ros2 lifecycle set "$n" "$t" 2>&1); grep -q "Transitioning successful" <<<"$out" && return 0; grep -q "Transitioning failed" <<<"$out" && { echo "$n $t FAILED: $out"; return 1; }; sleep 1; done; echo "$n $t timeout"; return 1; }
hz(){ timeout 14 ros2 topic hz "$1" --window 100 2>/dev/null | grep -oE "average rate: [0-9.]+" | tail -1 | awk '{print $3}'; }
gpu(){ nvidia-smi dmon -s um -c 8 2>/dev/null | awk 'NR>2 && $1!~/#/ {s+=$2; m+=$4; n++} END{ if(n) printf "%.0f %.0f", s/n, m/n; else print "na na"}'; }
cpu(){ top -b -n 3 -d 2 -p "$1" 2>/dev/null | awk -v p="$1" '$1==p {c=$9} END{print c+0}'; }
run_case(){ # name quality with_rnode rnode_mode
  local name=$1 q=$2 rn=$3 rmode=$4 vpid rpid
  echo "=== case $name (quality=$q rendering_node=$rn mode=$rmode)"
  ros2 run micropilot_visualization_node visualization_node --ros-args --params-file "$VPARAMS" \
    -p initial_mode:=3 -p use_sim_time:=true -p quality:=$q -p out_width:=1280 -p out_height:=720 -p profile:=urban \
    > "$OUT/$name.viz.log" 2>&1 & PIDS+=($!)
  sleep 3
  vpid=$(pgrep -f "lib/micropilot_visualization_node/visualization_[n]ode" | head -1)
  if ! lc /visualization_node configure || ! lc /visualization_node activate; then cleanup; return 1; fi
  if [ "$rn" = 1 ]; then
    ros2 run micropilot_rendering_node rendering_node --ros-args --params-file "$RPARAMS" -p initial_mode:=$rmode \
      > "$OUT/$name.rnode.log" 2>&1 & PIDS+=($!)
    sleep 4
    rpid=$(pgrep -f "lib/micropilot_rendering_node/rendering_[n]ode" | head -1)
    lc /rendering_node configure; lc /rendering_node activate
    [ "$rmode" = 3 ] && ros2 topic pub --once /rendering/set_mode std_msgs/msg/Int32 "{data: 3}" >/dev/null 2>&1
  fi
  python3 "$REPO/tools/tf_flatten_fixture.py" > "$OUT/$name.tf.log" 2>&1 & PIDS+=($!)
  ros2 bag play "$BAG" --loop --clock --qos-profile-overrides-path "$QOS" --remap /tf:=/tf_raw < /dev/null > "$OUT/$name.bag.log" 2>&1 & PIDS+=($!)
  sleep 12
  local h g c rc
  h=$(hz /rendering/image); g=$(gpu); c=$(cpu "$vpid"); rc=$([ -n "${rpid:-}" ] && cpu "$rpid" || echo -)
  echo "RESULT $name q=$q rnode=$rn rmode=$rmode image_hz=${h:-0} gpu_sm%_mem%=$g viz_cpu%=$c rnode_cpu%=$rc" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/$name.viz.log" | sed 's/^/  viz log warn+err lines: /'
  cleanup; sleep 2
}
cleanup
CASES="${*:-q0_alone:0:0:- q1_alone:1:0:- q2_alone:2:0:- q1_rnode_idle:1:1:3 q1_rnode_mode2:1:1:2}"
for c in $CASES; do IFS=: read -r n q r m <<<"$c"; run_case "$n" "$q" "$r" "$m"; done
echo DONE
