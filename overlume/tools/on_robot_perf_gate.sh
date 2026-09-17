#!/usr/bin/env bash
# Task 6 (VM-095) Step 1 -- the on-robot budget rerun, closing VM-043 for
# real (Decision 10): merged node as the ONLY rendering process, all three
# modes, real camera+lidar input, render_ms p50/p99 + GPU/CPU + GPU model.
#
# No robot hardware was reachable from this session -- run on this dev box
# (RTX 3090) as the most robot-representative box available (plan's own
# allowance, Global Constraints). budget_probe.md's Results (b) states this
# plainly as the dev-box proxy standing in for the robot; the on-actual-
# robot rerun stays a named open item for deployment.
#
# Same procedure shape as bowl_perf_gate.sh/hybrid_perf_gate.sh (this
# epic's own precedent): real fixture bag (six cameras + lidar), fresh
# single-pass playback, SensorDataQoS/best_effort, ROS_DOMAIN_ID=93.
# UNLIKE those two gates, both bowl_enabled AND hybrid_enabled are true in
# EVERY case here -- Step 6 flips both shipped defaults to true, so this is
# what production actually runs post-cutover, not an isolated capability
# check.
set -o pipefail
REPO=/home/ag7/Documents/TPSProjector
BAG=$HOME/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11
OUT="${OUT:-$(mktemp -d)}"
SAMPLER="$(dirname "$0")/sample_diagnostics.py"
mkdir -p "$OUT"
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
source "$REPO/ros/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
VPARAMS="$(ros2 pkg prefix overlume_ros)/share/overlume_ros/config/default_params.yaml"

echo "GPU model/class (named fixture gap #1 -- first time this repo records one):" | tee "$OUT/results.txt"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | tee -a "$OUT/results.txt"

cleanup(){
  for pat in "visualization_[n]ode" "bag pla[y]"; do
    for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill "$p" 2>/dev/null; done
  done
  sleep 2
  for pat in "visualization_[n]ode" "bag pla[y]"; do
    for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done
  done
}
trap cleanup EXIT
set -m

lc(){ local n=$1 t=$2 i; for i in $(seq 1 20); do out=$(ros2 lifecycle set "$n" "$t" 2>&1); grep -q "Transitioning successful" <<<"$out" && return 0; grep -q "Transitioning failed" <<<"$out" && { echo "$n $t FAILED: $out"; return 1; }; sleep 1; done; echo "$n $t timeout"; return 1; }
hz(){ timeout 14 ros2 topic hz "$1" --window 100 2>/dev/null | grep -oE "average rate: [0-9.]+" | tail -1 | awk '{print $3}'; }
gpu(){ nvidia-smi dmon -s um -c 8 2>/dev/null | awk 'NR>2 && $1!~/#/ {s+=$2; m+=$4; n++} END{ if(n) printf "%.0f %.0f", s/n, m/n; else print "na na"}'; }
cpu(){ top -b -n 3 -d 2 -p "$1" 2>/dev/null | awk -v p="$1" '$1==p {c=$9} END{print c+0}'; }

run_case(){ # name render_mode
  local name=$1 mode=$2 vpid
  echo "=== case $name (render_mode=$mode, bowl_enabled=true, hybrid_enabled=true)"
  cleanup >/dev/null 2>&1
  ros2 run overlume_ros overlume_node --ros-args --params-file "$VPARAMS" \
    -p initial_mode:=3 -p use_sim_time:=true -p quality:=1 -p out_width:=1280 -p out_height:=720 \
    -p profile:=urban -p bowl_enabled:=true -p hybrid_enabled:=true \
    -p pointcloud_topic:=/iv_points_fusion -p render_mode:=$mode \
    > "$OUT/$name.viz.log" 2>&1 &
  sleep 3
  vpid=$(pgrep -f "lib/overlume_ros/visualization_[n]ode" | head -1)
  if ! lc /overlume_node configure || ! lc /overlume_node activate; then cleanup; return 1; fi

  ros2 bag play "$BAG" --clock --rate 1.0 < /dev/null > "$OUT/$name.bag.log" 2>&1 &
  sleep 20  # 6 cameras' CameraInfo complete + bowl bake + lidar flowing, same warm-up as hybrid_perf_gate.sh

  python3 "$SAMPLER" 14 > "$OUT/$name.diag.log" 2>&1 &
  local diag_pid=$!
  local h g c
  h=$(hz /rendering/image)
  g=$(gpu)
  c=$(cpu "$vpid")
  wait "$diag_pid"
  local diag; diag=$(cat "$OUT/$name.diag.log")
  echo "RESULT $name render_mode=$mode image_hz=${h:-0} gpu_sm%_mem%=$g viz_cpu%=$c $diag" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/$name.viz.log" | sed 's/^/  viz log warn+err lines: /'
  cleanup; sleep 2
}
cleanup
CASES="${*:-bowl:1 hybrid:2 free_look:3}"
for c in $CASES; do IFS=: read -r n m <<<"$c"; run_case "$n" "$m"; done
echo DONE
