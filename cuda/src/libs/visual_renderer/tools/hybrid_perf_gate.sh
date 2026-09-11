#!/usr/bin/env bash
# VM-094 (unified-engine migration Task 5) Step 4 perf gate. Real fixture
# bag (six live cameras + /iv_points_fusion lidar), not epic2_fixtures_full
# (no camera/lidar topics at all -- same reasoning as bowl_perf_gate.sh's
# own header comment). Two cases: bowl_baseline (mode 1, no hybrid
# colorization) and hybrid_on (mode 2, real colorization) -- the render_ms
# delta between them IS "the colorization CPU cost specifically" (Step 4's
# own ask), same methodology bowl_perf_gate.sh already used for the bowl
# bake's own cost, not a new diagnostics field.
set -o pipefail
REPO=/home/ag7/Documents/TPSProjector
BAG=$HOME/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11
OUT="${OUT:-$(mktemp -d)}"
SAMPLER="$(dirname "$0")/sample_diagnostics.py"
mkdir -p "$OUT"
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
source "$REPO/cuda/install/ros_apps/setup.bash"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
VPARAMS="$(ros2 pkg prefix micropilot_visualization_node)/share/micropilot_visualization_node/config/default_params.yaml"

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

run_case(){ # name render_mode hybrid_enabled
  local name=$1 mode=$2 hybrid=$3
  echo "=== case $name (render_mode=$mode hybrid_enabled=$hybrid)"
  cleanup >/dev/null 2>&1
  ros2 run micropilot_visualization_node visualization_node --ros-args --params-file "$VPARAMS" \
    -p initial_mode:=3 -p use_sim_time:=true -p quality:=1 -p out_width:=1280 -p out_height:=720 \
    -p profile:=urban -p bowl_enabled:=true -p render_mode:=$mode \
    -p hybrid_enabled:=$hybrid -p pointcloud_topic:=/iv_points_fusion \
    > "$OUT/$name.viz.log" 2>&1 &
  sleep 3
  if ! lc /visualization_node configure || ! lc /visualization_node activate; then cleanup; return 1; fi

  # Fresh single-pass playback (no --loop), best_effort sensor QoS on both
  # ends (SensorDataQoS), matching bowl_perf_gate.sh's own convention.
  ros2 bag play "$BAG" --clock --rate 1.0 < /dev/null > "$OUT/$name.bag.log" 2>&1 &
  sleep 20  # let 6 cameras' CameraInfo complete + bowl bake + lidar start flowing

  python3 "$SAMPLER" 14 > "$OUT/$name.diag.log" 2>&1 &
  local diag_pid=$!
  local h; h=$(hz /rendering/image)
  wait "$diag_pid"
  local diag; diag=$(cat "$OUT/$name.diag.log")
  echo "RESULT $name render_mode=$mode hybrid_enabled=$hybrid image_hz=${h:-0} $diag" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/$name.viz.log" | sed 's/^/  viz log warn+err lines: /'
  cleanup; sleep 2
}

# Fixture bag's own /iv_points_fusion point count (Step 4: "record the
# actual number used") -- sampled once, outside either case, from a fresh
# short playback.
echo "=== sampling /iv_points_fusion point count"
ros2 bag play "$BAG" --clock --rate 1.0 < /dev/null > "$OUT/pc_sample.bag.log" 2>&1 &
sleep 5
PC_MSG="$(timeout 8 ros2 topic echo /iv_points_fusion --once --field width 2>/dev/null)"
PC_H="$(timeout 8 ros2 topic echo /iv_points_fusion --once --field height 2>/dev/null)"
cleanup >/dev/null 2>&1

: > "$OUT/results.txt"
echo "iv_points_fusion width=$PC_MSG height=$PC_H (point_count = width*height)" >> "$OUT/results.txt"
run_case bowl_baseline 1 false
run_case hybrid_on 2 true
echo DONE
echo "OUT=$OUT"
