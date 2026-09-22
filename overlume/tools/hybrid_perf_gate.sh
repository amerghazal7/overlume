#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BAG=$HOME/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11
OUT="${OUT:-$(mktemp -d)}"
SAMPLER="$(dirname "$0")/sample_diagnostics.py"
mkdir -p "$OUT"
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
source "$REPO/ros/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
VPARAMS="$(ros2 pkg prefix overlume_ros)/share/overlume_ros/config/default_params.yaml"

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

run_case(){
  local name=$1 mode=$2 hybrid=$3
  echo "=== case $name (render_mode=$mode hybrid_enabled=$hybrid)"
  cleanup >/dev/null 2>&1
  ros2 run overlume_ros overlume_node --ros-args --params-file "$VPARAMS" \
    -p initial_mode:=3 -p use_sim_time:=true -p quality:=1 -p out_width:=1280 -p out_height:=720 \
    -p profile:=urban -p bowl_enabled:=true -p render_mode:=$mode \
    -p hybrid_enabled:=$hybrid -p pointcloud_topic:=/iv_points_fusion \
    > "$OUT/$name.viz.log" 2>&1 &
  sleep 3
  if ! lc /overlume_node configure || ! lc /overlume_node activate; then cleanup; return 1; fi

  ros2 bag play "$BAG" --clock --rate 1.0 < /dev/null > "$OUT/$name.bag.log" 2>&1 &
  sleep 20

  python3 "$SAMPLER" 14 > "$OUT/$name.diag.log" 2>&1 &
  local diag_pid=$!
  local h; h=$(hz /rendering/image)
  wait "$diag_pid"
  local diag; diag=$(cat "$OUT/$name.diag.log")
  echo "RESULT $name render_mode=$mode hybrid_enabled=$hybrid image_hz=${h:-0} $diag" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/$name.viz.log" | sed 's/^/  viz log warn+err lines: /'
  cleanup; sleep 2
}

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
