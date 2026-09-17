#!/usr/bin/env bash
# Proxy budget probe: overlume_node @1280x720 per quality preset, bag
# replay. Single-process shape as of the unified-engine migration's Task 6
# (VM-095) cutover -- micropilot_rendering_node no longer exists; the
# rnode-cased branches this script used to carry (q1_rnode_idle/
# q1_rnode_mode2, run_case's rn/rmode/rpid plumbing) are dropped, not left
# to reference a deleted package. See budget_probe.md's own procedure text
# (reworded to match) and Results (a)/(d) for what those rows used to mean.
set -o pipefail
REPO=/home/ag7/Documents/TPSProjector
BAG=$HOME/TPSProjector-fixtures/epic2_fixtures_full
QOS=$HOME/TPSProjector-fixtures/qos_full.yaml
OUT=/tmp/claude-1000/-home-ag7-Documents-TPSProjector/6c5b4a8f-2dca-473f-b964-298e33a2bed7/scratchpad/probe
mkdir -p "$OUT"
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
source "$REPO/ros/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
VPARAMS="$(ros2 pkg prefix overlume_ros)/share/overlume_ros/config/default_params.yaml"
PIDS=()
cleanup(){ for pat in "visualization_[n]ode" "bag pla[y]" "tf_flatten_fixtur[e]"; do for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill "$p" 2>/dev/null; done; done; sleep 2; for pat in "visualization_[n]ode" "bag pla[y]"; do for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done; done; PIDS=(); }
trap cleanup EXIT
set -m
lc(){ local n=$1 t=$2 i; for i in $(seq 1 20); do out=$(ros2 lifecycle set "$n" "$t" 2>&1); grep -q "Transitioning successful" <<<"$out" && return 0; grep -q "Transitioning failed" <<<"$out" && { echo "$n $t FAILED: $out"; return 1; }; sleep 1; done; echo "$n $t timeout"; return 1; }
hz(){ timeout 14 ros2 topic hz "$1" --window 100 2>/dev/null | grep -oE "average rate: [0-9.]+" | tail -1 | awk '{print $3}'; }
gpu(){ nvidia-smi dmon -s um -c 8 2>/dev/null | awk 'NR>2 && $1!~/#/ {s+=$2; m+=$4; n++} END{ if(n) printf "%.0f %.0f", s/n, m/n; else print "na na"}'; }
cpu(){ top -b -n 3 -d 2 -p "$1" 2>/dev/null | awk -v p="$1" '$1==p {c=$9} END{print c+0}'; }
run_case(){ # name quality
  local name=$1 q=$2 vpid
  echo "=== case $name (quality=$q)"
  ros2 run overlume_ros overlume_node --ros-args --params-file "$VPARAMS" \
    -p initial_mode:=3 -p use_sim_time:=true -p quality:=$q -p out_width:=1280 -p out_height:=720 -p profile:=urban \
    > "$OUT/$name.viz.log" 2>&1 & PIDS+=($!)
  sleep 3
  vpid=$(pgrep -f "lib/overlume_ros/visualization_[n]ode" | head -1)
  if ! lc /overlume_node configure || ! lc /overlume_node activate; then cleanup; return 1; fi
  python3 "$REPO/tools/tf_flatten_fixture.py" > "$OUT/$name.tf.log" 2>&1 & PIDS+=($!)
  ros2 bag play "$BAG" --loop --clock --qos-profile-overrides-path "$QOS" --remap /tf:=/tf_raw < /dev/null > "$OUT/$name.bag.log" 2>&1 & PIDS+=($!)
  sleep 12
  local h g c
  h=$(hz /rendering/image); g=$(gpu); c=$(cpu "$vpid")
  echo "RESULT $name q=$q image_hz=${h:-0} gpu_sm%_mem%=$g viz_cpu%=$c" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/$name.viz.log" | sed 's/^/  viz log warn+err lines: /'
  cleanup; sleep 2
}
cleanup
CASES="${*:-q0_alone:0 q1_alone:1 q2_alone:2}"
for c in $CASES; do IFS=: read -r n q <<<"$c"; run_case "$n" "$q"; done
echo DONE
