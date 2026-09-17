#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# ── HISTORICAL HARNESS (frozen at the VM-095 cutover, 2026-09-11) ─────────
# This gate measured CUDA-node/Filament-node CO-RESIDENCE during the
# migration rollout window (Task 4 Step 3; results recorded in
# budget_probe.md Results (d) and signoff.md item 6). micropilot_rendering_node
# is retired -- this script can no longer run and is kept as the record of
# HOW that measurement was taken, not as a runnable tool.
echo "mode_consolidation_perf_gate.sh is a HISTORICAL harness: micropilot_rendering_node" >&2
echo "was retired at the VM-095 cutover; the co-residence measurement it took is recorded" >&2
echo "in budget_probe.md Results (d). Nothing to run." >&2
exit 2
# Task 4 (VM-093) Step 3 perf gate: old node (micropilot_rendering_node,
# UNMODIFIED, run from the MAIN checkout's already-built install, READ-ONLY --
# this task never edits or rebuilds that checkout) + this worktree's merged
# node (overlume_ros, camera-ingesting) running together --
# the actual state production is in for the whole rollout window (Decision 7).
#
# Baseline to compare against: Task 2 Step 5a's own same-bag `bowl_off` row
# (budget_probe.md Results (c)) -- NOT budget_probe.md's `q1_rnode_idle`/
# `q1_rnode_mode2` rows, which were measured on epic2_fixtures_full (no
# camera topics at all).
#
# Old node authoritative for modes 1/2 via the mux (initial_mode:=2 on BOTH
# nodes -- old node actually renders CUDA mode 2, fed by real cameras +
# /iv_points_fusion; new node's own active_mode_==2 early-return skips its
# render/publish loop, same as production -- but bowl_enabled:=true keeps its
# camera_ingest_ doing real per-tick work, "still-camera-ingesting", per the
# plan's own Step 3 text).
set -o pipefail
MAIN_REPO=/home/ag7/Documents/TPSProjector
WORKTREE_REPO=/home/ag7/Documents/TPSProjector-vm093
BAG=$HOME/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11
OUT="${OUT:-$(mktemp -d)}"
SAMPLER="$(dirname "$0")/sample_diagnostics.py"
# Review round 1 (2026-09-11): CASE selects which case(s) run --
# "co_residence" (old node + new node), "rnode_alone" (old node only, the
# control), or "both" (default -- co_residence then rnode_alone, so the
# control is no longer a separate throwaway script and budget_probe.md's
# Results (d) can cite THIS file for both rows).
CASE="${CASE:-both}"
mkdir -p "$OUT"
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
# Old node: MAIN checkout's install, READ-ONLY (never rebuilt/edited here).
source "$MAIN_REPO/ros/install/setup.bash"
# New node: this worktree's own scratch colcon install, layered on top.
source "$WORKTREE_REPO/.colcon_scratch/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
VPARAMS="$(ros2 pkg prefix overlume_ros)/share/overlume_ros/config/default_params.yaml"
RPARAMS="$(ros2 pkg prefix micropilot_rendering_node)/share/micropilot_rendering_node/config/default_params.yaml"

cleanup(){
  for pat in "visualization_[n]ode" "rendering_[n]ode" "bag pla[y]"; do
    for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill "$p" 2>/dev/null; done
  done
  sleep 2
  for pat in "visualization_[n]ode" "rendering_[n]ode" "bag pla[y]"; do
    for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done
  done
}
trap cleanup EXIT
set -m

lc(){ local n=$1 t=$2 i; for i in $(seq 1 20); do out=$(ros2 lifecycle set "$n" "$t" 2>&1); grep -q "Transitioning successful" <<<"$out" && return 0; grep -q "Transitioning failed" <<<"$out" && { echo "$n $t FAILED: $out"; return 1; }; sleep 1; done; echo "$n $t timeout"; return 1; }
hz(){ timeout 14 ros2 topic hz "$1" --window 100 2>/dev/null | grep -oE "average rate: [0-9.]+" | tail -1 | awk '{print $3}'; }
gpu(){ nvidia-smi dmon -s um -c 8 2>/dev/null | awk 'NR>2 && $1!~/#/ {s+=$2; m+=$4; n++} END{ if(n) printf "%.0f %.0f", s/n, m/n; else print "na na"}'; }
cpu(){ top -b -n 3 -d 2 -p "$1" 2>/dev/null | awk -v p="$1" '$1==p {c=$9} END{print c+0}'; }
# Review round 1 (2026-09-11): the plain `rpid=$(pgrep ...)` right after a
# fixed `sleep N` raced the process actually settling -- if `top` sampled
# before/after the pid existed in the process table it silently read 0,
# which is exactly the "pgrep timing artifact" budget_probe.md Results (d)
# named for the old rnode_alone control's CPU% column. Retry until pgrep
# finds a pid AND `top` returns a real (non-empty) sample for it.
wait_pid(){
  local pat=$1 n i
  for i in $(seq 1 20); do
    n=$(pgrep -f "$pat" | head -1)
    if [ -n "$n" ] && [ -n "$(top -b -n 1 -p "$n" 2>/dev/null | awk -v p="$n" '$1==p')" ]; then
      echo "$n"; return 0
    fi
    sleep 0.5
  done
  echo ""; return 1
}

cleanup >/dev/null 2>&1
: > "$OUT/results.txt"

run_co_residence(){
  echo "=== co-residence case: old node (mode 2, real cameras) + new node (bowl_enabled, idling for publish)"

  ros2 run micropilot_rendering_node rendering_node --ros-args --params-file "$RPARAMS" \
    -p initial_mode:=2 -p use_sim_time:=true \
    > "$OUT/rnode.log" 2>&1 &
  rpid=$(wait_pid "lib/micropilot_rendering_node/rendering_[n]ode")
  if ! lc /rendering_node configure || ! lc /rendering_node activate; then cleanup; exit 1; fi

  ros2 run overlume_ros overlume_node --ros-args --params-file "$VPARAMS" \
    -p initial_mode:=2 -p use_sim_time:=true -p quality:=1 -p out_width:=1280 -p out_height:=720 \
    -p profile:=urban -p bowl_enabled:=true \
    > "$OUT/viz.log" 2>&1 &
  vpid=$(wait_pid "lib/overlume_ros/visualization_[n]ode")
  if ! lc /overlume_node configure || ! lc /overlume_node activate; then cleanup; exit 1; fi

  # Fresh single-pass playback (no --loop), best_effort sensor QoS on both
  # recorder and subscriber sides (SensorDataQoS).
  ros2 bag play "$BAG" --clock --rate 1.0 < /dev/null > "$OUT/bag.log" 2>&1 &
  sleep 15  # steady playback + camera info/first frames arrived on both nodes

  python3 "$SAMPLER" 14 > "$OUT/viz.diag.log" 2>&1 &
  diag_pid=$!
  h=$(hz /rendering/image)
  g=$(gpu)
  c_r=$(cpu "$rpid")
  c_v=$(cpu "$vpid")
  wait "$diag_pid"
  diag=$(cat "$OUT/viz.diag.log")
  echo "RESULT co_residence image_hz(old_node_publish)=${h:-0} gpu_sm%_mem%=$g rnode_cpu%=$c_r viz_cpu%=$c_v viz_$diag" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/viz.log" | sed 's/^/  viz log warn+err lines: /'
  grep -ciE "warn|error" "$OUT/rnode.log" | sed 's/^/  rnode log warn+err lines: /'

  cleanup
}

run_rnode_alone(){
  # CONTROL: old node only, no new node process at all -- same bag/procedure
  # as run_co_residence() above, so the two rows are directly comparable and
  # this row's CPU%/image_hz are real readings (wait_pid above), not a
  # throwaway script's pgrep-timing artifact.
  echo "=== rnode_alone case (CONTROL): old node only (mode 2, real cameras), no new node running"

  ros2 run micropilot_rendering_node rendering_node --ros-args --params-file "$RPARAMS" \
    -p initial_mode:=2 -p use_sim_time:=true \
    > "$OUT/rnode_alone.log" 2>&1 &
  rpid=$(wait_pid "lib/micropilot_rendering_node/rendering_[n]ode")
  if ! lc /rendering_node configure || ! lc /rendering_node activate; then cleanup; exit 1; fi

  ros2 bag play "$BAG" --clock --rate 1.0 < /dev/null > "$OUT/bag_alone.log" 2>&1 &
  sleep 15  # same steady-playback warm-up as the co-residence case

  h=$(hz /rendering/image)
  g=$(gpu)
  c_r=$(cpu "$rpid")
  echo "RESULT rnode_alone image_hz(old_node_publish)=${h:-0} gpu_sm%_mem%=$g rnode_cpu%=$c_r" | tee -a "$OUT/results.txt"
  grep -ciE "warn|error" "$OUT/rnode_alone.log" | sed 's/^/  rnode_alone log warn+err lines: /'

  cleanup
}

case "$CASE" in
  co_residence) run_co_residence ;;
  rnode_alone)  run_rnode_alone ;;
  both)         run_co_residence; sleep 3; run_rnode_alone ;;
  *) echo "CASE must be co_residence, rnode_alone or both -- got '$CASE'"; exit 1 ;;
esac

echo DONE
echo "OUT=$OUT"
