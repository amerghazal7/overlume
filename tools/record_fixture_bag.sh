#!/usr/bin/env bash
# record_fixture_bag.sh — record a visual-mode fixture bag from the LIVE
# stack (VM-077, user directive 2026-09-09: new stack version, new topics;
# the recording replaces epic2_fixtures_full as validate_visual_mode.sh's
# default bag once its health gate passes).
#
# Usage: tools/record_fixture_bag.sh OUT_DIR [EXTRA_TOPIC ...]
#   OUT_DIR       bag output directory (must not exist; rosbag2 creates it)
#   EXTRA_TOPIC   new-stack topics to record beyond the baseline list below
#
# Baseline = every topic the epic2_fixtures_full bag carried (the profiles'
# rows all draw from these), so the new bag is a strict superset and the
# validate script keeps working unmodified until its default path is swapped.
set -euo pipefail

OUT=${1:?usage: record_fixture_bag.sh OUT_DIR [EXTRA_TOPIC ...]}
shift || true

BASELINE=(
  /tf /tf_static
  /hd_map_local_elements /hd_map_global_elements /road_markers
  /sim/hd_map/markers /local_hd_map_binary /global_hd_map_binary
  /perception/dynamic_objects_list /sim/ground_truth/boxes
  /behavior_path_planner/output_path_visualization
  /behavior_path_planner/reference_trajectory_visualization
  /local_vel_path
  /robot/feedback/robot_speed_mps /sim/feedback/gps
)

# ponytail: explicit include list, not `-a` — the new stack may publish
# camera/pointcloud topics that would balloon the bag; add them explicitly
# as EXTRA_TOPICs if a fixture ever needs them.
echo "Recording to $OUT:"
printf '  %s\n' "${BASELINE[@]}" "$@"
exec ros2 bag record -o "$OUT" "${BASELINE[@]}" "$@"
