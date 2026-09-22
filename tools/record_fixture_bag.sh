#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

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

echo "Recording to $OUT:"
printf '  %s\n' "${BASELINE[@]}" "$@"
exec ros2 bag record -o "$OUT" "${BASELINE[@]}" "$@"
