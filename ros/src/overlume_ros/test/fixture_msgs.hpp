// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <string>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapters/dynamic_objects.hpp"
#include "overlume_ros/profile.hpp"

namespace overlume::ros::testing {

visualization_msgs::msg::MarkerArray load_marker_array(const std::string& fixture_name);
nav_msgs::msg::Path load_path(const std::string& fixture_name);
nav_msgs::msg::OccupancyGrid load_occupancy_grid(const std::string& fixture_name);
map_msgs::msg::OccupancyGridUpdate load_occupancy_grid_update(const std::string& fixture_name);

ProfileRow urban_row(const std::string& topic);
ProfileRow sim_row(const std::string& topic);

ClassInferenceTable inference_table();

}
