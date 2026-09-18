// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file fixture_msgs.hpp
 *  @brief Test-only helpers, shared by every node gtest target. The inverse
 *  of scripts/bag_to_fixture.py: yaml-cpp -> ROS message, covering only the
 *  fields the adapters read.
 *
 *  Fixture filenames are relative to OVERLUME_NODE_FIXTURES_DIR
 *  (test/fixtures/), a macro every CMake gtest target that compiles
 *  fixture_msgs.cpp must define -- see CMakeLists.txt's
 *  `overlume_node_test_paths` INTERFACE target.
 */

#include <string>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "overlume_ros/adapters/dynamic_objects.hpp"
#include "overlume_ros/profile.hpp"

namespace overlume::ros::testing {

// Loads `<OVERLUME_NODE_FIXTURES_DIR>/<fixture_name>` (a
// rosidl_runtime_py.message_to_yaml-shaped YAML file, or a hand-edited
// variant of one) as a visualization_msgs::msg::MarkerArray / nav_msgs::
// msg::Path / nav_msgs::msg::OccupancyGrid. Throws std::runtime_error if
// the file can't be opened or parsed -- these are test fixtures, a missing
// one is a broken test, not a runtime condition to handle gracefully.
visualization_msgs::msg::MarkerArray load_marker_array(const std::string& fixture_name);
nav_msgs::msg::Path load_path(const std::string& fixture_name);
nav_msgs::msg::OccupancyGrid load_occupancy_grid(const std::string& fixture_name);
// map_msgs/OccupancyGridUpdate loader for OgmAdapter's second ingest
// overload. Synthetic, hand-authored YAML -- no OccupancyGridUpdate topic
// exists in the recorded bag.
map_msgs::msg::OccupancyGridUpdate load_occupancy_grid_update(const std::string& fixture_name);

// Loads the shipped urban_profile.yaml / sim_profile.yaml (from
// TEST_CONFIG_DIR) and returns a COPY of the row whose topic matches --
// so an adapter test constructs against the row that actually ships,
// rather than a hand-written one that can silently drift from it. Aborts
// the test (ASSERT via a thrown exception) if the shipped profile fails
// to load or the topic has no row.
ProfileRow urban_row(const std::string& topic);
ProfileRow sim_row(const std::string& topic);

// Loads the shipped config/class_inference.yaml (from TEST_CONFIG_DIR) --
// same "test against what actually ships" reasoning as urban_row/sim_row.
// Aborts the test if the shipped table fails to load.
ClassInferenceTable inference_table();

}  // namespace overlume::ros::testing
