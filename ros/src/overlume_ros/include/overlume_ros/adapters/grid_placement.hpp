// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include "overlume_ros/frame_transform.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

// ponytail: kNonFinite is a NaN-only check on the transformed origin, matching
// the pre-refactor OgmAdapter; switch to !std::isfinite if Inf origins are ever seen.
enum class PlacementResult { kOk, kNoTf, kNonFinite };

struct GridPlacement {
    overlume::Vec3 origin;
    double yaw_rad;
};

// Places an OccupancyGrid-style grid (header + info.origin pose) into the map
// frame. frame_override, when non-empty, replaces header.frame_id before the
// TF lookup. origin.z is zeroed when tf.flatten_z(). `out` is written only on
// kOk.
PlacementResult place_grid(const std_msgs::msg::Header& header,
                           const geometry_msgs::msg::Pose& grid_origin,
                           const std::string& frame_override, const FrameTransformer& tf,
                           GridPlacement& out);

}
