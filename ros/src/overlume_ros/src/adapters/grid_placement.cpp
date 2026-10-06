// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/grid_placement.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

bool HasNan(const tf2::Vector3& v) {
    return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z());
}

}

PlacementResult place_grid(const std_msgs::msg::Header& header_in,
                           const geometry_msgs::msg::Pose& grid_origin,
                           const std::string& frame_override,
                           const FrameTransformer& tf, GridPlacement& out) {
    tf2::Transform xform;
    std_msgs::msg::Header header = header_in;
    if (!frame_override.empty()) header.frame_id = frame_override;
    if (!tf.lookup(header, xform)) return PlacementResult::kNoTf;

    const auto& p = grid_origin.position;
    const tf2::Vector3 originTf = xform * tf2::Vector3(p.x, p.y, p.z);
    if (HasNan(originTf)) return PlacementResult::kNonFinite;

    out.origin = {originTf.x(), originTf.y(), tf.flatten_z() ? 0.0 : originTf.z()};
    const auto& q = grid_origin.orientation;
    const tf2::Quaternion gridInMap =
        xform.getRotation() * tf2::Quaternion(q.x, q.y, q.z, q.w).normalized();
    const tf2::Vector3 xAxis = tf2::Transform(gridInMap) * tf2::Vector3(1.0, 0.0, 0.0);
    out.yaw_rad = std::atan2(xAxis.y(), xAxis.x());
    return PlacementResult::kOk;
}

}
