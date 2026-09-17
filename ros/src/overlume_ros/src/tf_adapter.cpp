// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/tf_adapter.hpp"

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

namespace overlume::ros {

TfAdapter::TfAdapter(tf2_ros::Buffer& buffer, std::string map_frame, std::string base_frame,
                     double alpha, bool flatten_z)
    : buffer_(buffer),
      map_frame_(std::move(map_frame)),
      base_frame_(std::move(base_frame)),
      alpha_(alpha),
      flatten_z_(flatten_z) {}

overlume::EgoState TfAdapter::update() {
    overlume::EgoState ego{};  // valid = 0 unless a lookup below succeeds

    geometry_msgs::msg::TransformStamped t;
    try {
        t = buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
        // No TF yet (LookupException) or can't extrapolate
        // (ExtrapolationException) -- non-fatal, same "no data" philosophy
        // as set_ego_model's "bad data" fallback: hide the ego, don't crash
        // and don't park a clay box at the origin (scene.h's EgoState::valid
        // comment).
        have_prev_ = false;  // next successful lookup shouldn't finite-diff across the gap
        return ego;
    }

    const overlume::Vec3 pos{t.transform.translation.x, t.transform.translation.y,
                             t.transform.translation.z};
    const auto& q = t.transform.rotation;
    const double heading =
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const rclcpp::Time stamp(t.header.stamp, RCL_ROS_TIME);

    if (have_prev_) {
        const double dt = (stamp - prev_stamp_).seconds();
        if (dt > 1e-6) {
            const double dx = pos.x - prev_pos_.x;
            const double dy = pos.y - prev_pos_.y;
            const double dz = pos.z - prev_pos_.z;
            const double raw = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
            smoothed_speed_ += alpha_ * (raw - smoothed_speed_);
        }
        // dt <= 0 (duplicate/out-of-order stamp): keep last smoothed_speed_,
        // don't divide by ~0.
    } else {
        smoothed_speed_ = 0.0;  // first sample: no prior sample to diff against
        have_prev_ = true;
    }

    prev_pos_ = pos;
    prev_stamp_ = stamp;

    ego.position = pos;
    // flatten_z: 2D HD-map plane (see frame_transform.hpp) -- live TF
    // carries real altitude and the ego would float above every flattened
    // layer otherwise.
    if (flatten_z_) ego.position.z = 0.0;
    ego.heading_rad = heading;
    // Spec §7: prefer /robot/feedback/robot_speed_mps over the TF-diff/EMA
    // once the node has forwarded at least one sample (set_robot_speed_mps).
    ego.speed_mps = topic_speed_mps_.has_value() ? *topic_speed_mps_ : smoothed_speed_;
    ego.valid = 1;
    return ego;
}

}  // namespace overlume::ros
