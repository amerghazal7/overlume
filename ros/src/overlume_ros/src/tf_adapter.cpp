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
    overlume::EgoState ego{};

    geometry_msgs::msg::TransformStamped t;
    try {
        t = buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
        have_prev_ = false;
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
    } else {
        smoothed_speed_ = 0.0;
        have_prev_ = true;
    }

    prev_pos_ = pos;
    prev_stamp_ = stamp;

    ego.position = pos;
    if (flatten_z_) ego.position.z = 0.0;
    ego.heading_rad = heading;
    ego.speed_mps = topic_speed_mps_.has_value() ? *topic_speed_mps_ : smoothed_speed_;
    ego.valid = 1;
    return ego;
}

}  // namespace overlume::ros
