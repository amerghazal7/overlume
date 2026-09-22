// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/frame_transform.hpp"

#include <rclcpp/time.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace overlume::ros {

bool FrameTransformer::lookup(const std_msgs::msg::Header& header, tf2::Transform& out) const {
    if (header.frame_id.empty() || header.frame_id == target_frame_) {
        out.setIdentity();
        return true;
    }

    geometry_msgs::msg::TransformStamped msg;
    try {
        msg = buffer_.lookupTransform(target_frame_, header.frame_id,
                                      tf2_ros::fromRclcpp(rclcpp::Time(header.stamp)));
    } catch (const tf2::TransformException&) {
        try {
            msg = buffer_.lookupTransform(target_frame_, header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    tf2::fromMsg(msg.transform, out);
    return true;
}

}  // namespace overlume::ros
