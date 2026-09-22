// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

namespace overlume::ros {

class FrameTransformer {
public:
    explicit FrameTransformer(const tf2_ros::Buffer& buffer, std::string target_frame = "map",
                              bool flatten_z = true)
        : buffer_(buffer), target_frame_(std::move(target_frame)), flatten_z_(flatten_z) {}

    bool flatten_z() const { return flatten_z_; }

    bool lookup(const std_msgs::msg::Header& header, tf2::Transform& out) const;

private:
    const tf2_ros::Buffer& buffer_;
    std::string target_frame_;
    bool flatten_z_ = true;
};

}
