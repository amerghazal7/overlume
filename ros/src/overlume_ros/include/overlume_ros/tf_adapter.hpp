// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <optional>
#include <string>

#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>

#include "overlume/scene.h"

namespace overlume::ros {

class TfAdapter {
public:
    explicit TfAdapter(tf2_ros::Buffer& buffer, std::string map_frame = "map",
                       std::string base_frame = "base_link", double alpha = 0.2,
                       bool flatten_z = true);

    overlume::EgoState update();

    void set_robot_speed_mps(double mps) { topic_speed_mps_ = mps; }

private:
    tf2_ros::Buffer& buffer_;
    std::string map_frame_;
    std::string base_frame_;
    double alpha_;
    bool flatten_z_ = true;

    bool have_prev_{false};
    overlume::Vec3 prev_pos_{};
    rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
    double smoothed_speed_{0.0};

    std::optional<double> topic_speed_mps_;
};

}
