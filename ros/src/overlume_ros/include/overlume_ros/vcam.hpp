// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "overlume/api.h"

#include "overlume_ros/srv/set_virtual_cam.hpp"

namespace overlume::ros {

struct LookPoint {
    float eye[3];
    float target[3];
};

class Vcam {
public:
    using SetVirtualCam = overlume_ros::srv::SetVirtualCam;

    Vcam(rclcpp_lifecycle::LifecycleNode* node, const overlume::CameraPose& seed_pose);

    void advance_tween();

    const overlume::CameraPose& pose() const { return pose_; }
    int active_preset() const { return active_preset_; }

private:
    void on_set_virtual_cam(const std::shared_ptr<SetVirtualCam::Request> req,
                            std::shared_ptr<SetVirtualCam::Response> res);
    void on_set_look(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    std::array<LookPoint, 5> presets_{};
    static constexpr std::array<const char*, 5> kPresetNames{"config", "reverse_follow",
                                                             "left_side", "right_side", "top_down"};
    LookPoint cur_{}, src_{}, dst_{};
    double tween_t_{1.0};
    int active_preset_{1};

    overlume::CameraPose pose_{};

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Service<SetVirtualCam>::SharedPtr set_vcam_srv_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_look_sub_;
};

}
