// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file vcam.hpp
 *  @brief Virtual-camera presets + eased tween switching, extracted verbatim
 *  from overlume_node.{hpp,cpp} (plan Task 5 / VM-013).
 *
 *  Pure mechanical extraction — no behavior change (see the plan's Task 5
 *  reality-check paragraph). Everything here operates in the same
 *  EGO-RELATIVE OFFSET frame it always has; the ego-anchored composition
 *  (ego_anchor.hpp) stays in overlume_node.cpp's timer_callback, at the
 *  render-pose boundary, since Vcam only knows the offset frame (plan Task 5
 *  Scope-addition block).
 */

#include <array>
#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "overlume/api.h"

// SetVirtualCam is OWNED by this package since the VM-095 cutover (Step 4:
// the .srv moved in with rosidl generation; the qualified type name changed
// with it). The wire layout is byte-identical to the retired node's.
#include "overlume_ros/srv/set_virtual_cam.hpp"

namespace overlume::ros {

/// A virtual-camera placement as look-points in the rig frame (x-fwd, y-left,
/// z-up) — mirrors micropilot_rendering_node's LookPoint exactly (plan Task 5:
/// "port the tween math verbatim"). float precision is kept (not double, like
/// overlume::CameraPose) so that presets shared numerically with the CUDA node
/// (spec §6: "presets/orbits produce the same framing in both worlds") tween
/// to bit-identical results — the contract test's 1e-9 tolerance would not be
/// safely met if the two nodes rounded the same constants differently.
struct LookPoint {
    float eye[3];
    float target[3];
};

/// Owns the vcam preset table, the eased src_->dst_ tween, and the
/// ~/set_virtual_cam service + ~/set_look subscription that drive it.
/// Extracted verbatim from OverlumeNode (plan Task 5 / VM-013) — the
/// only new thing here is the class boundary itself.
class Vcam {
public:
    using SetVirtualCam = overlume_ros::srv::SetVirtualCam;

    /// node: owning LifecycleNode, used only to create the service/subscription
    /// against (never stored beyond that — no timer/lifecycle callbacks live
    /// here). seed_pose: the virtual_pose/virtual_vfov_deg-derived starting
    /// pose (identical formula to today's inline on_configure code) — seeds
    /// presets_[0] ("config") and this object's own pose_ (including
    /// vfov_deg, which the tween never touches).
    Vcam(rclcpp_lifecycle::LifecycleNode* node, const overlume::CameraPose& seed_pose);

    /// Advance the smoothstep tween one timer tick and update cur_ + pose_
    /// (no-op once settled at tween_t_ == 1.0, but pose_ is still refreshed
    /// from cur_ every call — identical to today's inline behavior).
    void advance_tween();

    const overlume::CameraPose& pose() const { return pose_; }
    int active_preset() const { return active_preset_; }

private:
    void on_set_virtual_cam(const std::shared_ptr<SetVirtualCam::Request> req,
                            std::shared_ptr<SetVirtualCam::Response> res);
    /// Handle ~/set_look: 6 floats [eye xyz | target xyz] applied immediately
    /// (no tween) — the generic runtime pose input used for free-look orbiting.
    void on_set_look(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Preset table: [0]=config [1]=reverse_follow [2]=left_side [3]=right_side
    // [4]=top_down. Index i is preset (i+1) in the service request. Identical
    // names/formulas to micropilot_rendering_node so the two nodes' presets
    // frame the world the same way (spec §6).
    std::array<LookPoint, 5> presets_{};
    static constexpr std::array<const char*, 5> kPresetNames{"config", "reverse_follow",
                                                             "left_side", "right_side", "top_down"};
    LookPoint cur_{}, src_{}, dst_{};
    double tween_t_{1.0};   // [0,1]; 1.0 = settled on dst_. Eased per timer tick.
    int active_preset_{1};  // 1-5 = preset in service numbering; 0 = free look

    overlume::CameraPose pose_{};  // current (possibly tweening) pose

    // Captured from the owning node at construction (never re-queried) so
    // on_set_virtual_cam/on_set_look log exactly as they did inline —
    // RCLCPP_WARN_THROTTLE in particular needs the SAME clock instance
    // across calls to throttle correctly, not a fresh one per call.
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Service<SetVirtualCam>::SharedPtr set_vcam_srv_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_look_sub_;
};

}  // namespace overlume::ros
