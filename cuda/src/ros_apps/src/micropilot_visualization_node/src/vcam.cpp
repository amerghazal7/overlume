/** @file vcam.cpp
 *  @brief Virtual-camera presets + eased tween switching (plan Task 5 / VM-013).
 *
 *  Bodies moved verbatim from visualization_node.cpp — ported originally from
 *  micropilot_rendering_node/src/rendering_node.cpp's smoothstep()/
 *  advance_tween() (float precision preserved — see LookPoint's doc comment
 *  in vcam.hpp for why). This extraction changes zero math (plan Task 5
 *  reality check + Step 2: "a byte-diff of the tween math against the
 *  pre-refactor version should show zero changes").
 */

#include "micropilot_visualization_node/vcam.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace micropilot::visualization_app
{

namespace
{
float smoothstep(float s)
{
    s = std::min(1.0f, std::max(0.0f, s));
    return s * s * (3.0f - 2.0f * s);
}
}  // namespace

Vcam::Vcam(rclcpp_lifecycle::LifecycleNode* node, const mpviz::CameraPose& seed_pose)
    : pose_(seed_pose), logger_(node->get_logger()), clock_(node->get_clock())
{
    // ── virtual-camera presets (plan Task 5) ─────────────────────────────────
    // Preset 1 ("config") is the just-declared virtual_pose, expressed
    // directly as a look-point (no R/t derivation needed here — unlike
    // rendering_node's CUDA camera, mpviz::CameraPose already IS eye/target).
    for (int i = 0; i < 3; ++i) presets_[0].eye[i] = static_cast<float>(pose_.eye[i]);
    for (int i = 0; i < 3; ++i) presets_[0].target[i] = static_cast<float>(pose_.target[i]);
    // Presets 2-5: identical formulas/constants to rendering_node's table
    // (spec §6 — same framing in both worlds).
    presets_[1] = LookPoint{
        {-presets_[0].eye[0], -presets_[0].eye[1], presets_[0].eye[2]},
        {-presets_[0].target[0], -presets_[0].target[1], presets_[0].target[2]}};
    presets_[2] = LookPoint{{0.0f, 4.0f, 2.5f}, {0.0f, 0.0f, 0.5f}};    // left_side
    presets_[3] = LookPoint{{0.0f, -4.0f, 2.5f}, {0.0f, 0.0f, 0.5f}};   // right_side
    presets_[4] = LookPoint{{0.0f, 0.0f, 8.0f}, {0.0f, 0.001f, 0.0f}};  // top_down
    cur_ = src_ = dst_ = presets_[0];
    tween_t_ = 1.0;  // start settled on the config preset
    active_preset_ = 1;

    // ── vcam control surface (plan Task 5 / spec §6) ─────────────────────────
    // Same message/service contracts as rendering_node's, under this node's
    // own namespace — the WS bridge fans commands out to both.
    set_vcam_srv_ = node->create_service<SetVirtualCam>(
        "~/set_virtual_cam",
        std::bind(&Vcam::on_set_virtual_cam, this, std::placeholders::_1, std::placeholders::_2));
    set_look_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/set_look", 10, std::bind(&Vcam::on_set_look, this, std::placeholders::_1));
}

void Vcam::advance_tween()
{
    if (tween_t_ < 1.0)
    {
        // Timer fires at 33 ms; ~0.5 s transition -> step 0.033/0.5 per tick.
        tween_t_ = std::min(1.0, tween_t_ + 0.033 / 0.5);
        float w = smoothstep(static_cast<float>(tween_t_));
        for (int i = 0; i < 3; ++i)
        {
            cur_.eye[i] = src_.eye[i] + (dst_.eye[i] - src_.eye[i]) * w;
            cur_.target[i] = src_.target[i] + (dst_.target[i] - src_.target[i]) * w;
        }
    }
    for (int i = 0; i < 3; ++i) pose_.eye[i] = static_cast<double>(cur_.eye[i]);
    for (int i = 0; i < 3; ++i) pose_.target[i] = static_cast<double>(cur_.target[i]);
}

void Vcam::on_set_virtual_cam(const std::shared_ptr<SetVirtualCam::Request> req,
                               std::shared_ptr<SetVirtualCam::Response> res)
{
    // ponytail: no lock — single-threaded executor (rclcpp::spin in main.cpp),
    // so this callback and timer_callback() never overlap.
    const int p = req->preset;
    if (p < 1 || p > static_cast<int>(presets_.size()))
    {
        res->success = false;
        res->active = "invalid preset (expected 1.." + std::to_string(presets_.size()) + ")";
        RCLCPP_WARN(logger_, "set_virtual_cam: rejected preset %d", p);
        return;
    }
    src_ = cur_;
    dst_ = presets_[p - 1];
    tween_t_ = 0.0;  // begin the eased transition
    active_preset_ = p;
    res->success = true;
    res->active = kPresetNames[p - 1];
    RCLCPP_INFO(logger_, "set_virtual_cam: -> preset %d (%s)", p, kPresetNames[p - 1]);
}

void Vcam::on_set_look(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                              "set_look expects 6 floats [eye xyz | target xyz], got %zu",
                              msg->data.size());
        return;
    }
    LookPoint lp;
    for (int i = 0; i < 3; ++i) lp.eye[i] = static_cast<float>(msg->data[i]);
    for (int i = 0; i < 3; ++i) lp.target[i] = static_cast<float>(msg->data[3 + i]);
    cur_ = src_ = dst_ = lp;
    tween_t_ = 1.0;  // cancel any in-flight preset tween
    for (int i = 0; i < 3; ++i) pose_.eye[i] = static_cast<double>(cur_.eye[i]);
    for (int i = 0; i < 3; ++i) pose_.target[i] = static_cast<double>(cur_.target[i]);
    active_preset_ = 0;  // free look
}

}  // namespace micropilot::visualization_app
