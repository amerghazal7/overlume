#pragma once
/** @file visualization_node.hpp
 *  @brief LifecycleNode wrapping micropilot::visualization (Filament headless
 *  renderer) — Visual Mode ("mode 3") on the shared /rendering/image mux.
 *
 *  Epic 0 Task 3 (docs/superpowers/plans/2026-08-18-visual-mode.md): a
 *  skeleton that streams the library's static hello-frame scene (ground +
 *  grid + cube, from Task 2) gated by the global `/rendering/set_mode`
 *  topic. Scene ingestion from autonomy topics arrives in later epics —
 *  this node only proves the mux + vcam-shaped output contract.
 */

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

#include "visual_renderer/api.h"

namespace micropilot::visualization_app
{

class VisualizationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit VisualizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VisualizationNode() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    void timer_callback();
    void teardown_active();
    void destroy_renderer_if_any();

    // ── configuration (declared in on_configure) ─────────────────────────────
    int out_width_{1280};
    int out_height_{720};
    int quality_{2};       // 0=low, 1=med, 2=high (mpviz::RenderConfig::quality)
    int initial_mode_{1};  // last-configured global mux mode; matches rendering_node's default
    mpviz::CameraPose pose_{};  // static hello-frame pose (virtual_pose + virtual_vfov_deg params)

    // ── mode mux ──────────────────────────────────────────────────────────────
    // Global (not "~/...") — both this node and rendering_node subscribe the
    // same topic (spec §3.1). Renders+publishes only while == 3.
    int active_mode_{1};
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_mode_sub_;

    // ── renderer + preallocated output buffer ────────────────────────────────
    mpviz::VisualRenderer* renderer_{nullptr};
    std::vector<uint8_t> frame_buf_;  // out_width_*out_height_*3, rgb8

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace micropilot::visualization_app
