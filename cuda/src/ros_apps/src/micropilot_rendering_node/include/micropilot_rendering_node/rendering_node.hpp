#pragma once
/** @file rendering_node.hpp
 *  @brief LifecycleNode that wraps the micropilot_rendering CUDA reprojector.
 *
 *  Production note: camera extrinsics are read from a ROS parameter
 *  (`camera_extrinsics`) for determinism / smoke-test convenience.  A tf2
 *  lookup is the natural production alternative: wire `tf2_ros::Buffer` in
 *  `on_configure`, call `lookupTransform` in the timer callback, and populate
 *  `CameraParams.R/t` from `geometry_msgs::msg::TransformStamped`.
 */

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rendering_reprojector/reprojector.hpp"
#include "rendering_reprojector/types.hpp"

namespace micropilot::rendering_app
{

class RenderingNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit RenderingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RenderingNode() override = default;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    void timer_callback();
    void teardown_active();

    // ── configuration ────────────────────────────────────────────────────────
    int n_cameras_{4};
    int out_width_{640};
    int out_height_{480};
    // Render only when all cameras have a new frame whose stamps fall within this
    // window (seconds). Prevents stitching temporally-misaligned async frames.
    double max_sync_latency_{0.12};
    // Fill color for genuinely-unseen pixels (above the bowl rim) — sky, not black,
    // so the teleop driving view shows a natural horizon. RGB in [0,1].
    float sky_color_[3]{0.53f, 0.70f, 0.92f};
    micropilot::rendering::BowlParams bowl_{6.0f, 0.08f, 20.0f};
    micropilot::rendering::CameraParams vcam_{};

    std::vector<micropilot::rendering::CameraParams> cam_params_;

    // ── runtime ──────────────────────────────────────────────────────────────
    std::unique_ptr<micropilot::rendering::Reprojector> reprojector_;

    struct PerCamera
    {
        std::optional<cv::Mat> image;        // float32 NHWC staging
        bool info_ready{false};
        rclcpp::Time stamp;                  // header stamp of the staged image
        bool have_new{false};                // a new frame arrived since last render
        std::unique_ptr<std::mutex> mtx;     // unique_ptr keeps vector movable

        PerCamera() : mtx(std::make_unique<std::mutex>()) {}
    };
    std::vector<PerCamera> per_cam_;

    // subscriptions (active only while ACTIVE)
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> img_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::TimerBase::SharedPtr timer_;

    // track which cameras had an image update since last upload
    std::vector<bool> img_dirty_;

    // explicit per-camera topic names (size n_cameras_); empty => default
    // "/camera/camN/image_raw" + "/camera/camN/camera_info" pattern.
    std::vector<std::string> image_topics_;
    std::vector<std::string> info_topics_;
};

}  // namespace micropilot::rendering_app
