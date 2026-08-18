/** @file visualization_node.cpp
 *  @brief ROS2 LifecycleNode wrapping micropilot::visualization (Filament).
 */

#include "micropilot_visualization_node/visualization_node.hpp"

#include <cmath>
#include <cstring>

#include <std_msgs/msg/header.hpp>

namespace micropilot::visualization_app
{

VisualizationNode::VisualizationNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("visualization_node", options)
{
    RCLCPP_INFO(get_logger(), "VisualizationNode constructed — awaiting configure transition.");
}

VisualizationNode::~VisualizationNode() { destroy_renderer_if_any(); }

void VisualizationNode::destroy_renderer_if_any()
{
    if (renderer_ != nullptr)
    {
        mpviz::destroy_renderer(renderer_);
        renderer_ = nullptr;
    }
}

// ── Lifecycle: on_configure ──────────────────────────────────────────────────
VisualizationNode::CallbackReturn VisualizationNode::on_configure(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_configure() called.");

    out_width_ = declare_parameter<int>("out_width", 1280);
    out_height_ = declare_parameter<int>("out_height", 720);
    quality_ = declare_parameter<int>("quality", 2);
    if (quality_ < 0 || quality_ > 2)
    {
        RCLCPP_ERROR(get_logger(), "quality must be 0 (low), 1 (med), or 2 (high), got %d",
                     quality_);
        return CallbackReturn::FAILURE;
    }

    // Global mux mode this node starts in (spec §3.1 "race handling"): both
    // rendering_node and this node default to the same last-configured value
    // so exactly one publisher is active from the first frame.
    initial_mode_ = declare_parameter<int>("initial_mode", 1);
    if (initial_mode_ < 1 || initial_mode_ > 3)
    {
        RCLCPP_ERROR(get_logger(), "initial_mode must be 1, 2, or 3, got %d", initial_mode_);
        return CallbackReturn::FAILURE;
    }
    active_mode_ = initial_mode_;

    // Static hello-frame camera pose (no scene ingestion yet — that's Epic 1+).
    // [eye xyz | target xyz], matching mpviz::CameraPose's own layout — see
    // Task 2's default pose (tests/test_hello_frame.cpp, examples/hello_frame.cpp).
    auto vp = declare_parameter<std::vector<double>>(
        "virtual_pose", {-4.0, 0.0, 3.5, 2.0, 0.0, -0.5});
    if (vp.size() != 6)
    {
        RCLCPP_ERROR(get_logger(), "virtual_pose must be 6 floats [eye xyz | target xyz], got %zu",
                     vp.size());
        return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < 3; ++i) pose_.eye[i] = vp[i];
    for (int i = 0; i < 3; ++i) pose_.target[i] = vp[3 + i];
    // 80°, not the CUDA node's 60° default: at this pose's ~34° downward
    // pitch, 60° puts no sky above the horizon (Task 2 Deviation 3).
    pose_.vfov_deg = declare_parameter<double>("virtual_vfov_deg", 80.0);

    // ── renderer ──────────────────────────────────────────────────────────────
    mpviz::RenderConfig config{};
    config.width = static_cast<uint32_t>(out_width_);
    config.height = static_cast<uint32_t>(out_height_);
    config.quality = static_cast<uint8_t>(quality_);
    renderer_ = mpviz::create_renderer(config);
    if (renderer_ == nullptr)
    {
        RCLCPP_ERROR(get_logger(), "mpviz::create_renderer() failed (no GPU/EGL?)");
        return CallbackReturn::FAILURE;
    }
    frame_buf_.assign(static_cast<size_t>(out_width_) * out_height_ * 3, 0);

    // ── publishers (created here, activated in on_activate) ─────────────────
    // Same global topic names as rendering_node — exactly one node publishes
    // at a time (mode mux, spec §3.1); consumers never re-subscribe.
    pub_image_ = create_publisher<sensor_msgs::msg::Image>("/rendering/image", 1);
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/rendering/camera_info", 1);

    // ── mode mux subscription (global, not "~/...") ──────────────────────────
    set_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/rendering/set_mode", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            if (msg->data != 1 && msg->data != 2 && msg->data != 3)
            {
                RCLCPP_WARN(get_logger(), "set_mode: expected 1|2|3, got %d", msg->data);
                return;
            }
            active_mode_ = msg->data;
            RCLCPP_INFO(get_logger(), "visualization_node: mode -> %d", active_mode_);
        });

    RCLCPP_INFO(get_logger(), "on_configure() succeeded. out=%dx%d quality=%d initial_mode=%d",
                out_width_, out_height_, quality_, initial_mode_);
    return CallbackReturn::SUCCESS;
}

// ── Lifecycle: on_activate ───────────────────────────────────────────────────
VisualizationNode::CallbackReturn VisualizationNode::on_activate(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_activate() called.");
    pub_image_->on_activate();
    pub_info_->on_activate();

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(33ms, [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(), "on_activate() succeeded.");
    return CallbackReturn::SUCCESS;
}

// ── Timer callback ───────────────────────────────────────────────────────────
void VisualizationNode::timer_callback()
{
    // Ingest adapters (Epic 2+) would update cheap CPU-side scene state here
    // regardless of mode; Task 3 has none yet. Render/readback/publish only
    // while this node is the active mux output (spec §3.1) — costs ~zero GPU
    // otherwise.
    if (active_mode_ != 3) return;

    mpviz::FrameView view{frame_buf_.data(), static_cast<uint32_t>(out_width_),
                          static_cast<uint32_t>(out_height_)};
    if (!mpviz::render_frame(renderer_, pose_, view))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "render_frame() failed");
        return;
    }

    auto stamp = now();

    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = stamp;
    img_msg.header.frame_id = "visualization_virtual_cam";
    img_msg.width = static_cast<uint32_t>(out_width_);
    img_msg.height = static_cast<uint32_t>(out_height_);
    img_msg.encoding = "rgb8";
    img_msg.is_bigendian = 0;
    img_msg.step = static_cast<uint32_t>(out_width_) * 3;
    img_msg.data.assign(frame_buf_.begin(), frame_buf_.end());
    pub_image_->publish(img_msg);

    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = img_msg.header;
    info_msg.width = img_msg.width;
    info_msg.height = img_msg.height;
    info_msg.distortion_model = "plumb_bob";
    double fy = (out_height_ / 2.0) / std::tan(pose_.vfov_deg * 0.5 * M_PI / 180.0);
    info_msg.k[0] = fy;               info_msg.k[1] = 0;  info_msg.k[2] = out_width_ / 2.0;
    info_msg.k[3] = 0;                info_msg.k[4] = fy; info_msg.k[5] = out_height_ / 2.0;
    info_msg.k[6] = 0;                info_msg.k[7] = 0;  info_msg.k[8] = 1;
    pub_info_->publish(info_msg);
}

// ── Lifecycle: teardown ──────────────────────────────────────────────────────
void VisualizationNode::teardown_active() { timer_.reset(); }

VisualizationNode::CallbackReturn VisualizationNode::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_deactivate() called.");
    teardown_active();
    pub_image_->on_deactivate();
    pub_info_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

VisualizationNode::CallbackReturn VisualizationNode::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_cleanup() called.");
    teardown_active();
    destroy_renderer_if_any();
    frame_buf_.clear();
    pub_image_.reset();
    pub_info_.reset();
    set_mode_sub_.reset();
    return CallbackReturn::SUCCESS;
}

VisualizationNode::CallbackReturn VisualizationNode::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() called.");
    teardown_active();
    destroy_renderer_if_any();
    set_mode_sub_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::visualization_app
