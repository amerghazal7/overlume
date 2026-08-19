/** @file visualization_node.cpp
 *  @brief ROS2 LifecycleNode wrapping micropilot::visualization (Filament).
 */

#include "micropilot_visualization_node/visualization_node.hpp"

#include <cmath>
#include <cstring>

#include <std_msgs/msg/header.hpp>

namespace micropilot::visualization_app
{

// ── Virtual-camera presets / eased switching (plan Task 5) ──────────────────
// Ported verbatim from micropilot_rendering_node/src/rendering_node.cpp's
// smoothstep()/advance_tween() (float precision preserved — see LookPoint's
// doc comment in the header for why).
namespace
{
float smoothstep(float s)
{
    s = std::min(1.0f, std::max(0.0f, s));
    return s * s * (3.0f - 2.0f * s);
}
}  // namespace

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
    quality_ = declare_parameter<int>("quality", 1);
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
    pub_vcam_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/vcam_state", 1);

    // ── vcam control surface (plan Task 5 / spec §6) ─────────────────────────
    // Same message/service contracts as rendering_node's, under this node's
    // own namespace — the WS bridge fans commands out to both.
    set_vcam_srv_ = create_service<SetVirtualCam>(
        "~/set_virtual_cam",
        std::bind(&VisualizationNode::on_set_virtual_cam, this, std::placeholders::_1,
                  std::placeholders::_2));
    set_look_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/set_look", 10,
        std::bind(&VisualizationNode::on_set_look, this, std::placeholders::_1));

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

    // ── theme control (Epic 1 Task 3 / VM-014) ───────────────────────────────
    theme_sub_ = create_subscription<std_msgs::msg::String>(
        "~/set_theme", 10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            if (!mpviz::set_theme(renderer_, msg->data.c_str(), sim_clock_sec_, 0.0))
            {
                RCLCPP_WARN(get_logger(), "set_theme: unknown theme '%s'", msg->data.c_str());
            }
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
    pub_vcam_state_->on_activate();

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(33ms, [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(), "on_activate() succeeded.");
    return CallbackReturn::SUCCESS;
}

// ── Timer callback ───────────────────────────────────────────────────────────
void VisualizationNode::timer_callback()
{
    // Ease the virtual camera toward the selected preset (no-op once settled)
    // and publish vcam telemetry BEFORE the mode gate below, mirroring
    // rendering_node: external UIs keep receiving pose updates (and can
    // pre-orbit) even while this node isn't the active mux output.
    advance_tween();

    std_msgs::msg::Float64MultiArray state;
    state.data = {cur_.eye[0],    cur_.eye[1],    cur_.eye[2],
                  cur_.target[0], cur_.target[1], cur_.target[2],
                  static_cast<double>(active_preset_),
                  static_cast<double>(active_mode_)};
    pub_vcam_state_->publish(state);

    // Epic 1 Task 3 (VM-014): the call this whole task depends on. EVERY
    // tick, regardless of mode (ingest continues regardless of mode — same
    // philosophy as the mux above), advance the node's own monotonic clock
    // and push it into the renderer via set_scene() -- BEFORE the mode gate
    // below that decides whether this tick actually renders/publishes an
    // image. Without this call, SceneBuffer::active().sim_time_sec never
    // advances, render_frame()'s theme-transition clock is permanently
    // stuck at t=0, and a ~/set_theme request would never visibly finish
    // outside a unit test that drives set_scene()/render_frame() directly.
    // Task 4 fills in scene.ego from the TF adapter here too; nothing else
    // is populated until Epic 2.
    sim_clock_sec_ += kTimerPeriodSec;
    mpviz::SceneGraph scene{};
    scene.sim_time_sec = sim_clock_sec_;
    mpviz::set_scene(renderer_, scene);

    // Render/readback/publish only while this node is the active mux output
    // (spec §3.1) — costs ~zero GPU otherwise.
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

// ── Virtual-camera presets / eased switching (plan Task 5) ──────────────────
// Ported verbatim from rendering_node.cpp's advance_tween()/on_set_virtual_cam()/
// on_set_look() — the only difference is applying the result directly as
// mpviz::CameraPose eye/target instead of rebuilding an R/t rotation (Filament's
// camera already takes eye/target, unlike the CUDA reprojector's).
void VisualizationNode::advance_tween()
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

void VisualizationNode::on_set_virtual_cam(const std::shared_ptr<SetVirtualCam::Request> req,
                                           std::shared_ptr<SetVirtualCam::Response> res)
{
    // ponytail: no lock — single-threaded executor (rclcpp::spin in main.cpp),
    // so this callback and timer_callback() never overlap.
    const int p = req->preset;
    if (p < 1 || p > static_cast<int>(presets_.size()))
    {
        res->success = false;
        res->active = "invalid preset (expected 1.." + std::to_string(presets_.size()) + ")";
        RCLCPP_WARN(get_logger(), "set_virtual_cam: rejected preset %d", p);
        return;
    }
    src_ = cur_;
    dst_ = presets_[p - 1];
    tween_t_ = 0.0;  // begin the eased transition
    active_preset_ = p;
    res->success = true;
    res->active = kPresetNames[p - 1];
    RCLCPP_INFO(get_logger(), "set_virtual_cam: -> preset %d (%s)", p, kPresetNames[p - 1]);
}

void VisualizationNode::on_set_look(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
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

// ── Lifecycle: teardown ──────────────────────────────────────────────────────
void VisualizationNode::teardown_active() { timer_.reset(); }

VisualizationNode::CallbackReturn VisualizationNode::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_deactivate() called.");
    teardown_active();
    pub_image_->on_deactivate();
    pub_info_->on_deactivate();
    pub_vcam_state_->on_deactivate();
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
    pub_vcam_state_.reset();
    set_mode_sub_.reset();
    set_vcam_srv_.reset();
    set_look_sub_.reset();
    theme_sub_.reset();
    return CallbackReturn::SUCCESS;
}

VisualizationNode::CallbackReturn VisualizationNode::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() called.");
    teardown_active();
    destroy_renderer_if_any();
    set_mode_sub_.reset();
    set_vcam_srv_.reset();
    set_look_sub_.reset();
    theme_sub_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::visualization_app
