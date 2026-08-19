/** @file visualization_node.cpp
 *  @brief ROS2 LifecycleNode wrapping micropilot::visualization (Filament).
 */

#include "micropilot_visualization_node/visualization_node.hpp"

#include <cmath>
#include <cstring>

#include <std_msgs/msg/header.hpp>

#include "micropilot_visualization_node/ego_anchor.hpp"

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

    // ── ego model (Epic 1 Task 4 / VM-012) ───────────────────────────────────
    // Mirrors micropilot_rendering_node's robot_model_path convention
    // exactly: "" is a legal default, load failure (missing file, bad
    // asset) is non-fatal (mpviz::set_ego_model's own contract already
    // falls back to a themed clay box at fallback_dims -- this WARN is
    // purely informational, not a gate).
    auto ego_model_path = declare_parameter<std::string>("ego_model_path", "");
    auto ego_dims = declare_parameter<std::vector<double>>("ego_fallback_dims", {4.5, 2.0, 1.8});
    if (ego_dims.size() != 3)
    {
        RCLCPP_ERROR(get_logger(), "ego_fallback_dims must be 3 floats [len, width, height], got %zu",
                     ego_dims.size());
        return CallbackReturn::FAILURE;
    }
    mpviz::Vec3 ego_fallback_dims{ego_dims[0], ego_dims[1], ego_dims[2]};
    if (!mpviz::set_ego_model(renderer_, ego_model_path.c_str(), ego_fallback_dims))
    {
        RCLCPP_WARN(get_logger(), "set_ego_model: failed to load '%s' -- using clay-box fallback",
                    ego_model_path.c_str());
    }

    // ── TF adapter (Epic 1 Task 4 / VM-012) ──────────────────────────────────
    // map->base_link -> SceneGraph.ego, finite-differenced + EMA-smoothed
    // speed. Buffer/TransformListener live on the node (need its
    // NodeInterfaces to construct); TfAdapter just wraps the lookup +
    // smoothing math on top.
    auto ego_speed_smoothing_alpha = declare_parameter<double>("ego_speed_smoothing_alpha", 0.2);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_adapter_ = std::make_unique<TfAdapter>(*tf_buffer_, "map", "base_link",
                                              ego_speed_smoothing_alpha);
    pub_ego_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/ego_state", 1);

    // Spec §7 (docs/superpowers/specs/2026-08-18-visual-mode-design.md:264):
    // ego speed PREFERS this topic over the TF finite-difference fallback
    // tf_adapter_ computes above. Global (not "~/..."): it's the robot's own
    // feedback, published once regardless of which mux mode/node is active.
    robot_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/robot/feedback/robot_speed_mps", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg)
        { tf_adapter_->set_robot_speed_mps(msg->data); });

    // ── publishers (created here, activated in on_activate) ─────────────────
    // Same global topic names as rendering_node — exactly one node publishes
    // at a time (mode mux, spec §3.1); consumers never re-subscribe.
    pub_image_ = create_publisher<sensor_msgs::msg::Image>("/rendering/image", 1);
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/rendering/camera_info", 1);
    pub_vcam_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/vcam_state", 1);

    // ── virtual-camera presets / tween (plan Task 5 / VM-013) ────────────────
    // Vcam's constructor seeds presets_[0] ("config") from pose_ with the
    // identical formula this file used inline before the extraction (see
    // vcam.cpp). Constructed AFTER every failure gate above, at the same
    // point the pre-extraction code created the ~/set_virtual_cam service and
    // ~/set_look subscription — a failed configure must not leave a live vcam
    // control surface advertised (Opus review, Task 5 round 1).
    vcam_ = std::make_unique<Vcam>(this, pose_);

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
    pub_ego_state_->on_activate();

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
    vcam_->advance_tween();
    pose_ = vcam_->pose();

    std_msgs::msg::Float64MultiArray state;
    state.data = {pose_.eye[0],    pose_.eye[1],    pose_.eye[2],
                  pose_.target[0], pose_.target[1], pose_.target[2],
                  static_cast<double>(vcam_->active_preset()),
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
    scene.ego = tf_adapter_->update();
    mpviz::set_scene(renderer_, scene);

    std_msgs::msg::Float64MultiArray ego_state;
    ego_state.data = {scene.ego.position.x,   scene.ego.position.y, scene.ego.position.z,
                      scene.ego.heading_rad,  scene.ego.speed_mps,
                      static_cast<double>(scene.ego.valid)};
    pub_ego_state_->publish(ego_state);

    // Render/readback/publish only while this node is the active mux output
    // (spec §3.1) — costs ~zero GPU otherwise.
    if (active_mode_ != 3) return;

    // Ego-anchored camera composition (2026-08-19 user directive, plan Task 5
    // scope addition): compose HERE ONLY, right before handing the pose to
    // the renderer -- pose_/cur_/telemetry above are never touched, so an
    // unchanged offset + a moving ego composes into a smooth follow with no
    // drift or feedback, and orbits/presets keep adjusting the offset only.
    mpviz::CameraPose render_pose = pose_;
    if (scene.ego.valid)
    {
        render_pose = compose_ego_anchored_pose(pose_, scene.ego);
    }
    // else: no TF yet -- offset pose used as an absolute world pose, exactly
    // today's behavior (keeps test_vcam_contract.py and every no-TF test
    // bit-identical).

    mpviz::FrameView view{frame_buf_.data(), static_cast<uint32_t>(out_width_),
                          static_cast<uint32_t>(out_height_)};
    if (!mpviz::render_frame(renderer_, render_pose, view))
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
    pub_vcam_state_->on_deactivate();
    pub_ego_state_->on_deactivate();
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
    pub_ego_state_.reset();
    set_mode_sub_.reset();
    vcam_.reset();
    theme_sub_.reset();
    robot_speed_sub_.reset();
    tf_adapter_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

VisualizationNode::CallbackReturn VisualizationNode::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() called.");
    teardown_active();
    destroy_renderer_if_any();
    set_mode_sub_.reset();
    vcam_.reset();
    theme_sub_.reset();
    robot_speed_sub_.reset();
    tf_adapter_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::visualization_app
