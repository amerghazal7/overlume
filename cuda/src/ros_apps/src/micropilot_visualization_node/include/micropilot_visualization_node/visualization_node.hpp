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

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

// Existing srv package (spec §6 / plan Task 5) — dependency, not a copy:
// this node re-implements the CUDA node's vcam surface under its own
// namespace but shares the exact same service type.
#include "micropilot_rendering_node/srv/set_virtual_cam.hpp"
#include "micropilot_visualization_node/tf_adapter.hpp"

namespace micropilot::visualization_app
{

/// A virtual-camera placement as look-points in the rig frame (x-fwd, y-left,
/// z-up) — mirrors micropilot_rendering_node's LookPoint exactly (plan Task 5:
/// "port the tween math verbatim"). float precision is kept (not double, like
/// mpviz::CameraPose) so that presets shared numerically with the CUDA node
/// (spec §6: "presets/orbits produce the same framing in both worlds") tween
/// to bit-identical results — the contract test's 1e-9 tolerance would not be
/// safely met if the two nodes rounded the same constants differently.
struct LookPoint
{
    float eye[3];
    float target[3];
};

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

    // ── virtual-camera presets / eased switching (plan Task 5) ───────────────
    using SetVirtualCam = micropilot_rendering_node::srv::SetVirtualCam;
    /// Advance the smoothstep tween one timer tick and update cur_ + pose_.
    void advance_tween();
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
    static constexpr std::array<const char*, 5> kPresetNames{
        "config", "reverse_follow", "left_side", "right_side", "top_down"};
    LookPoint cur_{}, src_{}, dst_{};
    double tween_t_{1.0};  // [0,1]; 1.0 = settled on dst_. Eased per timer tick.
    int active_preset_{1};  // 1-5 = preset in service numbering; 0 = free look

    rclcpp::Service<SetVirtualCam>::SharedPtr set_vcam_srv_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_look_sub_;
    // vcam telemetry: [eye xyz | target xyz | active_preset | active_mode],
    // one per timer tick — identical layout to rendering_node's ~/vcam_state.
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pub_vcam_state_;

    // ── configuration (declared in on_configure) ─────────────────────────────
    int out_width_{1280};
    int out_height_{720};
    int quality_{2};       // 0=low, 1=med, 2=high (mpviz::RenderConfig::quality)
    int initial_mode_{1};  // last-configured global mux mode; matches rendering_node's default
    mpviz::CameraPose pose_{};  // current (possibly tweening) pose; virtual_pose + virtual_vfov_deg params seed presets_[0]

    // ── mode mux ──────────────────────────────────────────────────────────────
    // Global (not "~/...") — both this node and rendering_node subscribe the
    // same topic (spec §3.1). Renders+publishes only while == 3.
    int active_mode_{1};
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_mode_sub_;

    // ── theme (Epic 1 Task 3 / VM-014) ───────────────────────────────────────
    // Node-private (not global like set_mode_sub_ above) -- a mode-3-only
    // concept, harmless if published while mode 1/2 is active (same
    // "ingest continues regardless of mode" philosophy as sim_clock_sec_
    // below). ~/set_theme -> mpviz::set_theme(renderer_, name, sim_clock_sec_, 0.0)
    // (0.0 -> library default transition duration, 0.8s).
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr theme_sub_;

    // ── ego (Epic 1 Task 4 / VM-012) ─────────────────────────────────────────
    // map->base_link TF -> mpviz::SceneGraph::ego (position/heading/smoothed
    // speed), rebuilt every timer tick before set_scene(), same "ingest
    // continues regardless of mode" philosophy as sim_clock_sec_ above. This
    // node owns the Buffer/TransformListener (constructing a
    // TransformListener needs the node's own NodeInterfaces); tf_adapter_
    // just holds the small bit of finite-difference/EMA state on top of it.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<micropilot::visualization_app::TfAdapter> tf_adapter_;
    // Global (not "~/...", same reasoning as set_mode_sub_ above -- this is
    // the robot's own feedback, not a per-node control): spec §7 preferred
    // speed source. Forwards each sample into tf_adapter_ (see its header).
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr robot_speed_sub_;
    // ~/ego_state debug telemetry, one Float64MultiArray per timer tick --
    // [x, y, z, heading_rad, speed_mps, valid] -- same per-tick publish
    // pattern as ~/vcam_state above; exists so tests (and any external
    // debugging UI) can observe the TF-driven ego without a GPU/render path.
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pub_ego_state_;
    // Monotonic node clock driving SceneGraph::sim_time_sec (and therefore
    // every staleness-fade/theme-transition computation downstream) --
    // incremented by the timer's own period (kTimerPeriodSec) each tick,
    // NEVER read from wall-clock, so the deterministic-clock contract
    // (scene.h) holds all the way out to the running node, not just in
    // tests that drive set_scene()/render_frame() directly.
    static constexpr double kTimerPeriodSec = 0.033;  // matches the 33ms create_wall_timer below
    double sim_clock_sec_{0.0};

    // ── renderer + preallocated output buffer ────────────────────────────────
    mpviz::VisualRenderer* renderer_{nullptr};
    std::vector<uint8_t> frame_buf_;  // out_width_*out_height_*3, rgb8

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace micropilot::visualization_app
