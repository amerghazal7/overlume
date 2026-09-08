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
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "micropilot_visualization_node/adapters/collision.hpp"
#include "micropilot_visualization_node/adapters/dynamic_objects.hpp"
#include "micropilot_visualization_node/adapters/generic_marker.hpp"
#include "micropilot_visualization_node/adapters/hd_map.hpp"
#include "micropilot_visualization_node/adapters/ogm.hpp"
#include "micropilot_visualization_node/adapters/path.hpp"
#include "micropilot_visualization_node/adapters/tf_axes.hpp"
#include "micropilot_visualization_node/diagnostics.hpp"
#include "micropilot_visualization_node/frame_transform.hpp"
#include "micropilot_visualization_node/profile.hpp"
#include "micropilot_visualization_node/scene_assembly.hpp"
#include "micropilot_visualization_node/tf_adapter.hpp"
#include "micropilot_visualization_node/vcam.hpp"

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
    // Epic 3 Task 2 (VM-034) Step 0: gathers every subscribed row's
    // AdapterStats (hd_map/dynamic_objects/path/ogm/collision/generic_marker
    // -- NOT tf_axes_rows_, a PRODUCER with no topic/stats of its own) into
    // mpviz_node::BuildDiagnostics(), stamps it, and publishes on
    // ~/diagnostics. Called every tick regardless of mode (Step 0's own AC:
    // "diagnostics shows per-topic age... and render_ms"), same "ingest
    // continues regardless of mode" philosophy as sim_clock_sec_.
    void publish_diagnostics();

    // ── virtual-camera presets / eased switching (plan Task 5 / VM-013) ──────
    // Extracted into its own class (vcam.hpp/vcam.cpp) — owns the preset
    // table, the src_/dst_ tween, and the ~/set_virtual_cam service +
    // ~/set_look subscription. Still the same EGO-RELATIVE OFFSET frame;
    // ego-anchored composition happens in timer_callback, not here (plan
    // Task 5 Scope-addition block).
    std::unique_ptr<Vcam> vcam_;

    // vcam telemetry: [eye xyz | target xyz | active_preset | active_mode],
    // one per timer tick — identical layout to rendering_node's ~/vcam_state.
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pub_vcam_state_;

    // ── configuration (declared in on_configure) ─────────────────────────────
    int out_width_{1280};
    int out_height_{720};
    int quality_{2};       // 0=low, 1=med, 2=high (mpviz::RenderConfig::quality)
    int initial_mode_{1};  // last-configured global mux mode; matches rendering_node's default
    // Current (possibly tweening) pose -- mirrors vcam_->pose() each tick
    // (timer_callback); virtual_pose + virtual_vfov_deg params seed vcam_'s
    // own preset table at construction (on_configure).
    mpviz::CameraPose pose_{};

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

    // ── HD-map lanes/crosswalks (Epic 2 Task 2 / VM-024) ─────────────────────
    // One HdMapAdapter per profile row with adapter: hd_map (urban ships 3;
    // sim adds a 4th, latched /sim/hd_map/markers) -- every instance's
    // fill() APPENDS into scene_asm_.map_elements (SceneAssembly's own
    // header: the last adapter to run must not erase what the others
    // already appended). frame_transformer_ is heap-allocated because
    // FrameTransformer holds a `const tf2_ros::Buffer&` that can only bind
    // once tf_buffer_ exists (on_configure, not construction).
    struct HdMapRow
    {
        std::unique_ptr<mpviz_node::HdMapAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs
        uint64_t warned_malformed = 0;  // counts already reported by
        uint64_t warned_no_tf = 0;      // warn_on_drop_growth()
    };
    std::unique_ptr<FrameTransformer> frame_transformer_;
    std::vector<HdMapRow> hd_map_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        hd_map_subs_;

    // ── Dynamic objects (Epic 2 Task 3 / VM-021) ─────────────────────────────
    // One DynamicObjectsAdapter per profile row with adapter: dynamic_objects
    // (urban/offroad/sim ship exactly one: /perception/dynamic_objects_list).
    // class_inference_ is loaded once in on_configure and must outlive every
    // adapter, which holds a `const ClassInferenceTable&` (same reference-
    // member shape as FrameTransformer above).
    struct DynamicObjectsRow
    {
        std::unique_ptr<mpviz_node::DynamicObjectsAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    mpviz_node::ClassInferenceTable class_inference_;
    std::vector<DynamicObjectsRow> dynamic_objects_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        dynamic_objects_subs_;

    // ── Path ribbons (Epic 2 Task 5 / VM-023) ────────────────────────────────
    // One PathAdapter per profile row with adapter: path -- both shipped
    // profiles ship FOUR rows over THREE roles (BEHAVIOR, LOCAL x2, GLOBAL;
    // fixture gap 2). Same fill()-appends/timeout_sec/warn_on_drop_growth
    // shape as hd_map/dynamic_objects above. PathRibbon carries
    // last_update_sec, so this category gets the library's staleness
    // FADE -- mark_stale_tick() past timeout_sec, same as the
    // dynamic_objects loop.
    struct PathRow
    {
        std::unique_ptr<mpviz_node::PathAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs
        uint64_t warned_malformed = 0;  // counts already reported by
        uint64_t warned_no_tf = 0;      // warn_on_drop_growth()
    };
    std::vector<PathRow> path_rows_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subs_;

    // ── OGM ground grids (Epic 2 Task 6 / VM-025) ────────────────────────────
    // One OgmAdapter per profile row with adapter: ogm (both shipped
    // profiles ship exactly two: dynamic_ogm/gradient_ogm). UNLIKE every
    // other category, subscriptions_for(row) returns TWO SubSpecs for one
    // row (the base topic + row.update_topic), so this loop creates TWO
    // subscriptions per row -- ogm_grid_subs_/ogm_update_subs_ stay
    // index-aligned with ogm_rows_ (slot i's two subscriptions both bind
    // the SAME adapter instance, via ingest()/ingest_update() respectively).
    // FIXTURE GAP 3: no OccupancyGrid topic exists in the recorded bag --
    // unvalidated against a live publisher. GroundGridLayer DOES carry
    // last_update_sec, so this category gets the library's staleness FADE
    // (ground_grid.mat's own alpha), same "stop filling past timeout_sec,
    // mark_stale_tick() instead" shape as dynamic_objects/path above.
    struct OgmRow
    {
        std::unique_ptr<mpviz_node::OgmAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs (the base topic)
        uint64_t warned_malformed = 0;  // counts already reported by
        uint64_t warned_no_tf = 0;      // warn_on_drop_growth()
    };
    std::vector<OgmRow> ogm_rows_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> ogm_grid_subs_;
    std::vector<rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr>
        ogm_update_subs_;

    // ── Collision alert polygons (Epic 2 Task 7 / VM-026) ────────────────────
    // One CollisionAdapter per profile row with adapter: collision -- urban
    // ships all FIVE (collision_markers/object_predicted_polygons/
    // ego_footprint_sweep/ego_merged_polygon/object_merged_polygons, Task 1
    // Step 3). FIXTURE GAP 4: all five topics were silent in the recorded
    // bag (a calm scenario) -- unvalidated against a live publisher. Same
    // fill()-appends/timeout_sec/warn_on_drop_growth shape as
    // dynamic_objects/path/ogm above. AlertPolygon carries
    // last_update_sec, so this category gets the library's staleness FADE
    // (its severity's constant alpha, multiplied down) -- mark_stale_tick()
    // past timeout_sec, same as every other faded category.
    struct CollisionRow
    {
        std::unique_ptr<mpviz_node::CollisionAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs
        uint64_t warned_malformed = 0;  // counts already reported by
        uint64_t warned_no_tf = 0;      // warn_on_drop_growth()
    };
    std::vector<CollisionRow> collision_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        collision_subs_;

    // ── Generic marker fallback (Epic 2 Task 8 / VM-027) ─────────────────────
    // One GenericMarkerAdapter per profile row with adapter: generic -- the
    // spec §7 parity guarantee (ANY MarkerArray topic renders via one YAML
    // row). Today's shipped profiles carry exactly one:
    // /sim/ground_truth/boxes (role neutral, best_effort: true, the bag's
    // only non-map-frame, only lifetime-expiring topic). Same fill()-
    // appends/timeout_sec/warn_on_drop_growth shape as every category
    // above -- GenericMarker carries last_update_sec, so this category
    // gets the library's staleness FADE.
    struct GenericMarkerRow
    {
        std::unique_ptr<mpviz_node::GenericMarkerAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs
        uint64_t warned_malformed = 0;  // counts already reported by
        uint64_t warned_no_tf = 0;      // warn_on_drop_growth()
    };
    std::vector<GenericMarkerRow> generic_marker_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        generic_marker_subs_;

    // ── TF-axes debug layer (Epic 2 Task 8 Step 7 / VM-027) ──────────────────
    // One TfAxesAdapter per profile row with adapter: tf_axes -- a
    // PRODUCER, not a subscriber (see that adapter's own header comment for
    // why). Both shipped profiles carry the row COMMENTED (Task 1 Step 3);
    // uncommenting it is the only way this vector is ever non-empty. No
    // subscription branch, no timeout/staleness gating (a live tf2 buffer
    // walk has no "message" to go stale) -- fill() runs every tick,
    // unconditionally, for every row here.
    std::vector<std::unique_ptr<mpviz_node::TfAxesAdapter>> tf_axes_rows_;

    SceneAssembly scene_asm_;

    // ── renderer + preallocated output buffer ────────────────────────────────
    mpviz::VisualRenderer* renderer_{nullptr};
    std::vector<uint8_t> frame_buf_;  // out_width_*out_height_*3, rgb8

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;

    // ── diagnostics (Epic 3 Task 2 / VM-034) ─────────────────────────────────
    // render_ms_ is measured around mpviz::render_frame() in timer_callback()
    // and fed into publish_diagnostics() -- ONLY while active_mode_==3
    // (spec's own render/readback/publish gate); every other tick explicitly
    // zeros it rather than leaving the last mode-3 tick's number in place, so
    // a diagnostics consumer never mistakes a stale number for a live one.
    double render_ms_{0.0};
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        pub_diagnostics_;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace micropilot::visualization_app
