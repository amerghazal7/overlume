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
#include <mutex>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
#include "micropilot_visualization_node/adapters/point_cloud.hpp"
#include "micropilot_visualization_node/adapters/tf_axes.hpp"
#include "micropilot_visualization_node/adapters/trajectory_carpet.hpp"
#include "micropilot_visualization_node/callouts.hpp"
#include "micropilot_visualization_node/camera_ingest.hpp"
#include "micropilot_visualization_node/diagnostics.hpp"
#include "micropilot_visualization_node/frame_transform.hpp"
#include "micropilot_visualization_node/geo_anchor.hpp"
#include "micropilot_visualization_node/hud_overlay.hpp"
#include "micropilot_visualization_node/lidar_colorize.hpp"
#include "micropilot_visualization_node/profile.hpp"
#include "micropilot_visualization_node/quality_governor.hpp"
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
    // Epic 3 Task 5 (VM-032) Step 1: live layer_* param updates -- see
    // layer_param_cb_'s own comment.
    rcl_interfaces::msg::SetParametersResult on_params(
        const std::vector<rclcpp::Parameter>& params);
    void teardown_active();
    void destroy_renderer_if_any();
    // Gathers every subscribed row's AdapterStats (hd_map/dynamic_objects/
    // path/ogm/collision/generic_marker -- NOT tf_axes_rows_, a PRODUCER
    // with no topic/stats of its own) into mpviz_node::BuildDiagnostics(),
    // stamps it, and publishes on ~/diagnostics. Called every tick
    // regardless of mode, same "ingest continues regardless of mode"
    // philosophy as sim_clock_sec_ below.
    void publish_diagnostics();

    // ── virtual-camera presets / eased switching (VM-013) ──────
    // Extracted into its own class (vcam.hpp/vcam.cpp) — owns the preset
    // table, the src_/dst_ tween, and the ~/set_virtual_cam service +
    // ~/set_look subscription. Still the same EGO-RELATIVE OFFSET frame;
    // ego-anchored composition happens in timer_callback, not here (plan
    // Task 5 Scope-addition block).
    std::unique_ptr<Vcam> vcam_;

    // vcam telemetry: [eye xyz | target xyz | active_preset | render_mode |
    // mux_mode] (9 elements -- Step (d), VM-037 appended mux_mode at index 8).
    // Index 7 is now render_mode_ (Task 4/VM-093 gave this node its own
    // local-view mode, mirroring rendering_node's render_mode_/active_mode_
    // split) instead of duplicating active_mode_ the way it used to before
    // Task 4 landed -- identical layout to rendering_node's ~/vcam_state.
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

    // ── local render-mode switch (Task 4 / VM-093) ───────────────────────────
    // SEPARATE from active_mode_ above -- active_mode_ decides WHETHER this
    // node is the mux-authoritative renderer (untouched by this task,
    // Decision 7); render_mode_ decides WHAT this node renders once it is.
    // The dispatch (bowl visibility + the per-mode layer mask,
    // scene_assembly.hpp's bowl_visible_for_mode()/mode_content_mask()) runs
    // every tick in timer_callback(), UNCONDITIONALLY, before the
    // `active_mode_ != 3` early return -- it is not gated on active_mode_ at
    // all. What active_mode_==3 actually gates is only render_frame()/the
    // published image downstream of that dispatch, so render_mode_'s effect
    // becomes VISIBLE only while this node is the mux-selected renderer, even
    // though the dispatch itself always runs. A plain node param
    // (`render_mode`), not a topic -- mirrors rendering_node's own
    // render_mode_/active_mode_ split (rendering_node.hpp:191/203), which
    // this node never had before this task. Live-tunable via on_params(),
    // same as the layer_* bools below -- a mode switch is a `ros2 param
    // set`/set_parameters() call, no ROS message on any topic.
    static constexpr int kRenderModeBowl = 1;
    static constexpr int kRenderModeHybrid = 2;
    static constexpr int kRenderModeFreeLook = 3;
    int render_mode_{kRenderModeFreeLook};

    // ── Surround Stitching (Task 4 / VM-093, follow-up USER DIRECTIVE
    // 2026-09-11) ─────────────────────────────────────────────────────────────
    // A mode-3 (FREE_LOOK) ONLY layer: renders Task 2's camera-textured bowl
    // IN THE SAME FRAME as the full autonomy scene, toggled independently of
    // render_mode_/BOWL. Default false (mode 3's current look is
    // unchanged) -- same disable-knob shape as every layer_* bool above,
    // just not a SceneAssembly category (it gates set_bowl_visible()
    // instead). surround_stitching_profile_ picks which content backs the
    // overlay: "bowl" = Task 2's bowl path alone; "hybrid" = the bowl PLUS
    // Task 5's camera-colorized lidar (visualization_node.cpp's HYBRID-lidar
    // block in timer_callback() also fires for FREE_LOOK when this profile
    // is "hybrid" -- before that fix this
    // profile value was inert and silently rendered bowl-only).
    bool layer_surround_stitching_{false};
    std::string surround_stitching_profile_{"bowl"};
    // the one place that decides "is
    // camera-colorized lidar actually consumed this tick" -- HYBRID mode, or
    // FREE_LOOK with Surround Stitching's hybrid profile (finding 4's fix).
    // Shared by the PointCloud2 callback (skip the whole per-point transform
    // in modes that never sample it) and timer_callback()'s HYBRID-lidar
    // block (single source of truth instead of two copies of this
    // condition) -- both read render_mode_/layer_surround_stitching_/
    // surround_stitching_profile_ live, so a mode switch via on_params()
    // takes effect on the very next PointCloud2 message and the very next
    // tick, no restart.
    bool hybrid_cloud_consumed() const
    {
        return render_mode_ == kRenderModeHybrid ||
               (render_mode_ == kRenderModeFreeLook && layer_surround_stitching_ &&
                surround_stitching_profile_ == "hybrid");
    }

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

    // ── Point clouds (Epic 3 Task 6 / VM-035) ────────────────────────────────
    // One PointCloudAdapter per profile row with adapter: point_cloud.
    // FIXTURE GAP: zero sensor_msgs/PointCloud2 topics exist in any
    // recording, and no shipped profile carries a live row -- unvalidated
    // against a live publisher. Same fill()-appends/timeout_sec/
    // warn_on_drop_growth shape as every category above. PointCloud
    // carries last_update_sec, so this category gets the library's
    // staleness FADE (point_cloud.mat's own settable alpha).
    struct PointCloudRow
    {
        std::unique_ptr<mpviz_node::PointCloudAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs
        uint64_t warned_malformed = 0;  // counts already reported by
        uint64_t warned_no_tf = 0;      // warn_on_drop_growth()
    };
    std::vector<PointCloudRow> point_cloud_rows_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
        point_cloud_subs_;

    // ── Trajectory carpet (VM-077) ────────────────────────────────────────────
    // One TrajectoryCarpetAdapter per profile row with adapter:
    // trajectory_carpet -- today's shipped urban profile carries exactly
    // one (/navigation_motion_obstacle_planner_node/output_trajectory_carpet).
    // Same fill()-appends/timeout_sec/warn_on_drop_growth shape as every
    // category above. TrajectoryCarpet carries last_update_sec, so this
    // category gets the library's staleness FADE
    // (trajectory_carpet.mat's own settable alpha).
    struct CarpetRow
    {
        std::unique_ptr<mpviz_node::TrajectoryCarpetAdapter> adapter;
        double timeout_sec;
        std::string topic;              // named in drop-growth WARNs
        uint64_t warned_malformed = 0;  // counts already reported by
        uint64_t warned_no_tf = 0;      // warn_on_drop_growth()
    };
    std::vector<CarpetRow> carpet_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        carpet_subs_;

    // ── TF-axes debug layer (Epic 2 Task 8 Step 7 / VM-027) ──────────────────
    // One TfAxesAdapter per profile row with adapter: tf_axes -- a
    // PRODUCER, not a subscriber (see that adapter's own header comment for
    // why). Both shipped profiles carry the row COMMENTED (Task 1 Step 3);
    // uncommenting it is the only way this vector is ever non-empty. No
    // subscription branch, no timeout/staleness gating (a live tf2 buffer
    // walk has no "message" to go stale) -- fill() runs every tick,
    // unconditionally, for every row here.
    std::vector<std::unique_ptr<mpviz_node::TfAxesAdapter>> tf_axes_rows_;

    // ── Geo-anchor (VM-050) ───────────────────────────────────────────────────
    // Reuses tf_buffer_ above (the SAME buffer TfAdapter reads) -- no second
    // TransformListener. Fed every /sim/feedback/gps callback until solved
    // (kMinAnchorSamples reached) or overridden by geo_datum_* params
    // (on_configure, validated all-or-nothing). Task 3 (VM-052) reads
    // geo_anchor_solver_->solved()/anchor() at on_activate() time; this task
    // only solves and logs the transition (geo_anchor_logged_ latches the
    // one-shot RCLCPP_INFO on solved() first flipping true).
    std::unique_ptr<GeoAnchorSolver> geo_anchor_solver_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    bool geo_anchor_logged_{false};

    // ── Environment (VM-052) ──────────────────────────────────────────────────
    // environment_enabled_/environment_chunks_dir_ read once in on_configure(),
    // same shape as hud_enabled_/hud_font_path_ below (STANDING directive
    // disable knob + per-checkout-path gap). on_activate() calls
    // mpviz::set_environment_source() iff environment_enabled_ AND
    // geo_anchor_solver_->solved() -- else WARNs once
    // (environment_warned_) and never calls it, per spec §4.5/§9's "no
    // anchor -> environment layer disabled with one WARN". NOT gated per
    // render_mode_ once armed: there is no library-side way to hide a live
    // environment source without tearing down its loaded chunk index (see
    // timer_callback()'s comment above the bowl-visibility dispatch) --
    // buildings render in BOWL/HYBRID too whenever this succeeds. Named
    // exception 7, docs/visual_mode/signoff.md.
    bool environment_enabled_{true};
    std::string environment_chunks_dir_;
    bool environment_warned_{false};

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

    // ── quality auto-drop governor (VM-040, Epic 5) ──────────────────────────
    // Reads render_ms_ (above) every mode-3 tick once governor_enabled_ is
    // true (declared in on_configure(), default false -- a new auto-behavior
    // ships opt-in per this task's own AC). quality_governor_ is constructed
    // in on_configure() with quality_ as its initial preset and every
    // threshold/window below as ROS params (tunable without a code change);
    // timer_callback() feeds it render_ms_ and, on a DROPPED/RECOVERED
    // transition, calls mpviz::set_quality() (VM-040's own appended library
    // entry point -- no renderer re-create) and RCLCPP_WARNs, then updates
    // quality_ so `ros2 param get quality` reports what is actually live.
    bool governor_enabled_{false};
    std::unique_ptr<mpviz_node::QualityGovernor> quality_governor_;

    // ── HUD overlay (Epic 3 Task 3 / VM-030) ─────────────────────────────────
    // hud_font_path_ read once in on_configure() (VM-044 closes the
    // per-checkout-path gap this shares with ego_model_path_'s own default,
    // config/default_params.yaml). hud_font_warned_ makes the
    // missing/unloadable-font WARN one-shot, not per-tick (spec §9, same
    // "non-fatal, WARN once" shape as set_ego_model's own clay-box path).
    std::string hud_font_path_;
    bool hud_font_warned_{false};
    // hud_enabled_ read once in on_configure(), same as hud_font_path_. The
    // STANDING user directive (visual-mode-epic3.md, "every rendered element
    // ships with style + disable config") requires this: when false,
    // timer_callback() never calls CompositeHud() and frame_buf_ passes
    // through untouched -- not "empty font path", a real disable knob. No
    // node-level gtest exercises the guard itself (no existing test in this
    // suite stands up a full VisualizationNode/rclcpp harness -- see Step
    // 0's own note on why PopulateHud became a free function instead of
    // adding one); the guard is a single `if` around an already-tested call,
    // documented here and at its call site instead.
    bool hud_enabled_{true};

    // ── Alert callouts (Epic 3 Task 4 / VM-031) ──────────────────────────────
    // callouts_enabled_ read once in on_configure(), same shape as
    // hud_enabled_ above (STANDING directive disable knob): when false,
    // timer_callback() never even calls BuildNearestCallout(), frame_buf_
    // passes through untouched from the HUD block above. No node-level gtest
    // exercises the guard itself, same "no full VisualizationNode/rclcpp
    // harness in this suite" reason hud_enabled_'s own comment gives.
    bool callouts_enabled_{true};

    // ── Layer visibility (Epic 3 Task 5 / VM-032) ────────────────────────────
    // Disable-knob-per-category, STANDING directive -- but unlike
    // hud_enabled_/callouts_enabled_ above, these are read LIVE (on_params()
    // below, registered via layer_param_cb_): a GUI/WS set_layers update
    // takes effect on the very next timer_callback() tick, no restart. Node
    // side gate only (Step 0 decision) -- timer_callback() clears the
    // matching scene_asm_ vector right before point_at() when false; no
    // renderer-side change. layer_point_clouds_ gates scene_asm_.
    // point_clouds as of Task 6 (VM-035) -- see timer_callback().
    bool layer_objects_{true};
    bool layer_paths_{true};
    bool layer_map_elements_{true};
    bool layer_grids_{true};
    bool layer_alerts_{true};
    bool layer_markers_{true};
    bool layer_point_clouds_{true};
    // VM-077: gates scene_asm_.trajectory_carpets. Singular knob name,
    // matching the category's own singular topic/adapter/role.
    bool layer_trajectory_carpet_{true};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr layer_param_cb_;

    // ── Camera bowl ingest (VM-091, unified-engine migration Task 2 Step 6) ──
    // bowl_enabled_ is the STANDING disable knob (read once in on_configure,
    // same shape as hud_enabled_/callouts_enabled_ above): false means
    // camera_ingest_'s image callbacks do no cv_bridge conversion work and
    // set_bowl_config()/set_camera_frame()/set_camera_motion_delta() are
    // never called. bowl_R0_/bowl_k_/bowl_Rmax_/feather_margin_/sky_color_/
    // exposure_compensation_ are the BowlConfig fields camera_ingest_ can't
    // fill itself (camera intrinsics/extrinsics + dims come from
    // camera_ingest_, everything else is a plain ROS param this node owns
    // directly, same split as every other adapter's config-vs-ingest
    // separation in this file). fill_blind_zone_/exposure_match_ are
    // declared but CLAMPED to false (Decision 3's exposure/blind-zone
    // deferral -- neither has a Filament-side implementation this epic) in
    // both on_configure() and on_params() below, with a WARN whenever a
    // config or set_parameters() call carries `true`.
    bool bowl_enabled_{false};
    // One-shot WARN latch -- render_mode BOWL/HYBRID while bowl_enabled_ is
    // false (the shipped default) masks the whole autonomy scene for a bowl
    // that was never configured, a near-empty frame with no diagnostic
    // otherwise. See on_configure()'s close-out check and on_params()'s
    // render_mode branch, both of which set this the first time they warn.
    bool bowl_mode_warned_{false};
    // GUI-tunable; in this merged node it is the ego-motion re-alignment/
    // staleness window (Step 6's redefined frame-sync gate semantics) --
    // handed to camera_ingest_ via set_max_sync_latency() (VM-091 gate
    // close-out finding 3): update_motion_deltas() compares each camera's
    // (t_max - stamp) spread against it and THROTTLE-WARNs when exceeded,
    // carrying over the old node's gate WARN even though delta-compensation
    // and rendering both proceed regardless (identity/rig_delta
    // compensation runs every tick either way).
    double max_sync_latency_{0.12};
    double bowl_R0_{6.0}, bowl_k_{0.08}, bowl_Rmax_{20.0};
    double feather_margin_{30.0};
    bool fill_blind_zone_{false};
    bool exposure_match_{false};
    // VM-091 gate close-out finding 2: on_params() stores a live edit to
    // any of bowl_R0_/bowl_k_/bowl_Rmax_/feather_margin_/sky_color_/
    // bowl_exposure_compensation_ into these very members, but nothing used
    // to re-call apply_bowl_config() -- the timer's own bowl block only
    // re-baked on a CameraInfo change (consume_info_dirty()), so a GUI
    // slider drag was silently inert until some unrelated camera reconnect
    // happened to re-bake. Set true by on_params() on any such edit;
    // consumed (and cleared) by timer_callback()'s bowl block, which
    // re-calls apply_bowl_config() (a full re-bake, same contract as the
    // CameraInfo-change path) once all_info_ready() -- the same "GUI edit
    // costs at most one dropped frame" budget the plan's own set_bowl_config
    // contract already accepts for a slider drag.
    bool bowl_config_dirty_{false};
    float sky_color_[3]{0.53f, 0.70f, 0.92f};
    // MEASURED (bowl color fidelity fix, 2026-09-11), not guessed -- see
    // scene.h's BowlConfig::exposure_compensation / default_params.yaml's
    // own comment for the gray-ramp probe methodology behind 1.56.
    float bowl_exposure_compensation_{1.56f};
    // Task 4/VM-093 owns the real per-mode set_bowl_visible() dispatch (see
    // timer_callback()): visible in BOWL/HYBRID, hidden in FREE_LOOK unless
    // layer_surround_stitching_ is on -- computed fresh every tick from
    // render_mode_/layer_surround_stitching_ above, regardless of
    // bowl_enabled_/active_mode_ (set_bowl_visible() itself no-ops when the
    // bowl was never configured, scene.h's own contract).
    std::unique_ptr<CameraIngest> camera_ingest_;

    // ── Hybrid lidar colorization (VM-094, unified-engine migration Task 5) ──
    // hybrid_enabled_ is the STANDING disable knob (read once in
    // on_configure, same shape as bowl_enabled_ above): false means
    // cloud_sub_ is never created and timer_callback()'s HYBRID block never
    // runs -- render_mode HYBRID then falls back to bowl-only (Decision 5).
    // Meaningless without camera_ingest_ (bowl_enabled_ true): the rig
    // geometry and last-ingested RGB buffers ColorizeFromCameras() samples
    // both come from it.
    bool hybrid_enabled_{false};
    std::string pointcloud_topic_;
    // [R(9 row-major)|t(3)] cloud-frame -> rig-frame -- same param name/
    // convention as micropilot_rendering_node's own pointcloud_transform
    // (rendering_node.cpp:95-96).
    float pointcloud_tf_[12]{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    // Guards cloud_pts_rig_: written by cloud_sub_'s callback, read (and
    // swapped out) by timer_callback() -- same mutex-around-a-snapshot
    // shape as camera_ingest.cpp's own odom_mtx_/twists_.
    std::mutex cloud_mtx_;
    std::vector<mpviz::Vec3> cloud_pts_rig_;  // rig frame (post pointcloud_transform)

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace micropilot::visualization_app
