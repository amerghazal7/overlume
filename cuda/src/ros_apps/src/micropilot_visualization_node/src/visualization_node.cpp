/** @file visualization_node.cpp
 *  @brief ROS2 LifecycleNode wrapping micropilot::visualization (Filament).
 */

#include "micropilot_visualization_node/visualization_node.hpp"

#include <cmath>
#include <cstring>

#include <ament_index_cpp/get_package_share_directory.hpp>
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

    // Gate-review addition (2026-08-20, spec §9 minor): create_renderer()
    // silently substitutes its compiled-in fallback theme whenever the
    // requested theme_assets_dir/initial_theme fails to load -- non-fatal by
    // design (rendering still comes up), but until now gave this node no way
    // to WARN that it happened. config.theme_assets_dir is null here (this
    // node doesn't expose a parameter for it yet), so the dir actually tried
    // is the library's own compiled-in default.
    if (!mpviz::theme_assets_loaded(renderer_))
    {
        RCLCPP_WARN(get_logger(),
                    "theme assets failed to load from '%s' -- rendering with the "
                    "compiled-in fallback theme instead",
                    config.theme_assets_dir ? config.theme_assets_dir
                                             : "<compiled-in default theme dir>");
    }

    // ── profile YAML loader (Epic 2 Task 1 / VM-020) ─────────────────────────
    // Drives which adapters subscribe to what (Tasks 2-8 add the actual
    // create_subscription() calls, one adapter at a time, by walking
    // mpviz_node::subscriptions_for(row) over profile->rows -- nothing here
    // subscribes to anything yet). Failure to load is fatal (matches the
    // existing virtual_pose validation convention): every collected error
    // is logged, not just the first, because a config file with three
    // mistakes should take one edit pass, not three.
    auto profile_name = declare_parameter<std::string>("profile", "urban");
    auto profile_dir_param = declare_parameter<std::string>("profile_dir", "");
    std::string profile_dir = profile_dir_param;
    if (profile_dir.empty())
    {
        profile_dir = ament_index_cpp::get_package_share_directory("micropilot_visualization_node") +
                      "/config";
    }
    const std::string profile_path = profile_dir + "/" + profile_name + "_profile.yaml";
    std::vector<std::string> profile_errors;
    auto profile = mpviz_node::load_profile(profile_path, profile_errors);
    if (!profile.has_value())
    {
        RCLCPP_ERROR(get_logger(), "failed to load profile '%s':", profile_path.c_str());
        for (const auto& err : profile_errors) RCLCPP_ERROR(get_logger(), "  %s", err.c_str());
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "profile '%s' loaded (%zu rows) from '%s'", profile->name.c_str(),
                profile->rows.size(), profile_path.c_str());
    // load_profile() can return a valid profile AND non-fatal warnings (e.g.
    // "unknown key 'best_efort' (ignored)") -- the ERROR branch above logs
    // profile_errors on failure, but a successful load must too, or the one
    // diagnostic naming a config typo is computed and silently dropped.
    for (const auto& err : profile_errors) RCLCPP_WARN(get_logger(), "  %s", err.c_str());
    for (const auto& row : profile->rows)
    {
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty())
        {
            RCLCPP_INFO(get_logger(), "  (no subscription) -> %s/%s", row.adapter.c_str(),
                        row.role.c_str());
            continue;
        }
        for (const auto& spec : specs)
        {
            RCLCPP_INFO(get_logger(), "  %s -> %s/%s (qos: %s%s)", spec.topic.c_str(),
                        row.adapter.c_str(), row.role.c_str(),
                        spec.best_effort ? "best_effort" : "reliable",
                        spec.transient_local ? "+transient_local" : "");
        }
    }

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
    // flatten_z (user directive 2026-08-20): the HD-map layer is a 2D plane
    // today, so real z (live TF altitude, dynamic-object bbox centers) would
    // otherwise render as floating geometry -- see frame_transform.hpp/
    // tf_adapter.hpp's own ctor docs. Set false once the HD-map layer grows
    // real 3D coordinates.
    auto flatten_z = declare_parameter<bool>("flatten_z", true);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_adapter_ = std::make_unique<TfAdapter>(*tf_buffer_, "map", "base_link",
                                              ego_speed_smoothing_alpha, flatten_z);
    pub_ego_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/ego_state", 1);

    // ── HD-map adapters (Epic 2 Task 2 / VM-024) ─────────────────────────────
    // One HdMapAdapter per profile row with adapter: hd_map, subscribed via
    // subscriptions_for(row) -- the pure function Task 1 built specifically
    // so this loop never hand-rolls QoS logic. fill() APPENDS into
    // scene_asm_ every tick (timer_callback), never assigns it, so all of
    // urban's 3 rows (and sim's 4th, latched) render together.
    frame_transformer_ = std::make_unique<FrameTransformer>(*tf_buffer_, "map", flatten_z);
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "hd_map") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<mpviz_node::HdMapAdapter>(row, *frame_transformer_);
        mpviz_node::HdMapAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        hd_map_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));
        hd_map_rows_.push_back(HdMapRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "hd_map: %zu row(s) subscribed", hd_map_rows_.size());

    // ── Dynamic objects (Epic 2 Task 3 / VM-021) ─────────────────────────────
    // class_inference.yaml lives next to the profile YAMLs (same share/config
    // directory, same profile_dir override) -- loaded once, before any
    // DynamicObjectsAdapter is constructed, since every adapter holds a
    // reference to it for the lifetime of this configure/activate cycle.
    // Failure to load is fatal for the same reason a bad profile is: a
    // config file this node depends on to classify every tracked object.
    const std::string class_inference_path = profile_dir + "/class_inference.yaml";
    std::vector<std::string> class_inference_errors;
    if (auto table = mpviz_node::load_class_inference(class_inference_path, class_inference_errors))
    {
        class_inference_ = std::move(*table);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "failed to load class inference table '%s':",
                    class_inference_path.c_str());
        for (const auto& err : class_inference_errors) RCLCPP_ERROR(get_logger(), "  %s", err.c_str());
        return CallbackReturn::FAILURE;
    }

    // One DynamicObjectsAdapter per profile row with adapter: dynamic_objects
    // -- same subscriptions_for(row)/QoS pattern as the hd_map loop above.
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "dynamic_objects") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<mpviz_node::DynamicObjectsAdapter>(
            row, *frame_transformer_, class_inference_);
        mpviz_node::DynamicObjectsAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        dynamic_objects_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));
        dynamic_objects_rows_.push_back(
            DynamicObjectsRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "dynamic_objects: %zu row(s) subscribed",
                dynamic_objects_rows_.size());

    // ── Path ribbons (Epic 2 Task 5 / VM-023) ────────────────────────────────
    // One PathAdapter per profile row with adapter: path -- same
    // subscriptions_for(row)/QoS pattern as the hd_map/dynamic_objects loops
    // above. Both shipped profiles ship FOUR rows over THREE roles.
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "path") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<mpviz_node::PathAdapter>(row, *frame_transformer_);
        mpviz_node::PathAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        path_subs_.push_back(create_subscription<nav_msgs::msg::Path>(
            spec.topic, qos,
            [this, adapter_ptr](const nav_msgs::msg::Path::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));
        path_rows_.push_back(PathRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "path: %zu row(s) subscribed", path_rows_.size());

    // ── OGM ground grids (Epic 2 Task 6 / VM-025) ────────────────────────────
    // One OgmAdapter per profile row with adapter: ogm -- UNLIKE every other
    // loop above, subscriptions_for(row) returns TWO SubSpecs for this one
    // row (the base topic + row.update_topic), so this creates TWO
    // subscriptions, binding each spec's `type` to the matching ingest()
    // overload on the SAME adapter instance (profile.cpp's own comment:
    // best_effort propagates to the update stream, transient_local never
    // does -- an update stream is inherently VOLATILE).
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "ogm") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.size() != 2) continue;  // profile.cpp always returns 2 for adapter: ogm
        const auto& gridSpec = specs[0];
        const auto& updateSpec = specs[1];

        auto adapter = std::make_unique<mpviz_node::OgmAdapter>(row, *frame_transformer_);
        mpviz_node::OgmAdapter* adapter_ptr = adapter.get();

        rclcpp::QoS gridQos(10);
        if (gridSpec.best_effort) gridQos.best_effort();
        if (gridSpec.transient_local) gridQos.transient_local();
        ogm_grid_subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
            gridSpec.topic, gridQos,
            [this, adapter_ptr](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));

        rclcpp::QoS updateQos(10);
        if (updateSpec.best_effort) updateQos.best_effort();
        if (updateSpec.transient_local) updateQos.transient_local();
        ogm_update_subs_.push_back(create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            updateSpec.topic, updateQos,
            [this, adapter_ptr](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
            { adapter_ptr->ingest_update(*msg, sim_clock_sec_); }));

        ogm_rows_.push_back(OgmRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "ogm: %zu row(s) subscribed", ogm_rows_.size());

    // Spec §7 (docs/superpowers/specs/2026-08-18-visual-mode-design.md:264):
    // ego speed PREFERS this topic over the TF finite-difference fallback
    // tf_adapter_ computes above. Global (not "~/..."): it's the robot's own
    // feedback, published once regardless of which mux mode/node is active.
    // QoS fix (Epic 2 Task 1 / VM-020 Step 4): the bag's metadata.yaml
    // records this publisher's offered `reliability: 2` (BEST_EFFORT). A
    // bare `10` here defaults to RELIABLE, which NEVER matches a
    // BEST_EFFORT publisher -- no error, no warning, a permanently silent
    // topic, with the TF finite-difference fallback quietly covering for
    // it. Same root cause as the profile `best_effort` field (see
    // urban_profile.yaml's /sim/ground_truth/boxes row); this subscription
    // isn't a profile row, so it's fixed here directly.
    robot_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/robot/feedback/robot_speed_mps", rclcpp::QoS(10).best_effort(),
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

    RCLCPP_INFO(get_logger(), "on_configure() succeeded. out=%dx%d quality=%d initial_mode=%d "
                "flatten_z=%s",
                out_width_, out_height_, quality_, initial_mode_, flatten_z ? "true" : "false");
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
namespace
{
// Epic 2 plan, "Diagnostics counters": WARN_THROTTLE (5 s) per adapter when
// dropped_malformed or dropped_no_tf GROWS. Rule drops (dropped_by_rule) are
// the designed steady state and never warn. One helper shared by every
// adapter row so later Epic 2 adapters get identical wording for free.
// Watermarks advance even while the throttle suppresses the print — the
// counters are cumulative, so the next growth after the window still warns.
void warn_on_drop_growth(const rclcpp::Logger& logger, rclcpp::Clock& clock,
                         const std::string& topic, const mpviz_node::AdapterStats& s,
                         uint64_t& warned_malformed, uint64_t& warned_no_tf)
{
    if (s.dropped_malformed > warned_malformed || s.dropped_no_tf > warned_no_tf)
    {
        RCLCPP_WARN_THROTTLE(logger, clock, 5000,
                             "%s: dropped %llu malformed, %llu without TF (cumulative)",
                             topic.c_str(),
                             static_cast<unsigned long long>(s.dropped_malformed),
                             static_cast<unsigned long long>(s.dropped_no_tf));
        warned_malformed = s.dropped_malformed;
        warned_no_tf = s.dropped_no_tf;
    }
}
}  // namespace

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

    // Epic 2 Task 2 (VM-024): merge every hd_map adapter's current geometry
    // into one SceneAssembly, THEN point the frozen SceneGraph at it --
    // scene_asm_.clear() must run before any adapter's fill(), or last
    // tick's elements pile up on top of this tick's (SceneAssembly's own
    // header, "ClearBetweenTicksDoesNotAccumulate"). STATED DEVIATION
    // (epic2 plan, "Staleness"): MapElement carries no last_update_sec, so
    // the library can't fade this category -- past a row's timeout_sec the
    // node just stops calling fill() for it, a pop rather than a fade (see
    // adapters/hd_map.hpp's own header comment).
    scene_asm_.clear();
    for (auto& hr : hd_map_rows_)
    {
        warn_on_drop_growth(get_logger(), *get_clock(), hr.topic, hr.adapter->stats(),
                            hr.warned_malformed, hr.warned_no_tf);
        if (sim_clock_sec_ - hr.adapter->stats().last_msg_sec > hr.timeout_sec) continue;
        hr.adapter->fill(scene_asm_);
    }

    // Epic 2 Task 3 (VM-021): same "stop filling past timeout_sec" rule as
    // hd_map above, except TrackedObject DOES carry last_update_sec (unlike
    // MapElement), so this category gets the library's staleness FADE
    // instead of hd_map's pop -- Task 4 wires clay_translucent.mat to it,
    // this adapter just has to keep publishing right up to timeout_sec.
    for (auto& dr : dynamic_objects_rows_)
    {
        // Review fix (VM-021 gate): a topic that has NEVER published is
        // absent, not stale -- without this, last_msg_sec==0 makes
        // dropped_stale tick at ~30 Hz from startup on a silent topic and
        // VM-034 would report data loss on data that never existed.
        if (dr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), dr.topic, dr.adapter->stats(),
                            dr.warned_malformed, dr.warned_no_tf);
        if (sim_clock_sec_ - dr.adapter->stats().last_msg_sec > dr.timeout_sec)
        {
            dr.adapter->mark_stale_tick();
            continue;
        }
        dr.adapter->fill(scene_asm_);
    }

    // Epic 2 Task 5 (VM-023): same "absent row never counts as stale" /
    // "stop filling past timeout_sec, mark_stale_tick() instead" shape as
    // dynamic_objects above -- PathRibbon carries last_update_sec, so the
    // library fades it rather than popping.
    for (auto& pr : path_rows_)
    {
        if (pr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), pr.topic, pr.adapter->stats(),
                            pr.warned_malformed, pr.warned_no_tf);
        if (sim_clock_sec_ - pr.adapter->stats().last_msg_sec > pr.timeout_sec)
        {
            pr.adapter->mark_stale_tick();
            continue;
        }
        pr.adapter->fill(scene_asm_);
    }

    // Epic 2 Task 6 (VM-025): same "absent row never counts as stale" /
    // "stop filling past timeout_sec, mark_stale_tick() instead" shape as
    // dynamic_objects/path above -- GroundGridLayer carries last_update_sec,
    // so the library fades it (ground_grid.mat's own alpha) rather than
    // popping. stats().msgs counts BOTH ingest()/ingest_update() overloads'
    // accepted messages (ogm.hpp's own stated decision), so a row that has
    // only ever received _updates patches (impossible in practice --
    // ingest_update() before any ingest() is a no-op, see
    // UpdateBeforeAnyFullGridIsDroppedAndCounted) would still correctly
    // read as "never produced a renderable grid" via fill() emitting
    // nothing, not via this msgs==0 gate.
    for (auto& gr : ogm_rows_)
    {
        if (gr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), gr.topic, gr.adapter->stats(),
                            gr.warned_malformed, gr.warned_no_tf);
        if (sim_clock_sec_ - gr.adapter->stats().last_msg_sec > gr.timeout_sec)
        {
            gr.adapter->mark_stale_tick();
            continue;
        }
        gr.adapter->fill(scene_asm_);
    }

    mpviz::SceneGraph scene{};
    scene.sim_time_sec = sim_clock_sec_;
    scene.ego = tf_adapter_->update();
    scene_asm_.point_at(scene);
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
    hd_map_subs_.clear();
    hd_map_rows_.clear();
    dynamic_objects_subs_.clear();
    dynamic_objects_rows_.clear();
    path_subs_.clear();
    path_rows_.clear();
    ogm_grid_subs_.clear();
    ogm_update_subs_.clear();
    ogm_rows_.clear();
    frame_transformer_.reset();
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
    hd_map_subs_.clear();
    hd_map_rows_.clear();
    dynamic_objects_subs_.clear();
    dynamic_objects_rows_.clear();
    path_subs_.clear();
    path_rows_.clear();
    ogm_grid_subs_.clear();
    ogm_update_subs_.clear();
    ogm_rows_.clear();
    frame_transformer_.reset();
    tf_adapter_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::visualization_app
