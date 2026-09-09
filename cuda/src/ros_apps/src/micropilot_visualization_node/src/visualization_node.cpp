/** @file visualization_node.cpp
 *  @brief ROS2 LifecycleNode wrapping micropilot::visualization (Filament).
 */

#include "micropilot_visualization_node/visualization_node.hpp"

#include <chrono>
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

    // Both nodes default to the same mode so exactly one publisher is active
    // from the first frame (spec §3.1 "race handling").
    initial_mode_ = declare_parameter<int>("initial_mode", 1);
    if (initial_mode_ < 1 || initial_mode_ > 3)
    {
        RCLCPP_ERROR(get_logger(), "initial_mode must be 1, 2, or 3, got %d", initial_mode_);
        return CallbackReturn::FAILURE;
    }
    active_mode_ = initial_mode_;

    // [eye xyz | target xyz], matches mpviz::CameraPose's own layout.
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
    // 80°, not the CUDA node's 60° -- at this pose's ~34° downward pitch, 60°
    // puts no sky above the horizon.
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

    // create_renderer() silently falls back to its compiled-in theme if
    // theme_assets_dir/initial_theme fails to load (non-fatal by design);
    // config.theme_assets_dir is null here since this node exposes no such
    // parameter yet, so the dir actually tried is the library's own default.
    if (!mpviz::theme_assets_loaded(renderer_))
    {
        RCLCPP_WARN(get_logger(),
                    "theme assets failed to load from '%s' -- rendering with the "
                    "compiled-in fallback theme instead",
                    config.theme_assets_dir ? config.theme_assets_dir
                                             : "<compiled-in default theme dir>");
    }

    // ── profile YAML loader ───────────────────────────────────────────────────
    // Drives which adapters subscribe to what, via mpviz_node::subscriptions_for(row)
    // over profile->rows. Failure to load is fatal; every collected error is
    // logged, not just the first, so a config file with several mistakes
    // takes one edit pass, not several.
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
    // load_profile() can succeed with non-fatal warnings too; log those here
    // or the diagnostic naming a config typo is computed and silently dropped.
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

    // Mirrors micropilot_rendering_node's robot_model_path convention: "" is a
    // legal default, load failure is non-fatal (set_ego_model() falls back to
    // a themed clay box at fallback_dims; the WARN below is informational only).
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

    // HUD compositor font (VM-030 Step 2); missing/unloadable is non-fatal
    // (CompositeHud() in timer_callback() WARNs once, HUD just isn't drawn).
    hud_font_path_ = declare_parameter<std::string>("hud_font_path", "");
    // HUD disable knob (STANDING directive, visual-mode-epic3.md): default
    // true, unrelated to hud_font_path_ -- an empty/unloadable font path is
    // an accidental side effect that still calls CompositeHud() every tick;
    // this is the actual specified switch.
    hud_enabled_ = declare_parameter<bool>("hud_enabled", true);
    // Alert-callout disable knob (Epic 3 Task 4 / VM-031, STANDING
    // directive): default true, same "real disable knob, not a font-path
    // side effect" shape as hud_enabled_ above.
    callouts_enabled_ = declare_parameter<bool>("callouts_enabled", true);

    // map->base_link -> SceneGraph.ego, finite-differenced + EMA-smoothed
    // speed. Buffer/TransformListener live on the node (need its
    // NodeInterfaces to construct); TfAdapter wraps the lookup + smoothing on top.
    auto ego_speed_smoothing_alpha = declare_parameter<double>("ego_speed_smoothing_alpha", 0.2);
    // HD-map layer is a 2D plane today, so real z (live TF altitude,
    // dynamic-object bbox centers) would render as floating geometry -- see
    // frame_transform.hpp/tf_adapter.hpp. Set false once HD-map gains real 3D.
    auto flatten_z = declare_parameter<bool>("flatten_z", true);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_adapter_ = std::make_unique<TfAdapter>(*tf_buffer_, "map", "base_link",
                                              ego_speed_smoothing_alpha, flatten_z);
    pub_ego_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/ego_state", 1);

    // ── HD-map adapters ───────────────────────────────────────────────────────
    // One HdMapAdapter per profile row with adapter: hd_map. fill() APPENDS
    // into scene_asm_ every tick, never assigns, so every matching row renders
    // together.
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

    // ── Dynamic objects ───────────────────────────────────────────────────────
    // class_inference_ must be loaded before any DynamicObjectsAdapter is
    // constructed -- adapters hold a reference to it for the lifetime of this
    // configure/activate cycle. Failure to load is fatal, same as a bad profile.
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

    // ── Path ribbons ──────────────────────────────────────────────────────────
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

    // ── OGM ground grids ──────────────────────────────────────────────────────
    // Unlike every other loop above, subscriptions_for(row) returns TWO specs
    // for adapter: ogm (base + update topic), each bound to a different
    // ingest() overload on the same adapter. transient_local never applies to
    // the update stream (it's inherently VOLATILE); best_effort still does.
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

    // ── Collision alert polygons ──────────────────────────────────────────────
    // FIXTURE GAP: all five collision topics were silent in the recorded bag
    // -- unvalidated against a live publisher.
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "collision") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<mpviz_node::CollisionAdapter>(row, *frame_transformer_);
        mpviz_node::CollisionAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        collision_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));
        collision_rows_.push_back(CollisionRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "collision: %zu row(s) subscribed", collision_rows_.size());

    // ── Generic marker fallback ───────────────────────────────────────────────
    // Adding a topic is one YAML row, no code change (spec §7 parity guarantee).
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "generic") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<mpviz_node::GenericMarkerAdapter>(row, *frame_transformer_);
        mpviz_node::GenericMarkerAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        generic_marker_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));
        generic_marker_rows_.push_back(GenericMarkerRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "generic: %zu row(s) subscribed", generic_marker_rows_.size());

    // ── TF-axes debug layer ───────────────────────────────────────────────────
    // PRODUCER, not a subscriber (subscriptions_for() returns {} for this
    // adapter). Both shipped profiles carry the row commented out. Takes the
    // node's own tf_buffer_ directly, not frame_transformer_ (see this
    // adapter's own header comment).
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "tf_axes") continue;
        tf_axes_rows_.push_back(std::make_unique<mpviz_node::TfAxesAdapter>(row, *tf_buffer_));
    }
    RCLCPP_INFO(get_logger(), "tf_axes: %zu row(s) configured", tf_axes_rows_.size());

    // Ego speed PREFERS this topic over the TF finite-difference fallback
    // computed above. Global (not "~/..."): the robot's own feedback,
    // published once regardless of which mux mode/node is active.
    // Must be best_effort(): the bag records this publisher as BEST_EFFORT; a
    // bare `10` here defaults to RELIABLE and never matches, leaving the
    // topic permanently (and silently) unsubscribed. Same root cause as the
    // profile `best_effort` field (see urban_profile.yaml); this subscription
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
    // Per-topic age/drop counters + render_ms, published every tick regardless
    // of mode.
    pub_diagnostics_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 1);

    // ── virtual-camera presets / tween ────────────────────────────────────────
    // Constructed AFTER every failure gate above -- a failed configure must
    // not leave a live vcam control surface advertised.
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

    // ── theme control ─────────────────────────────────────────────────────────
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
    pub_diagnostics_->on_activate();

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(33ms, [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(), "on_activate() succeeded.");
    return CallbackReturn::SUCCESS;
}

// ── Timer callback ───────────────────────────────────────────────────────────
namespace
{
// WARN_THROTTLE (5s) when dropped_malformed/dropped_no_tf grows; dropped_by_rule
// is the designed steady state and never warns. Watermarks advance even while
// throttled, since the counters are cumulative, so the next growth still warns.
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
    // Publish vcam telemetry BEFORE the mode gate below (mirrors
    // rendering_node) so external UIs keep getting pose updates while inactive.
    vcam_->advance_tween();
    pose_ = vcam_->pose();

    std_msgs::msg::Float64MultiArray state;
    state.data = {pose_.eye[0],    pose_.eye[1],    pose_.eye[2],
                  pose_.target[0], pose_.target[1], pose_.target[2],
                  static_cast<double>(vcam_->active_preset()),
                  static_cast<double>(active_mode_)};
    pub_vcam_state_->publish(state);

    // Runs every tick regardless of mode, before the mode gate below. Without
    // this, SceneBuffer::active().sim_time_sec never advances and a
    // ~/set_theme request never visibly finishes.
    sim_clock_sec_ += kTimerPeriodSec;

    // scene_asm_.clear() must run before any adapter's fill(), or last tick's
    // elements pile up on top of this tick's. MapElement fades via the
    // library's shared staleness handling (see kStaleFadeTimeoutSec in
    // cuda/src/libs/visual_renderer/src/renderer_internal.hpp); timeout_sec
    // below is the separate hard cutoff.
    scene_asm_.clear();
    for (auto& hr : hd_map_rows_)
    {
        warn_on_drop_growth(get_logger(), *get_clock(), hr.topic, hr.adapter->stats(),
                            hr.warned_malformed, hr.warned_no_tf);
        if (sim_clock_sec_ - hr.adapter->stats().last_msg_sec > hr.timeout_sec) continue;
        hr.adapter->fill(scene_asm_);
    }

    for (auto& dr : dynamic_objects_rows_)
    {
        // A topic that has never published is absent, not stale -- without
        // this, last_msg_sec==0 would report data loss on data that never existed.
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

    // stats().msgs counts BOTH ingest()/ingest_update() overloads' accepted
    // messages (see ogm.hpp); ingest_update() before any ingest() is a no-op,
    // so a row that only ever received update patches still reads as msgs==0.
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

    // msgs==0 is the expected steady state here (all five collision topics
    // are silent in the recorded bag), not an error path.
    for (auto& cr : collision_rows_)
    {
        if (cr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), cr.topic, cr.adapter->stats(),
                            cr.warned_malformed, cr.warned_no_tf);
        if (sim_clock_sec_ - cr.adapter->stats().last_msg_sec > cr.timeout_sec)
        {
            cr.adapter->mark_stale_tick();
            continue;
        }
        cr.adapter->fill(scene_asm_);
    }

    for (auto& gmr : generic_marker_rows_)
    {
        if (gmr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), gmr.topic, gmr.adapter->stats(),
                            gmr.warned_malformed, gmr.warned_no_tf);
        if (sim_clock_sec_ - gmr.adapter->stats().last_msg_sec > gmr.timeout_sec)
        {
            gmr.adapter->mark_stale_tick();
            continue;
        }
        gmr.adapter->fill(scene_asm_);
    }

    // No timeout gate: a live tf2 walk, not message-driven; this adapter
    // stamps "now" onto every marker it emits, so it can never itself go stale.
    for (auto& axes : tf_axes_rows_)
    {
        axes->fill(scene_asm_, sim_clock_sec_);
    }

    mpviz::SceneGraph scene{};
    scene.sim_time_sec = sim_clock_sec_;
    scene.ego = tf_adapter_->update();
    // speed_mps/active_mode only -- chips/chip_count stay zero-init (Task 4
    // / VM-031 scope). See hud_overlay.hpp's own comment for why this is a
    // free function, not the two lines inlined here.
    mpviz_node::PopulateHud(scene, active_mode_);
    scene_asm_.point_at(scene);
    mpviz::set_scene(renderer_, scene);

    std_msgs::msg::Float64MultiArray ego_state;
    ego_state.data = {scene.ego.position.x,   scene.ego.position.y, scene.ego.position.z,
                      scene.ego.heading_rad,  scene.ego.speed_mps,
                      static_cast<double>(scene.ego.valid)};
    pub_ego_state_->publish(ego_state);

    // Render/readback/publish only while this node is the active mux output
    // (spec §3.1) — costs ~zero GPU otherwise. Diagnostics still publish every
    // tick regardless. render_ms_ is explicitly zeroed here, not left at
    // whatever the last mode-3 tick measured, so a diagnostics consumer never
    // mistakes a stale number for a live one.
    if (active_mode_ != 3)
    {
        render_ms_ = 0.0;
        publish_diagnostics();
        return;
    }

    // Composed HERE ONLY, right before handing the pose to the renderer --
    // pose_ itself is never touched, so orbits/presets keep adjusting the
    // offset only.
    mpviz::CameraPose render_pose = pose_;
    if (scene.ego.valid)
    {
        render_pose = compose_ego_anchored_pose(pose_, scene.ego);
    }
    // else: no TF yet -- offset pose used as an absolute world pose (keeps
    // test_vcam_contract.py and every no-TF test bit-identical).

    mpviz::FrameView view{frame_buf_.data(), static_cast<uint32_t>(out_width_),
                          static_cast<uint32_t>(out_height_)};
    // render_ms_ instrumentation wraps render_frame() without changing the
    // call; measured only in this branch (active_mode_==3).
    const auto render_start = std::chrono::steady_clock::now();
    if (!mpviz::render_frame(renderer_, render_pose, view))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "render_frame() failed");
        render_ms_ = 0.0;
        publish_diagnostics();
        return;
    }
    render_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                             render_start)
                     .count();

    // Composited in place on frame_buf_, after the render but before the
    // image message is built below -- operates on the exact bytes about to
    // be published. Non-fatal on a missing/unloadable font (hud_font_path_
    // empty counts as "unloadable" here too): CompositeHud() leaves
    // frame_buf_ untouched and this just WARNs once, same "asset load
    // failure -> non-fatal" philosophy as set_ego_model above.
    //
    // hud_enabled_ gates the whole block (disable knob, STANDING directive):
    // false means CompositeHud() is never called at all and frame_buf_ is
    // passed through untouched -- distinct from the font-path fallback
    // above, which still calls CompositeHud() every tick.
    if (hud_enabled_)
    {
        const mpviz::HudColors hud_colors = mpviz::get_hud_colors(renderer_);
        const mpviz_node::HudSnapshot hud_snapshot{scene.hud.speed_mps, scene.hud.active_mode};
        if (!mpviz_node::CompositeHud(
                frame_buf_.data(), static_cast<uint32_t>(out_width_),
                static_cast<uint32_t>(out_height_), hud_snapshot,
                mpviz_node::HudRgb{hud_colors.text_color[0], hud_colors.text_color[1],
                                   hud_colors.text_color[2]},
                mpviz_node::HudRgb{hud_colors.accent_color[0], hud_colors.accent_color[1],
                                   hud_colors.accent_color[2]},
                hud_colors.scale, hud_font_path_.c_str()) &&
            !hud_font_warned_)
        {
            RCLCPP_WARN(get_logger(),
                        "CompositeHud: failed to load/use font '%s' -- HUD not drawn this run",
                        hud_font_path_.c_str());
            hud_font_warned_ = true;
        }
    }

    // Epic 3 Task 4 (VM-031): the nearest-obstacle distance callout --
    // leader line + chip, drawn through hud_overlay's DrawLine()/DrawText()
    // primitives (extended, Task 3) onto the same frame_buf_ the HUD block
    // above already composited onto. Reuses scene.alerts (this tick's live
    // AlertPolygon list, already built by the collision-adapter rows above)
    // and scene.ego.position as the nearest-obstacle anchor source --
    // Hud::chips/chip_count stay unpopulated (scene.h's own comment next to
    // Hud::chips).
    //
    // callouts_enabled_ gates the whole block (disable knob, STANDING
    // directive): false means BuildNearestCallout() is never even called,
    // frame_buf_ passes through untouched from the HUD block above.
    // ego.valid gate (review 2026-09-09): valid==0 means "no TF yet -> ego
    // hidden, not a clay box at origin" (scene.h's contract; the render pose
    // above branches on it the same way) -- a distance measured from a
    // non-existent ego would label the frame confidently wrong.
    if (callouts_enabled_ && scene.ego.valid != 0)
    {
        mpviz_node::Callout callout{};
        if (mpviz_node::BuildNearestCallout(renderer_, scene.alerts, scene.alert_count,
                                            scene.ego.position, callout))
        {
            // Style token (STANDING directive): theme hud.accent_color,
            // reused verbatim -- the same live (possibly mid-transition)
            // color the HUD's own mode chip already draws with, not a new
            // theme.hud field just for this.
            const mpviz::HudColors hud_colors = mpviz::get_hud_colors(renderer_);
            mpviz_node::DrawCallout(
                frame_buf_.data(), static_cast<uint32_t>(out_width_),
                static_cast<uint32_t>(out_height_), callout,
                mpviz_node::HudRgb{hud_colors.accent_color[0], hud_colors.accent_color[1],
                                   hud_colors.accent_color[2]},
                hud_colors.scale, hud_font_path_.c_str());
        }
        // else: no obstacle in view this tick (no alerts, or the nearest
        // anchor is behind the camera / outside the frustum) --
        // BuildNearestCallout() already returned false; nothing drawn,
        // frame_buf_ untouched ("suppressed, not misdrawn").
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

    publish_diagnostics();
}

// Gathers every subscribed row's stats (not tf_axes_rows_, a PRODUCER with no
// stats of its own) into one DiagnosticArray. last_msg_age_sec is computed
// here, not inside diagnostics.hpp, which deliberately takes no ROS clock so
// it stays a pure, easily unit-tested data transform.
void VisualizationNode::publish_diagnostics()
{
    std::vector<mpviz_node::RowStats> rows;
    rows.reserve(hd_map_rows_.size() + dynamic_objects_rows_.size() + path_rows_.size() +
                 ogm_rows_.size() + collision_rows_.size() + generic_marker_rows_.size());

    auto append_row = [&](const std::string& topic, const mpviz_node::AdapterStats& stats,
                           double timeout_sec)
    {
        mpviz_node::RowStats rs;
        rs.topic = topic;
        rs.stats = stats;
        rs.last_msg_age_sec = sim_clock_sec_ - stats.last_msg_sec;
        rs.timeout_sec = timeout_sec;
        rows.push_back(std::move(rs));
    };

    for (const auto& hr : hd_map_rows_) append_row(hr.topic, hr.adapter->stats(), hr.timeout_sec);
    for (const auto& dr : dynamic_objects_rows_)
        append_row(dr.topic, dr.adapter->stats(), dr.timeout_sec);
    for (const auto& pr : path_rows_) append_row(pr.topic, pr.adapter->stats(), pr.timeout_sec);
    for (const auto& gr : ogm_rows_) append_row(gr.topic, gr.adapter->stats(), gr.timeout_sec);
    for (const auto& cr : collision_rows_)
        append_row(cr.topic, cr.adapter->stats(), cr.timeout_sec);
    for (const auto& gmr : generic_marker_rows_)
        append_row(gmr.topic, gmr.adapter->stats(), gmr.timeout_sec);

    auto msg = mpviz_node::BuildDiagnostics(rows, render_ms_);
    msg.header.stamp = now();
    pub_diagnostics_->publish(msg);
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
    pub_diagnostics_->on_deactivate();
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
    pub_diagnostics_.reset();
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
    collision_subs_.clear();
    collision_rows_.clear();
    generic_marker_subs_.clear();
    generic_marker_rows_.clear();
    tf_axes_rows_.clear();
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
    collision_subs_.clear();
    collision_rows_.clear();
    generic_marker_subs_.clear();
    generic_marker_rows_.clear();
    tf_axes_rows_.clear();
    frame_transformer_.reset();
    tf_adapter_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::visualization_app
