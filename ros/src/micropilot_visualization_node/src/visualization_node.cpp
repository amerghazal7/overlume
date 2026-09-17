/** @file visualization_node.cpp
 *  @brief ROS2 LifecycleNode wrapping micropilot::visualization (Filament).
 */

#include "micropilot_visualization_node/visualization_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/header.hpp>

#include "micropilot_visualization_node/ego_anchor.hpp"
#include "micropilot_visualization_node/environment_source_uri.hpp"

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

    // ── quality auto-drop governor (VM-040, see backlog Done note) ───────────
    // Opt-in (default false). Every threshold/window is its own declared
    // param (not left to QualityGovernorParams{}'s in-class defaults) so
    // `ros2 param get` shows each one.
    // governor_enabled and the five threshold/window params below are read
    // ONCE here -- `ros2 param set` updates the stored value but takes effect
    // only on the next configure/restart (they are not in on_params()'s
    // handled set; a live-retunable governor is future scope, stated rather
    // than implied).
    governor_enabled_ = declare_parameter<bool>("governor_enabled", false);
    mpviz_node::QualityGovernorParams governor_params;
    const int governor_window_size_param = declare_parameter<int>(
        "governor_window_size", static_cast<int>(governor_params.window_size));
    governor_params.drop_threshold_ms =
        declare_parameter<double>("governor_drop_threshold_ms", governor_params.drop_threshold_ms);
    governor_params.recover_threshold_ms = declare_parameter<double>(
        "governor_recover_threshold_ms", governor_params.recover_threshold_ms);
    const int governor_recover_windows_required_param = declare_parameter<int>(
        "governor_recover_windows_required",
        static_cast<int>(governor_params.recover_windows_required));
    const int governor_min_dwell_windows_param = declare_parameter<int>(
        "governor_min_dwell_windows", static_cast<int>(governor_params.min_dwell_windows));
    if (governor_params.drop_threshold_ms <= governor_params.recover_threshold_ms)
    {
        RCLCPP_ERROR(get_logger(),
                     "governor_drop_threshold_ms (%.3f) must be > "
                     "governor_recover_threshold_ms (%.3f) -- that gap IS the hysteresis",
                     governor_params.drop_threshold_ms, governor_params.recover_threshold_ms);
        return CallbackReturn::FAILURE;
    }
    // Validated as `int` BEFORE the uint32_t cast below: a negative value
    // wraps to a huge window/streak-length that never closes, so the
    // governor goes silently dead with no log -- the uint32_t field itself
    // can no longer catch that once it's been cast.
    if (governor_window_size_param < 1 || governor_recover_windows_required_param < 1 ||
        governor_min_dwell_windows_param < 1)
    {
        RCLCPP_ERROR(get_logger(),
                     "governor_window_size (%d), governor_recover_windows_required (%d) and "
                     "governor_min_dwell_windows (%d) must all be >= 1",
                     governor_window_size_param, governor_recover_windows_required_param,
                     governor_min_dwell_windows_param);
        return CallbackReturn::FAILURE;
    }
    governor_params.window_size = static_cast<uint32_t>(governor_window_size_param);
    governor_params.recover_windows_required =
        static_cast<uint32_t>(governor_recover_windows_required_param);
    governor_params.min_dwell_windows = static_cast<uint32_t>(governor_min_dwell_windows_param);
    quality_governor_ = std::make_unique<mpviz_node::QualityGovernor>(
        governor_params, static_cast<uint32_t>(quality_));

    // Post-cutover (VM-095): the two-process race-handling this parameter
    // used to serve (spec §3.1, "both nodes default to the same mode so
    // exactly one publisher is active from the first frame") no longer
    // applies -- there is only one process. Kept as the launch-arg seed for
    // this node's own render_mode_ (below): a bare `-p initial_mode:=1`
    // still selects bowl at startup, same CLI shape every existing
    // launch/test invocation already uses, now driving the single mode
    // variable directly instead of a since-removed mux-arbitration one.
    initial_mode_ = declare_parameter<int>("initial_mode", 1);
    if (initial_mode_ < 1 || initial_mode_ > 3)
    {
        RCLCPP_ERROR(get_logger(), "initial_mode must be 1, 2, or 3, got %d", initial_mode_);
        return CallbackReturn::FAILURE;
    }

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

    // ── theme assets (VM-044) ─────────────────────────────────────────────────
    // Resolve via ament_index instead of relying on DEFAULT_THEME_ASSETS_DIR
    // (visual_renderer/CMakeLists.txt compiles THIS CHECKOUT's own absolute
    // source path into the library -- silently wrong off this dev box).
    // "" is the honest default: it means "use what this package installed",
    // not "use whatever machine happened to build the library".
    auto theme_assets_dir_param = declare_parameter<std::string>("theme_assets_dir", "");
    std::string theme_assets_dir = theme_assets_dir_param;
    if (theme_assets_dir.empty())
    {
        theme_assets_dir =
            ament_index_cpp::get_package_share_directory("micropilot_visualization_node") +
            "/assets/themes";
    }
    // Write the resolved value back so `ros2 param get theme_assets_dir`
    // reports where the node actually looked (VM-044 AC), not the "" default.
    set_parameter(rclcpp::Parameter("theme_assets_dir", theme_assets_dir));

    auto initial_theme = declare_parameter<std::string>("initial_theme", "dark_adas");
    // Validated against what this install actually shipped -- an
    // unrecognized name silently landing on the compiled-in fallback theme
    // (the same failure mode this task closes for theme_assets_dir) would be
    // a confusing regression to debug from a `ros2 param get` that then lies
    // about which theme is live.
    const std::string initial_theme_yaml = theme_assets_dir + "/" + initial_theme + ".yaml";
    if (!std::ifstream(initial_theme_yaml).good())
    {
        RCLCPP_ERROR(get_logger(),
                     "initial_theme '%s' not found under theme_assets_dir '%s' (expected '%s')",
                     initial_theme.c_str(), theme_assets_dir.c_str(), initial_theme_yaml.c_str());
        return CallbackReturn::FAILURE;
    }
    set_parameter(rclcpp::Parameter("initial_theme", initial_theme));

    // ── renderer ──────────────────────────────────────────────────────────────
    mpviz::RenderConfig config{};
    config.width = static_cast<uint32_t>(out_width_);
    config.height = static_cast<uint32_t>(out_height_);
    config.quality = static_cast<uint8_t>(quality_);
    config.theme_assets_dir = theme_assets_dir.c_str();
    config.initial_theme = initial_theme.c_str();
    renderer_ = mpviz::create_renderer(config);
    if (renderer_ == nullptr)
    {
        RCLCPP_ERROR(get_logger(), "mpviz::create_renderer() failed (no GPU/EGL?)");
        return CallbackReturn::FAILURE;
    }
    frame_buf_.assign(static_cast<size_t>(out_width_) * out_height_ * 3, 0);

    // create_renderer() copies theme_assets_dir/initial_theme into its own
    // storage (api.h's RenderConfig doc comment) -- theme_assets_loaded()
    // can still non-fatal-WARN here (e.g. a corrupt yaml on an otherwise
    // resolved path); the path named below is now the real, resolved one.
    if (!mpviz::theme_assets_loaded(renderer_))
    {
        RCLCPP_WARN(get_logger(),
                    "theme assets failed to load from '%s' -- rendering with the "
                    "compiled-in fallback theme instead",
                    theme_assets_dir.c_str());
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
    if (ego_model_path.empty())
    {
        // VM-044: resolve to the installed M02P glTF (provisioned into this
        // package's own share dir by scripts/provision_ego_model.sh) instead
        // of a per-user ~/Downloads path baked into default_params.yaml.
        // Genuinely not provisioned on this install -> stays "" and falls
        // through to the clay-box fallback below, honestly (no fabricated
        // path to a file that isn't there).
        const std::string installed_ego =
            ament_index_cpp::get_package_share_directory("micropilot_visualization_node") +
            "/assets/ego/M02P.glb";
        if (std::ifstream(installed_ego).good())
        {
            ego_model_path = installed_ego;
        }
    }
    // Resolved (or still-empty-and-honest-about-it) value visible via
    // `ros2 param get ego_model_path` (VM-044 AC), same as theme_assets_dir above.
    set_parameter(rclcpp::Parameter("ego_model_path", ego_model_path));
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
    // "" resolves via ament_index to this package's own installed
    // share/assets/fonts (VM-044), same shape as theme_assets_dir/
    // ego_model_path above, instead of a per-checkout absolute path.
    hud_font_path_ = declare_parameter<std::string>("hud_font_path", "");
    if (hud_font_path_.empty())
    {
        const std::string installed_font =
            ament_index_cpp::get_package_share_directory("micropilot_visualization_node") +
            "/assets/fonts/NotoSans-Regular.ttf";
        if (std::ifstream(installed_font).good())
        {
            hud_font_path_ = installed_font;
        }
    }
    // Resolved (or still-empty-and-honest-about-it) value visible via
    // `ros2 param get hud_font_path` (VM-044 AC), same as theme_assets_dir/
    // ego_model_path above.
    set_parameter(rclcpp::Parameter("hud_font_path", hud_font_path_));
    // HUD disable knob (STANDING directive, visual-mode-epic3.md): default
    // true, unrelated to hud_font_path_ -- an empty/unloadable font path is
    // an accidental side effect that still calls CompositeHud() every tick;
    // this is the actual specified switch.
    hud_enabled_ = declare_parameter<bool>("hud_enabled", true);
    // Alert-callout disable knob (Epic 3 Task 4 / VM-031, STANDING
    // directive): default true, same "real disable knob, not a font-path
    // side effect" shape as hud_enabled_ above.
    callouts_enabled_ = declare_parameter<bool>("callouts_enabled", true);

    // Per-category layer visibility (Epic 3 Task 5 / VM-032 Step 1, STANDING
    // directive disable-knob story for whole categories): unlike
    // hud_enabled_/callouts_enabled_ above, these are read LIVE every tick
    // (on_params() below), not just once here -- a SetParametersCallback
    // update from the GUI/WS bridge takes effect on the very next
    // timer_callback(), no restart needed. layer_point_clouds_ gates
    // scene_asm_.point_clouds as of Task 6 (VM-035) -- see the gate list in
    // timer_callback() below.
    layer_objects_ = declare_parameter<bool>("layer_objects", true);
    layer_paths_ = declare_parameter<bool>("layer_paths", true);
    layer_map_elements_ = declare_parameter<bool>("layer_map_elements", true);
    layer_grids_ = declare_parameter<bool>("layer_grids", true);
    layer_alerts_ = declare_parameter<bool>("layer_alerts", true);
    layer_markers_ = declare_parameter<bool>("layer_markers", true);
    layer_point_clouds_ = declare_parameter<bool>("layer_point_clouds", true);
    // VM-077: gates scene_asm_.trajectory_carpets -- see the gate list in
    // timer_callback() below.
    layer_trajectory_carpet_ = declare_parameter<bool>("layer_trajectory_carpet", true);

    // Local render-mode switch (Task 4 / VM-093; the ONLY mode-selection
    // mechanism post-cutover, Task 6/VM-095) -- full contract at the
    // render_mode_ field comment, visualization_node.hpp. Declare-time
    // out-of-range WARNs and clamps to FREE_LOOK; the live on_params() path
    // below rejects instead (a bad request, not a value to silently coerce).
    // Default is `initial_mode_` (already validated above), not a bare
    // literal -- a launch that only passes `-p initial_mode:=1` still
    // starts in BOWL; one that also passes `-p render_mode:=X` has that
    // value win (declare_parameter's 2nd arg is only the fallback when no
    // override was given), and `/rendering/set_mode` (below) drives this
    // same variable live thereafter.
    render_mode_ = declare_parameter<int>("render_mode", initial_mode_);
    if (render_mode_ < kRenderModeBowl || render_mode_ > kRenderModeFreeLook)
    {
        RCLCPP_WARN(get_logger(), "render_mode must be 1 (bowl), 2 (hybrid) or 3 (free_look), "
                    "got %d -- defaulting to 3 (free_look)", render_mode_);
        render_mode_ = kRenderModeFreeLook;
    }
    // Surround Stitching (Task 4/VM-093 follow-up directive) -- full contract
    // at the layer_surround_stitching_ field comment, visualization_node.hpp.
    layer_surround_stitching_ = declare_parameter<bool>("layer_surround_stitching", false);
    surround_stitching_profile_ = declare_parameter<std::string>("surround_stitching_profile",
                                                                  "bowl");
    if (surround_stitching_profile_ != "bowl" && surround_stitching_profile_ != "hybrid")
    {
        RCLCPP_WARN(get_logger(), "surround_stitching_profile must be 'bowl' or 'hybrid', got "
                    "'%s' -- defaulting to 'bowl'", surround_stitching_profile_.c_str());
        surround_stitching_profile_ = "bowl";
    }

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

    // VM-050 (Epic 4 Task 1): geo-anchor. Reuses tf_buffer_ above -- the
    // SAME buffer TfAdapter reads -- no second TransformListener.
    // gps_link == base_link (identity TF, Epic 4 plan Decision 7), so this
    // samples map->base_link directly, same frames as tf_adapter_.
    geo_anchor_solver_ = std::make_unique<GeoAnchorSolver>(*tf_buffer_, "map", "base_link");
    const double geo_datum_lat_deg = declare_parameter<double>("geo_datum_lat_deg",
                                                                std::numeric_limits<double>::quiet_NaN());
    const double geo_datum_lon_deg = declare_parameter<double>("geo_datum_lon_deg",
                                                                std::numeric_limits<double>::quiet_NaN());
    const double geo_datum_heading_deg = declare_parameter<double>(
        "geo_datum_heading_deg", std::numeric_limits<double>::quiet_NaN());
    switch (ClassifyGeoDatum(geo_datum_lat_deg, geo_datum_lon_deg, geo_datum_heading_deg))
    {
        case GeoDatumOverride::Complete:
            geo_anchor_solver_->set_override(geo_datum_lat_deg, geo_datum_lon_deg,
                                             geo_datum_heading_deg);
            // set_override() makes solved() true synchronously (Step 2) --
            // log the transition right here, same one-shot field order as
            // the on_fix()-driven log below (gps_sub_'s lambda never fires
            // this block too, since geo_anchor_logged_ is already latched).
            geo_anchor_logged_ = true;
            RCLCPP_INFO(get_logger(),
                        "geo-anchor solved (geo_datum override): --anchor-lat %.8f --anchor-lon "
                        "%.8f --anchor-heading-deg %.4f",
                        geo_datum_lat_deg, geo_datum_lon_deg, geo_datum_heading_deg);
            break;
        case GeoDatumOverride::Partial:
            // All-or-nothing (spec): a partial override is a config ERROR,
            // not a silently-applied partial anchor. Logged once at
            // startup; sampling from NavSatFix+TF proceeds as if no
            // override was given.
            RCLCPP_ERROR(get_logger(),
                        "geo_datum_lat_deg/lon_deg/heading_deg must be given all three or none "
                        "-- ignoring the partial override, sampling from NavSatFix+TF instead");
            break;
        case GeoDatumOverride::None:
            break;
    }
    // Global (not "~/..."), same reasoning as robot_speed_sub_ below --
    // the robot's own live feedback. BEST_EFFORT: the bag records this
    // publisher as BEST_EFFORT (Epic 2 Task 1's own QoS finding, same root
    // cause as robot_speed_sub_'s own comment) -- RELIABLE here would
    // silently never connect.
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/sim/feedback/gps", rclcpp::QoS(10).best_effort(),
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
        {
            geo_anchor_solver_->on_fix(*msg);
            if (!geo_anchor_logged_ && geo_anchor_solver_->solved())
            {
                geo_anchor_logged_ = true;
                const mpviz::GeoAnchor a = geo_anchor_solver_->anchor();
                // Field order matches bake_environment.py's --anchor-lat/
                // --anchor-lon/--anchor-heading-deg flags exactly (Task 2's
                // Interfaces section) -- an operator copies these three
                // numbers straight onto that script's command line.
                RCLCPP_INFO(get_logger(),
                            "geo-anchor solved: --anchor-lat %.8f --anchor-lon %.8f "
                            "--anchor-heading-deg %.4f",
                            a.origin_lat_deg, a.origin_lon_deg, a.heading_rad * 180.0 / M_PI);
            }
        });

    // ── Camera bowl ingest (VM-091, Task 2 Step 6) ───────────────────────────
    // bowl_enabled_: STANDING disable knob, read once (same shape as
    // hud_enabled_/callouts_enabled_ below) -- false means CameraIngest's
    // image callbacks never touch cv_bridge and set_bowl_config()/
    // set_camera_frame()/set_camera_motion_delta() are never called.
    // Declare-default stays OFF deliberately (bare-run safety);
    // config/default_params.yaml ships true since VM-095 Step 6.
    bowl_enabled_ = declare_parameter<bool>("bowl_enabled", false);
    const int n_cameras = declare_parameter<int>("n_cameras", 6);
    if (n_cameras <= 0 || static_cast<uint32_t>(n_cameras) > mpviz::kMaxBowlCameras)
    {
        RCLCPP_ERROR(get_logger(), "n_cameras must be in [1, %u], got %d", mpviz::kMaxBowlCameras,
                     n_cameras);
        return CallbackReturn::FAILURE;
    }
    auto image_topics =
        declare_parameter<std::vector<std::string>>("image_topics", std::vector<std::string>{});
    auto info_topics =
        declare_parameter<std::vector<std::string>>("info_topics", std::vector<std::string>{});
    // VM-091 gate close-out finding 1: a hard FAILURE here used to be the
    // only guard, and it only fired when bowl_enabled_ was already true --
    // CameraIngest's constructor (below) indexed image_topics[i]/
    // info_topics[i] for i in [0, n_cameras) UNCONDITIONALLY, regardless of
    // bowl_enabled_, so a default-params configure (bowl_enabled_ false,
    // image_topics/info_topics both the declared empty defaults, n_cameras
    // defaulting to 6) walked off the end of two empty vectors and
    // segfaulted before this check was ever consulted. Fix: validate here
    // (WARN-once + force bowl_enabled_ false, not a fatal configure) and
    // gate CONSTRUCTION of CameraIngest on bowl_enabled_ AND valid sizes
    // below, so a misconfigured/absent topic list disables the bowl instead
    // of crashing the node, and zero camera subscriptions are created
    // whenever the bowl is disabled or unconfigured.
    const bool bowl_topics_valid = static_cast<int>(image_topics.size()) == n_cameras &&
                                    static_cast<int>(info_topics.size()) == n_cameras;
    if (bowl_enabled_ && !bowl_topics_valid)
    {
        RCLCPP_WARN(get_logger(),
                    "bowl_enabled requested but image_topics/info_topics don't have n_cameras (%d) "
                    "entries (%zu/%zu) -- bowl disabled this run",
                    n_cameras, image_topics.size(), info_topics.size());
        bowl_enabled_ = false;
    }
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "");
    // Read back by the GUI bridge's config-save path; nothing in this node
    // reads it beyond that (mirrors micropilot_rendering_node's own
    // config_path param).
    declare_parameter<std::string>("config_path", "");
    max_sync_latency_ = declare_parameter<double>("max_sync_latency", 0.12);
    bowl_R0_ = declare_parameter<double>("bowl_R0", bowl_R0_);
    bowl_k_ = declare_parameter<double>("bowl_k", bowl_k_);
    bowl_Rmax_ = declare_parameter<double>("bowl_Rmax", bowl_Rmax_);
    feather_margin_ = declare_parameter<double>("feather_margin", feather_margin_);
    // Decision 3's exposure/blind-zone deferral: declared, but CLAMPED to
    // false with a WARN regardless of what the config carries (the yaml
    // default alone protects nothing -- m2o1_params.yaml ships both `true`).
    {
        const bool fbz = declare_parameter<bool>("fill_blind_zone", false);
        if (fbz)
        {
            RCLCPP_WARN(get_logger(),
                        "fill_blind_zone: true requested but forced to false -- no Filament-side "
                        "implementation this epic (Decision 3)");
        }
        fill_blind_zone_ = false;
        const bool em = declare_parameter<bool>("exposure_match", false);
        if (em)
        {
            RCLCPP_WARN(get_logger(),
                        "exposure_match: true requested but forced to false -- no Filament-side "
                        "implementation this epic (Decision 3)");
        }
        exposure_match_ = false;
    }
    auto sky = declare_parameter<std::vector<double>>(
        "sky_color", {sky_color_[0], sky_color_[1], sky_color_[2]});
    if (sky.size() == 3)
        for (int i = 0; i < 3; ++i) sky_color_[i] = static_cast<float>(sky[i]);
    bowl_exposure_compensation_ = static_cast<float>(
        declare_parameter<double>("bowl_exposure_compensation", bowl_exposure_compensation_));

    std::vector<mpviz::CameraExtrinsics> camera_extrinsics(static_cast<size_t>(n_cameras));
    if (bowl_enabled_)
    {
        // Default: identity-rotation ring around the origin (mirrors
        // micropilot_rendering_node's own default-ring fallback shape) --
        // real deployments always override this via the param.
        std::vector<double> ext_default;
        for (int i = 0; i < n_cameras; ++i)
        {
            const double angle = 2.0 * M_PI * i / n_cameras;
            const double ca = std::cos(angle), sa = std::sin(angle);
            const std::vector<double> row = {ca, -sa, 0, 0,  0,        -1,
                                             sa, ca,  0, 0.55 * ca, 0.55 * sa, 0.55};
            ext_default.insert(ext_default.end(), row.begin(), row.end());
        }
        auto ext_vec = declare_parameter<std::vector<double>>("camera_extrinsics", ext_default);
        if (static_cast<int>(ext_vec.size()) != n_cameras * 12)
        {
            RCLCPP_ERROR(get_logger(), "camera_extrinsics must have n_cameras*12 = %d floats, got %zu",
                         n_cameras * 12, ext_vec.size());
            return CallbackReturn::FAILURE;
        }
        for (int i = 0; i < n_cameras; ++i)
        {
            mpviz::CameraExtrinsics& ext = camera_extrinsics[static_cast<size_t>(i)];
            const int base = i * 12;
            for (int j = 0; j < 9; ++j) ext.R[j] = ext_vec[base + j];
            for (int j = 0; j < 3; ++j) ext.t[j] = ext_vec[base + 9 + j];
        }
    }
    else
    {
        // Declared regardless, so a later `ros2 param set bowl_enabled true`
        // + set_parameters(camera_extrinsics) sequence has somewhere to
        // land -- Task 4/6 concern, not exercised by this task's own gate.
        declare_parameter<std::vector<double>>("camera_extrinsics", std::vector<double>{});
    }
    // VM-091 gate close-out finding 1: construct CameraIngest ONLY under
    // bowl_enabled_ (already validated true only alongside valid topic-list
    // sizes above) -- when the bowl is disabled or unconfigured,
    // camera_ingest_ stays null and NO camera subscriptions are ever
    // created (every call site below already guards on
    // `bowl_enabled_ && camera_ingest_`/reset()-safety).
    if (bowl_enabled_)
    {
        camera_ingest_ = std::make_unique<CameraIngest>(this, static_cast<uint32_t>(n_cameras),
                                                         image_topics, info_topics, odom_topic,
                                                         camera_extrinsics);
        camera_ingest_->set_renderer(renderer_);
        camera_ingest_->set_bowl_enabled(bowl_enabled_);
        camera_ingest_->set_max_sync_latency(max_sync_latency_);
    }

    // ── Hybrid lidar colorization (VM-094, unified-engine migration Task 5) ──
    // hybrid_enabled_: STANDING disable knob (same shape as bowl_enabled_
    // above) -- false means no PointCloud2 subscription is created at all
    // and timer_callback()'s HYBRID block never runs (HYBRID mode falls
    // back to bowl-only, Decision 5). Real colorization also needs
    // camera_ingest_ (bowl_enabled_ true) -- ColorizeFromCameras() has no
    // camera rig geometry or RGB buffers to sample without it; that gate
    // lives in timer_callback(), not here, so the subscription can still be
    // created independently (harmless if bowl_enabled_ is false: the topic
    // is buffered but never colorized).
    // Declare-default stays OFF deliberately (bare-run safety);
    // config/default_params.yaml ships true since VM-095 Step 6.
    hybrid_enabled_ = declare_parameter<bool>("hybrid_enabled", false);
    if (camera_ingest_) camera_ingest_->set_hybrid_enabled(hybrid_enabled_);
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "");
    auto pc_tf = declare_parameter<std::vector<double>>(
        "pointcloud_transform", {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0});
    // Validated only when the hybrid feed is on -- same convention as the
    // bowl's own camera_extrinsics check living inside if (bowl_enabled_):
    // a disabled feature's malformed param must not refuse configure.
    if (hybrid_enabled_)
    {
        if (pc_tf.size() != 12)
        {
            RCLCPP_ERROR(get_logger(),
                         "pointcloud_transform must be 12 floats [R(9)|t(3)], got %zu",
                         pc_tf.size());
            return CallbackReturn::FAILURE;
        }
        for (int i = 0; i < 12; ++i) pointcloud_tf_[i] = static_cast<float>(pc_tf[i]);
    }
    // VESTIGIAL (Decision 5): declared for GUI/TUNABLE_PARAMS compatibility
    // only -- point size is owned by the theme token
    // point_cloud.point_size_px, never read by this node's own code. See
    // default_params.yaml's own comment on this key for the full reasoning.
    declare_parameter<int>("splat_radius", 2);
    if (hybrid_enabled_ && !pointcloud_topic_.empty())
    {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {
                // Skip the full ~155k-point transform in every mode
                // that never samples cloud_pts_rig_ (BOWL, or FREE_LOOK
                // without Surround Stitching's hybrid profile) and when the
                // camera ingest that would color it doesn't exist;
                // render_mode_ is live-mutable, so this tracks a mode
                // switch on the very next message.
                if (!camera_ingest_ || !hybrid_cloud_consumed()) return;
                // Locate float32 x/y/z field offsets -- same layout-
                // agnostic scan as micropilot_rendering_node's own lidar
                // callback (rendering_node.cpp:479-486).
                int ox = -1, oy = -1, oz = -1;
                for (const auto& f : msg->fields)
                {
                    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
                    if (f.name == "x") ox = static_cast<int>(f.offset);
                    else if (f.name == "y") oy = static_cast<int>(f.offset);
                    else if (f.name == "z") oz = static_cast<int>(f.offset);
                }
                if (ox < 0 || oy < 0 || oz < 0)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "hybrid: point cloud lacks float32 x/y/z fields; ignoring");
                    return;
                }
                const float* T = pointcloud_tf_;
                const size_t n = static_cast<size_t>(msg->width) * msg->height;
                std::vector<mpviz::Vec3> pts;
                pts.reserve(n);
                const uint8_t* base = msg->data.data();
                for (size_t p = 0; p < n; ++p)
                {
                    const uint8_t* rec = base + p * msg->point_step;
                    float x, y, z;
                    std::memcpy(&x, rec + ox, 4);
                    std::memcpy(&y, rec + oy, 4);
                    std::memcpy(&z, rec + oz, 4);
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                    // cloud -> rig, same [R(9)|t(3)] convention as
                    // rendering_node.cpp's own lidar callback (Decision 3's
                    // frame convention: colorization happens in rig frame).
                    pts.push_back(mpviz::Vec3{T[0] * x + T[1] * y + T[2] * z + T[9],
                                               T[3] * x + T[4] * y + T[5] * z + T[10],
                                               T[6] * x + T[7] * y + T[8] * z + T[11]});
                }
                std::lock_guard<std::mutex> lk(cloud_mtx_);
                cloud_pts_rig_.swap(pts);
            });
        RCLCPP_INFO(get_logger(), "hybrid rendering enabled (point cloud: %s)",
                    pointcloud_topic_.c_str());
    }

    // VM-052 (Epic 4 Task 3): environment disable knob + per-checkout chunks
    // dir, same read-once shape as hud_enabled_/hud_font_path_ below.
    // Actually wiring set_environment_source() happens in on_activate()
    // (geo_anchor_solver_->solved() may still be false here, at
    // on_configure() time -- sampling from NavSatFix+TF hasn't necessarily
    // finished yet).
    environment_enabled_ = declare_parameter<bool>("environment_enabled", true);
    environment_chunks_dir_ = declare_parameter<std::string>("environment_chunks_dir", "");
    // VM-063 (Epic 6 Task 4): source-selection config -- "" (shipped
    // default for both) means "keep using environment_chunks_dir_ as a
    // baked dir", byte-for-byte today's behavior. See on_activate()'s
    // composition comment for how a non-empty environment_source_uri_
    // combines with these.
    environment_source_uri_ = declare_parameter<std::string>("environment_source_uri", "");
    environment_tile_cache_dir_ = declare_parameter<std::string>("environment_tile_cache_dir", "");
    // VM-064 (Epic 6 Task 5): Google's Map Tiles terms require visible
    // attribution wherever Photorealistic 3D Tiles content is shown -- a
    // plain disable knob (STANDING directive), independent of whether this
    // deployment's environment_source_uri_ actually uses materials=original
    // (the draw site below checks that too; see docs/visual_mode/cesium.md's
    // Google section).
    environment_attribution_ = declare_parameter<bool>("environment_attribution", true);
    // VM-096 (vcam GUI Environment Tiles toggle): the deployment's own
    // "clipped" ion asset -- read once, never dereferenced by this node
    // itself (the WS bridge reads it back via get_parameters() to resolve
    // the "clipped" preset name; see default_params.yaml's own comment).
    environment_own_asset_uri_ = declare_parameter<std::string>("environment_own_asset_uri", "");

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

    // ── Point clouds ──────────────────────────────────────────────────────────
    // FIXTURE GAP: zero PointCloud2 topics exist in any recording, and no
    // shipped profile carries a live row -- unvalidated against a live
    // publisher. Same wiring shape as every other single-topic category
    // above (collision/generic).
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "point_cloud") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<mpviz_node::PointCloudAdapter>(row, *frame_transformer_);
        mpviz_node::PointCloudAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        point_cloud_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
            spec.topic, qos,
            [this, adapter_ptr](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));
        point_cloud_rows_.push_back(PointCloudRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "point_cloud: %zu row(s) subscribed", point_cloud_rows_.size());
    // hybrid_enabled_'s own cloud_sub_
    // (pointcloud_topic_, below) is a SEPARATE subscription from this
    // profile-row loop's own PointCloudAdapter subs -- a profile (e.g.
    // urban_profile.yaml) that declares a point_cloud row on the SAME topic
    // hybrid rendering also points at (a routine config overlap, not a
    // config error) leaves this node subscribed to that topic TWICE. Not
    // fixed here (two independent consumers with different per-message
    // work -- deduplicating means one feeding the other, a bigger change
    // than this finding's own severity), just surfaced once at startup.
    if (hybrid_enabled_ && !pointcloud_topic_.empty())
    {
        for (const auto& pcr : point_cloud_rows_)
        {
            if (pcr.topic == pointcloud_topic_)
            {
                RCLCPP_WARN(get_logger(),
                            "pointcloud_topic '%s' matches a profile point_cloud row -- this "
                            "node holds TWO subscriptions to it (hybrid's own cloud_sub_ plus "
                            "this profile row's PointCloudAdapter)",
                            pointcloud_topic_.c_str());
            }
        }
    }

    // ── Trajectory carpet (VM-077) ────────────────────────────────────────────
    for (const auto& row : profile->rows)
    {
        if (row.adapter != "trajectory_carpet") continue;
        const auto specs = mpviz_node::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<mpviz_node::TrajectoryCarpetAdapter>(row, *frame_transformer_);
        mpviz_node::TrajectoryCarpetAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        carpet_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            { adapter_ptr->ingest(*msg, sim_clock_sec_); }));
        carpet_rows_.push_back(CarpetRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "trajectory_carpet: %zu row(s) subscribed", carpet_rows_.size());

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
    // Same global topic names the now-decommissioned micropilot_rendering_node
    // used to publish (ADR-0002) -- kept unchanged post-cutover (ADR-0006) so
    // consumers never re-subscribe; this is now the ONLY publisher, not one
    // of two arbitrated by a mux.
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

    // ── render-mode subscription (global, not "~/...") ───────────────────────
    // Post-cutover (Task 6/VM-095 Step 2): a normal single-subscriber topic,
    // no mux semantics -- this is the ONLY thing that selects render_mode_
    // at runtime, direct assignment, no arbitration in between. Topic name/
    // type/QoS are UNCHANGED from the mux era so every existing external
    // publisher (the WS bridge, GUI, operator tooling) keeps working with no
    // edit of its own: transient_local + reliable, depth 1 (VM-037 Step
    // (a)) -- a restarted node is itself a late-joiner against the bridge's
    // durable publisher, and that durability is what resumes the operator's
    // last-published mode after a crash/lifecycle restart instead of
    // falling back to initial_mode/render_mode's own declare-time default
    // (the behavior micropilot_rendering_node/test/smoke_test.py's
    // test_restart_rejoins_live_mode hardened, now this node's alone to
    // keep).
    set_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/rendering/set_mode", rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            if (msg->data != 1 && msg->data != 2 && msg->data != 3)
            {
                RCLCPP_WARN(get_logger(), "set_mode: expected 1|2|3, got %d", msg->data);
                return;
            }
            render_mode_ = msg->data;
            RCLCPP_INFO(get_logger(), "visualization_node: render_mode -> %d", render_mode_);
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

    // Live tuning: registered AFTER all declares (same convention as
    // rendering_node.cpp's param_cb_) so it only fires on updates -- from
    // the GUI's layers checklist via the WS bridge's set_layers -> N
    // set_parameters() calls.
    layer_param_cb_ = add_on_set_parameters_callback(
        std::bind(&VisualizationNode::on_params, this, std::placeholders::_1));

    // render_mode BOWL/HYBRID with bowl_enabled_ false (the shipped default)
    // masks the whole autonomy scene for a bowl that was never configured --
    // sky + a box ego, no diagnostic. One-shot, bowl_mode_warned_ shared with
    // on_params()'s render_mode branch below.
    if ((render_mode_ == kRenderModeBowl || render_mode_ == kRenderModeHybrid) && !bowl_enabled_ &&
        !bowl_mode_warned_)
    {
        bowl_mode_warned_ = true;
        RCLCPP_WARN(get_logger(),
                    "render_mode=%d masks the whole autonomy scene but bowl_enabled is false -- "
                    "the frame will be near-empty (sky + ego) until the bowl is configured",
                    render_mode_);
    }

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

    // VM-052 (Epic 4 Task 3): baked environment chunks. Gated on the
    // disable knob AND on the geo-anchor already being solved (or
    // overridden, Step 2's set_override() path) -- solving from live
    // NavSatFix+TF takes real motion (kMinAnchorSamples @ 50 Hz,
    // geo_anchor.hpp), so a run with no geo_datum_* override typically
    // reaches here before solved() flips true; that is the stated, accepted
    // gap this step's own WARN names, not silently patched around (spec
    // §4.5/§9: "no anchor from either source -> environment layer disabled
    // with one WARN").
    //
    // VM-096 gate round 1 finding: push the configured environment_enabled_
    // into the renderer's visibility flag BEFORE arming a source below --
    // set_environment_source() syncs a newly-opened source from that flag
    // (environment_stream.cpp), so a deployment shipping environment_enabled:
    // false must have visible_ = false in place first, or a later live
    // preset switch arms a fully VISIBLE source while the GUI switch reads
    // OFF. VisualRenderer::environmentVisible defaults true, so this call is
    // a real state change on that default configuration, not a no-op.
    mpviz::set_environment_visible(renderer_, environment_enabled_);
    if (environment_enabled_ && environment_chunks_dir_.empty() && environment_source_uri_.empty())
    {
        // Neither knob configured ("" is the shipped default for both,
        // VM-044-style per-checkout gap): "not configured", not "failed" --
        // never call the entry point, and the warning names BOTH params
        // instead of reading as an open failure (VM-063: a streaming-only
        // deployment, ion:// URI with no bake, is now a supported config
        // this gate must not silently disable).
        RCLCPP_WARN(get_logger(),
                    "neither environment_chunks_dir nor environment_source_uri is set -- "
                    "environment layer disabled this run");
    }
    else if (environment_enabled_ && geo_anchor_solver_->solved())
    {
        // VM-063 Decision 5: compose the ONE source_uri string
        // set_environment_source() dispatches on. environment_source_uri_
        // empty -> environment_chunks_dir_ verbatim, byte-for-byte today's
        // behavior. Non-empty -> "<uri>[?cache=<dir>][&fallback=<dir>]" --
        // a streamed deployment's own baked bake (Epic 4's own output) is
        // automatically its fallback with zero extra config. Reachable
        // with an empty chunks dir: then no &fallback= is appended
        // (Decision 11's no-fallback path). Known ceiling (Decision 5,
        // restated here): a path containing '?' or '&' is unsupported by
        // the library's split-only parser -- not re-validated here, same
        // as every other path param in this file.
        // environment_source_uri.hpp: cache=/fallback= are appended only
        // when the configured URI doesn't already carry that key (VM-064
        // gate round 1 finding -- an explicit cache=off from the google
        // preset must not be silently overwritten by this separate dir).
        const std::string source_uri = compose_environment_source_uri(
            environment_chunks_dir_, environment_source_uri_, environment_tile_cache_dir_);
        if (!mpviz::set_environment_source(renderer_, source_uri.c_str(),
                                            geo_anchor_solver_->anchor()))
        {
            RCLCPP_WARN(get_logger(),
                        "set_environment_source: failed to open '%s' -- no buildings this run",
                        source_uri.c_str());
        }
        else
        {
            // VM-096: an honest one-shot arming log. This whole branch is
            // gated on environment_enabled_ (the `else if` above), so the
            // state word here is always "visible" -- a disabled launch
            // arms nothing at all and, because every branch of this chain
            // is gated on environment_enabled_, emits no WARN either. The
            // armed-but-HIDDEN case lives on the on_params() live-switch
            // path below, whose own log test_environment_live_switch.py's
            // check 5 actually greps.
            RCLCPP_INFO(get_logger(), "environment source armed: '%s' (visible)",
                        source_uri.c_str());
        }
        // No per-mode gate on success: see timer_callback()'s comment above
        // the bowl-visibility dispatch -- buildings render regardless of
        // render_mode_ once armed here (named exception 7, signoff.md).
    }
    else if (environment_enabled_ && !environment_warned_)
    {
        environment_warned_ = true;
        RCLCPP_WARN(get_logger(),
                    "environment_enabled but no geo-anchor solved yet at on_activate() -- "
                    "environment layer disabled this run (spec Sec.4.5/9)");
    }

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(33ms, [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(), "on_activate() succeeded.");
    return CallbackReturn::SUCCESS;
}

// ── Live parameter updates (Epic 3 Task 5 / VM-032 Step 1) ──────────────────
// The eight layer_* bools are live-tunable here. `quality` itself is still
// not wired into on_params (VM-032's open scope) -- setting it by hand via
// `ros2 param set` still takes effect only on the next restart. But since
// VM-040, mpviz::set_quality() DOES apply a preset live at the library level,
// and the quality governor (quality_governor.hpp) uses exactly that path
// every mode-3 tick, bypassing on_params entirely. Same name-match-and-assign
// shape as rendering_node.cpp's on_params(); unmatched param names fall
// through untouched (accepted, nothing to apply live).
rcl_interfaces::msg::SetParametersResult VisualizationNode::on_params(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    for (const auto& p : params)
    {
        const std::string& n = p.get_name();
        try
        {
            if (n == "layer_objects") layer_objects_ = p.as_bool();
            else if (n == "layer_paths") layer_paths_ = p.as_bool();
            else if (n == "layer_map_elements") layer_map_elements_ = p.as_bool();
            else if (n == "layer_grids") layer_grids_ = p.as_bool();
            else if (n == "layer_alerts") layer_alerts_ = p.as_bool();
            else if (n == "layer_markers") layer_markers_ = p.as_bool();
            else if (n == "layer_point_clouds") layer_point_clouds_ = p.as_bool();
            else if (n == "layer_trajectory_carpet") layer_trajectory_carpet_ = p.as_bool();
            // ── Local render-mode switch (Task 4 / VM-093) ───────────────────
            // Unlike the layer_* bools above (any bool value is valid), an
            // out-of-range render_mode is a bad REQUEST, not a value to
            // silently clamp -- reject it (res.successful=false) so the
            // caller (GUI/WS bridge/`ros2 param set`) sees the failure
            // instead of a silently-ignored mode switch.
            else if (n == "render_mode")
            {
                const int v = static_cast<int>(p.as_int());
                if (v < kRenderModeBowl || v > kRenderModeFreeLook)
                {
                    res.successful = false;
                    res.reason = "render_mode must be 1 (bowl), 2 (hybrid) or 3 (free_look)";
                }
                else
                {
                    render_mode_ = v;
                    // Same one-shot WARN as on_configure()'s close-out check
                    // -- a live switch INTO BOWL/HYBRID with the bowl never
                    // configured is the same near-empty-frame trap, just
                    // reached via `ros2 param set` instead of a declare-time
                    // default.
                    const bool bowl_ready =
                        bowl_enabled_ && camera_ingest_ && camera_ingest_->config_applied();
                    if ((render_mode_ == kRenderModeBowl || render_mode_ == kRenderModeHybrid) &&
                        !bowl_ready && !bowl_mode_warned_)
                    {
                        bowl_mode_warned_ = true;
                        RCLCPP_WARN(get_logger(),
                                    "render_mode=%d masks the whole autonomy scene but the bowl "
                                    "is not configured (bowl_enabled=%s, config_applied=%s) -- "
                                    "the frame will be near-empty (sky + ego)",
                                    render_mode_, bowl_enabled_ ? "true" : "false",
                                    (camera_ingest_ && camera_ingest_->config_applied()) ? "true"
                                                                                          : "false");
                    }
                }
            }
            else if (n == "layer_surround_stitching")
            {
                layer_surround_stitching_ = p.as_bool();
                // A silent no-op here cost a live debugging session
                // (2026-09-11): the toggle flips visibility on a bowl entity
                // that only exists once bowl_enabled + camera ingest have
                // configured one -- say so instead of doing nothing.
                if (layer_surround_stitching_ &&
                    (!camera_ingest_ || !camera_ingest_->config_applied()))
                {
                    RCLCPP_WARN(get_logger(),
                                "layer_surround_stitching enabled but no bowl is configured "
                                "(bowl_enabled=%s, camera ingest %s) -- nothing will render "
                                "until the bowl is enabled and every camera_info has arrived",
                                bowl_enabled_ ? "true" : "false",
                                camera_ingest_ ? (camera_ingest_->config_applied()
                                                      ? "configured"
                                                      : "waiting on camera_info")
                                               : "absent");
                }
            }
            else if (n == "surround_stitching_profile")
            {
                const std::string v = p.as_string();
                if (v != "bowl" && v != "hybrid")
                {
                    res.successful = false;
                    res.reason = "surround_stitching_profile must be 'bowl' or 'hybrid'";
                }
                else
                {
                    surround_stitching_profile_ = v;
                }
            }
            // ── Environment tiles (VM-096 -- vcam GUI Environment Tiles
            //    toggle, Epic 6 follow-up) ─────────────────────────────────
            // environment_enabled reuses the SAME disable knob VM-052 already
            // declared (STANDING directive: don't invent a second one) --
            // this is what makes it LIVE. It only ever toggles VISIBILITY of
            // whatever source on_activate() (or a later environment_source_uri
            // switch below) already armed; it never arms one itself, same
            // precondition as that arming path (geo-anchor solved).
            else if (n == "environment_enabled")
            {
                environment_enabled_ = p.as_bool();
                mpviz::set_environment_visible(renderer_, environment_enabled_);
                // Honest reporting (this repo's own "view: pointcloud button
                // does nothing" precedent): a toggle with no source armed
                // yet silently does nothing to any rendered frame -- say so
                // instead of letting the caller believe it took effect.
                if (mpviz::environment_source_state(renderer_) ==
                    mpviz::EnvironmentSourceState::NONE)
                {
                    RCLCPP_WARN(get_logger(),
                                "environment_enabled set to %s but no environment source is "
                                "configured yet -- nothing to show/hide this run",
                                environment_enabled_ ? "true" : "false");
                }
            }
            else if (n == "environment_source_uri")
            {
                const std::string requested = p.as_string();
                // Same precondition on_activate()'s own environment-arming
                // branch enforces (geo_anchor_solver_->solved()) -- reject/
                // WARN rather than pretend a live switch happened (HARD
                // RULE: a failed switch must leave the previous source
                // intact, never a half-state). set_environment_source()
                // itself only tears down the OLD source after the NEW one
                // opens successfully, so a rejected switch here changes
                // nothing about whatever is already rendering.
                if (!geo_anchor_solver_ || !geo_anchor_solver_->solved())
                {
                    res.successful = false;
                    res.reason = "environment_source_uri: geo-anchor not solved yet -- "
                                 "cannot switch the environment source live";
                }
                else
                {
                    const std::string source_uri = compose_environment_source_uri(
                        environment_chunks_dir_, requested, environment_tile_cache_dir_);
                    if (source_uri.empty())
                    {
                        // Finding #19: an empty requested preset (e.g. the
                        // GUI's "baked" preset) composes to "" whenever
                        // environment_chunks_dir_ is also not configured on
                        // this deployment -- reject with the real gap named
                        // instead of letting set_environment_source() fail
                        // and report a misleading "failed to open ''".
                        res.successful = false;
                        res.reason =
                            "environment_source_uri: '" + requested +
                            "' selected but environment_chunks_dir is not configured on "
                            "this deployment";
                        RCLCPP_WARN(get_logger(), "%s", res.reason.c_str());
                    }
                    else if (mpviz::set_environment_source(renderer_, source_uri.c_str(),
                                                            geo_anchor_solver_->anchor()))
                    {
                        environment_source_uri_ = requested;
                        // A freshly armed source has never fallen back --
                        // the fallback WARN latch is per ARMED SOURCE, not
                        // per-run (finding #18/#37), so a live re-arm must
                        // reset it the same way on_cleanup()/on_shutdown()
                        // do, or the next transition on this new source
                        // goes unwarned. The attribution latch is a
                        // font/DrawText-failure latch (keyed on hud_font,
                        // not on the source); it is reset here only so a
                        // preset switch to/from Google gets one fresh WARN
                        // if the credit still cannot be drawn.
                        environment_fallback_warned_ = false;
                        environment_attribution_warned_ = false;
                        // VM-096 gate round 1 finding: this live-switch arm
                        // path doesn't gate on environment_enabled_ at all
                        // (by design -- it only toggles VISIBILITY,
                        // set_environment_source() syncs the new source from
                        // r->environmentVisible itself). Same honest arming
                        // log as on_activate()'s own success path, so a
                        // switch made while environment_enabled_ is false
                        // (this session's own launch value, or a prior live
                        // toggle) is provably HIDDEN, not silently visible.
                        RCLCPP_INFO(get_logger(), "environment source armed: '%s' (%s)",
                                    source_uri.c_str(), environment_enabled_ ? "visible" : "hidden");
                    }
                    else
                    {
                        res.successful = false;
                        res.reason = "set_environment_source: failed to open '" + source_uri +
                                     "' -- previous environment source left intact";
                        RCLCPP_WARN(get_logger(), "%s", res.reason.c_str());
                    }
                }
            }
            // ── Camera bowl live tuning (VM-091 Task 2 Step 6) ───────────────
            // Same fall-through-unmatched-names-as-successful shape as the
            // rest of this handler; extends the ALREADY-registered
            // layer_param_cb_ rather than a second callback (rclcpp invokes
            // every registered callback, and a second one that doesn't fall
            // through unmatched names would reintroduce the reject-unknowns
            // bug this node's own on_params already avoids).
            // VM-091 gate close-out finding 2: each of these six params
            // used to be stored and nothing else -- timer_callback()'s bowl
            // block only re-baked on a CameraInfo change, so a live
            // set_parameters() edit here was inert until an unrelated
            // camera reconnect happened to re-bake. bowl_config_dirty_
            // makes timer_callback() re-call apply_bowl_config() on the
            // very next tick once all_info_ready() -- a full re-bake, same
            // as the CameraInfo-change path, accepted per the plan's own
            // one-dropped-frame GUI-edit budget.
            else if (n == "bowl_R0") { bowl_R0_ = p.as_double(); bowl_config_dirty_ = true; }
            else if (n == "bowl_k") { bowl_k_ = p.as_double(); bowl_config_dirty_ = true; }
            else if (n == "bowl_Rmax") { bowl_Rmax_ = p.as_double(); bowl_config_dirty_ = true; }
            else if (n == "feather_margin")
            {
                feather_margin_ = p.as_double();
                bowl_config_dirty_ = true;
            }
            else if (n == "bowl_exposure_compensation")
            {
                bowl_exposure_compensation_ = static_cast<float>(p.as_double());
                bowl_config_dirty_ = true;
            }
            else if (n == "sky_color")
            {
                auto v = p.as_double_array();
                if (v.size() == 3)
                {
                    for (int i = 0; i < 3; ++i) sky_color_[i] = static_cast<float>(v[i]);
                    bowl_config_dirty_ = true;
                }
            }
            // Decision 3's root-cause guard, restated here: a live
            // set_parameters() call (not just the initial declare) carrying
            // `true` for either is CLAMPED to false with a WARN, same as
            // on_configure()'s own declare-time clamp.
            else if (n == "fill_blind_zone")
            {
                const bool requested = p.as_bool();
                fill_blind_zone_ = false;
                if (requested)
                    RCLCPP_WARN(get_logger(),
                                "fill_blind_zone: true requested but forced to false -- no "
                                "Filament-side implementation this epic (Decision 3)");
            }
            else if (n == "exposure_match")
            {
                const bool requested = p.as_bool();
                exposure_match_ = false;
                if (requested)
                    RCLCPP_WARN(get_logger(),
                                "exposure_match: true requested but forced to false -- no "
                                "Filament-side implementation this epic (Decision 3)");
            }
            // other params: accept (stored by rclcpp) but nothing to apply live
        }
        catch (const std::exception& e)
        {
            res.successful = false;
            res.reason = std::string("bad value for ") + n + ": " + e.what();
        }
    }
    return res;
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

    // [eye xyz | target xyz | active_preset | render_mode | mux_mode] (9
    // elements -- Step (d), VM-037 appended mux_mode at index 8). Index 8
    // is a PERMANENT MIRROR of index 7 post-cutover (Decision 8, Task 6/
    // VM-095): there is no second process's mode to distinguish it from
    // any more, but the wire shape stays 9 elements -- every consumer
    // (vcam_ws_bridge.py, test_vcam_contract.py, test_ego_anchored_vcam.py)
    // only asserts `len(vcam_state) >= 9`, so shrinking the format would
    // cost more than it saves.
    std_msgs::msg::Float64MultiArray state;
    state.data = {pose_.eye[0],    pose_.eye[1],    pose_.eye[2],
                  pose_.target[0], pose_.target[1], pose_.target[2],
                  static_cast<double>(vcam_->active_preset()),
                  static_cast<double>(render_mode_),
                  static_cast<double>(render_mode_)};
    pub_vcam_state_->publish(state);

    // Runs every tick regardless of mode, before the mode gate below. Without
    // this, SceneBuffer::active().sim_time_sec never advances and a
    // ~/set_theme request never visibly finishes.
    sim_clock_sec_ += kTimerPeriodSec;

    // ── Camera bowl (VM-091, Task 2 Step 6) ───────────────────────────────────
    // Runs every tick regardless of mode -- same "ingest continues
    // regardless of mode" philosophy as sim_clock_sec_ above -- so the bowl
    // stays warm (textures uploaded, mesh baked) whenever mode switches to
    // it later (Task 4). bowl_enabled_ false means camera_ingest_ itself
    // already does nothing (its image callbacks return before touching
    // cv_bridge); this block is then also a no-op.
    if (bowl_enabled_ && camera_ingest_)
    {
        // Same BowlConfig every (re)bake -- only WHEN it's called differs
        // between the first-completion and re-bake-on-change branches below.
        auto apply_bowl_config = [&]() -> bool
        {
            std::vector<mpviz::CameraExtrinsics> ext;
            std::vector<mpviz::CameraIntrinsics> in;
            std::vector<uint32_t> w, h;
            camera_ingest_->fill_bowl_intrinsics(ext, in, w, h);
            mpviz::BowlConfig bc{};
            bc.camera_count = static_cast<uint32_t>(ext.size());
            bc.extrinsics = ext.data();
            bc.intrinsics = in.data();
            bc.cam_width = w.data();
            bc.cam_height = h.data();
            bc.bowl_R0 = bowl_R0_;
            bc.bowl_k = bowl_k_;
            bc.bowl_Rmax = bowl_Rmax_;
            bc.feather_margin = feather_margin_;
            bc.fill_blind_zone = fill_blind_zone_ ? 1 : 0;
            bc.exposure_match = exposure_match_ ? 1 : 0;
            bc.sky_color[0] = sky_color_[0];
            bc.sky_color[1] = sky_color_[1];
            bc.sky_color[2] = sky_color_[2];
            bc.exposure_compensation = bowl_exposure_compensation_;
            return mpviz::set_bowl_config(renderer_, bc);
        };

        if (!camera_ingest_->config_applied())
        {
            if (camera_ingest_->all_info_ready())
            {
                if (apply_bowl_config())
                {
                    camera_ingest_->mark_bowl_config_applied();
                    // Visibility itself is now dispatched every tick, below
                    // (Task 4/VM-093's per-mode set_bowl_visible() call) --
                    // nothing to do here beyond marking the config applied.
                    RCLCPP_INFO(get_logger(), "bowl: configured");
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "set_bowl_config() failed with all CameraInfo present");
                }
            }
        }
        else if (camera_ingest_->consume_info_dirty())
        {
            if (apply_bowl_config())
                RCLCPP_INFO(get_logger(), "bowl: re-baked (CameraInfo changed)");
            else
                RCLCPP_WARN(get_logger(), "bowl: re-bake after CameraInfo change failed");
        }
        // VM-091 gate close-out finding 2: a live bowl_R0_/bowl_k_/
        // bowl_Rmax_/feather_margin_/sky_color_/bowl_exposure_compensation_
        // edit (on_params()) re-bakes here on the very next tick -- once
        // config_applied() is true, all_info_ready() stays true forever
        // (IngestState never un-sets info_ready), so no extra readiness
        // check is needed. A full re-bake per edit, same one-dropped-frame
        // budget the plan's own set_bowl_config contract accepts.
        else if (bowl_config_dirty_)
        {
            // Cleared only on SUCCESS -- a failed re-bake keeps the request
            // pending and retries next tick instead of silently discarding
            // the operator's edit.
            if (apply_bowl_config())
            {
                bowl_config_dirty_ = false;
                RCLCPP_INFO(get_logger(), "bowl: re-baked (live param change)");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "bowl: live-param re-bake failed -- retrying");
            }
        }
        // Cheap per-tick ego-motion re-alignment (no re-bake, no texture
        // touch) -- identity deltas until config_applied()/odometry exist.
        camera_ingest_->update_motion_deltas();
    }

    // ── Bowl visibility dispatch (Task 4 / VM-093) ────────────────────────────
    // Runs every tick regardless of bowl_enabled_ (same "warm state, cheap
    // flag flip" philosophy as the bowl block above) --
    // set_bowl_visible() itself no-ops when the bowl was never configured
    // (cameraCount==0, scene.h's own contract), so this is safe even with
    // bowl_enabled_ false. Predicate lives in scene_assembly.hpp/.cpp
    // (unit-tested there instead of only smoke-tested here)
    // -- surround_stitching_profile_ picks bowl-vs-hybrid CONTENT: "bowl"
    // shows Task 2's camera-textured bowl alone; "hybrid" additionally
    // appends Task 5's camera-colorized lidar (see the HYBRID-lidar block
    // below, which now also fires for FREE_LOOK + this profile -- VM-094
    // ).
    const auto render_mode = static_cast<RenderMode>(render_mode_);
    mpviz::set_bowl_visible(renderer_, bowl_visible_for_mode(render_mode, layer_surround_stitching_));

    // Environment/buildings layer (Epic 4/VM-052) is renderer-internal, not a
    // SceneAssembly/LayerFlags category (scene_assembly.hpp's mode_content_mask
    // comment) -- NOT gated per render_mode_. Buildings are instead gated on
    // the renderer's environmentVisible flag, which mpviz::set_environment_visible()
    // drives from environment_enabled_ (on_activate() and the on_params()
    // live toggle above) -- so buildings render regardless of render_mode_
    // in BOWL/HYBRID whenever a source is armed AND environment_enabled_ is
    // true. The arming source itself is environment_source_uri_ OR
    // environment_chunks_dir_ (VM-063), either one composed by
    // compose_environment_source_uri(). This is named exception 7,
    // docs/visual_mode/signoff.md -- CLOSED (library-side blocker) by
    // VM-096's set_environment_visible(); the remaining by-design open item
    // recorded there is that there is still no automatic per-render_mode
    // gating.

    // scene_asm_.clear() must run before any adapter's fill(), or last tick's
    // elements pile up on top of this tick's. MapElement fades via the
    // library's shared staleness handling (see kStaleFadeTimeoutSec in
    // overlume/src/renderer_internal.hpp); timeout_sec
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

    // FIXTURE GAP: no PointCloud2 topic exists in any recording -- msgs==0
    // is the expected steady state today, not an error path (same
    // reasoning as the collision-row loop above).
    for (auto& pcr : point_cloud_rows_)
    {
        if (pcr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), pcr.topic, pcr.adapter->stats(),
                            pcr.warned_malformed, pcr.warned_no_tf);
        if (sim_clock_sec_ - pcr.adapter->stats().last_msg_sec > pcr.timeout_sec)
        {
            pcr.adapter->mark_stale_tick();
            continue;
        }
        pcr.adapter->fill(scene_asm_);
    }

    // VM-077: same fill()-appends/timeout_sec/warn_on_drop_growth shape as
    // every category above.
    for (auto& cr : carpet_rows_)
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
    // free function, not the two lines inlined here. Post-cutover, this
    // node's render_mode_ IS the active mode (no separate mux value).
    mpviz_node::PopulateHud(scene, render_mode_);

    // Epic 3 Task 5 (VM-032) Step 0: layer visibility is a NODE-SIDE gate,
    // not a renderer API -- clearing a category's vector right before
    // point_at() publishes it as count==0 for this tick, exactly as if no
    // adapter had ever filled it (every render path already handles the
    // empty case). layer_point_clouds_'s gate closes Task 5's forward
    // reference (Task 6 / VM-035 owns this one line + the vector it clears).
    // All seven gates -- this one included -- are exercised directly by
    // test_scene_assembly.cpp's ApplyLayerGates* cases; the other six are
    // additionally covered end-to-end by test_bridge_e2e_set_layers_hides_and_shows.
    // Velocity ribbon nests WITHIN the local ribbon (user directive
    // 2026-09-10) -- re-spine it onto the local path's own geometry so the
    // two strips are concentric instead of ~1m-offset crisscrossing edges.
    micropilot::visualization_app::respine_velocity_ribbon_onto_local_path(scene_asm_);

    // Task 4 (VM-093), USER DIRECTIVE 2026-09-11 (mode content exclusivity):
    // AND the per-mode content mask over the user's own layer_* params --
    // never overwriting layer_objects_ etc. themselves, so switching back to
    // FREE_LOOK restores exactly what the user had (compose_layer_gates()
    // reads `user`, returns a new value, mutates nothing).
    // hoisted to run BEFORE the hybrid
    // block below (it used to run after, at the tail of this function) --
    // that let the user's own layer_point_clouds_ param clear HYBRID's (and
    // Surround-Stitching-hybrid's) colorized-lidar row right back out after
    // it was pushed, silently reproducing bowl-only rendering under the
    // follow-up directive's own parity-golden procedure (c), which disables
    // every layer except Surround Stitching before a capture. Applying the
    // gate here, before that content exists, and never re-applying it later
    // this tick, makes the hybrid row immune to the autonomy layer_* gate by
    // construction -- it is CUDA-parity content, not an autonomy layer the
    // user's layer_* params are meant to toggle.
    const LayerFlags user_layer_flags{layer_objects_,       layer_paths_,
                                       layer_map_elements_,  layer_grids_,
                                       layer_alerts_,        layer_markers_,
                                       layer_point_clouds_,  layer_trajectory_carpet_};
    apply_layer_gates(scene_asm_,
                       compose_layer_gates(user_layer_flags, mode_content_mask(render_mode)));

    // ── Hybrid lidar colorization (VM-094, unified-engine migration Task 5) ──
    // USER DIRECTIVE 2026-09-11 (mode content exclusivity): HYBRID renders
    // bowl + camera-colorized lidar + ego ONLY -- this is that content, now
    // real (mode_content_mask()'s own comment used to note this category
    // carried an un-colorized autonomy PointCloud row until this task
    // landed; see that comment for the updated, honest state).
    // this block ALSO fires for FREE_LOOK
    // when Surround Stitching is on with the "hybrid" profile -- the
    // 2026-09-11 follow-up directive's own parity-golden procedure (c)
    // needs a REAL hybrid capture in mode 3 (all layers off except Surround
    // Stitching), which was inert until this fix (the gate used to be
    // HYBRID-only, so `surround_stitching_profile:=hybrid` silently
    // rendered bowl-only in FREE_LOOK). The two cases differ in how the
    // colorized cloud reaches scene_asm_.point_clouds: HYBRID still
    // clear-and-replaces the category outright (mode_content_mask(HYBRID) does NOT clear this category --
    // point_clouds is the one category its mask keeps visible, precisely so
    // this content isn't hidden -- so this clear is what actually enforces
    // exclusivity, not a belt-and-braces no-op; without it, any autonomy
    // point_cloud row the gate call above left untouched would survive
    // alongside the colorized cloud); FREE_LOOK APPENDS instead, because
    // mode 3's own PointCloudAdapter rows (VM-035, the loop above) must
    // survive here -- the clear-and-replace discipline is HYBRID-only by
    // design, not a general rule. Either way, this happens strictly AFTER
    // the gate call above and nothing re-applies that gate later this tick,
    // so the row pushed below is immune to layer_point_clouds_ regardless of
    // which branch runs.
    // hybrid_enabled_ false (shipped default) or camera_ingest_ null (bowl
    // disabled) means this block does nothing, and scene_asm_.point_clouds
    // keeps whatever mode 3's own PointCloudAdapter rows already appended.
    // `hybrid_points` is declared here (not inside the `if` below) so its
    // backing storage stays alive through scene_asm_.point_at(scene)/
    // set_scene() further down -- those pointers alias into it, same
    // aliasing-lifetime contract scene_assembly.hpp's own header comment
    // documents for every adapter.
    // camera_ingest_'s own retention copy
    // (store_rgb(), ~24 MB/full-camera-set) is meaningless outside a tick
    // that actually samples it -- re-drive it every tick from the SAME
    // condition the point cloud callback and the block below already gate
    // on, so a mode switch (BOWL <-> HYBRID <-> FREE_LOOK[+stitching])
    // stops/starts that extra per-camera-frame copy on the next image, not
    // only at on_configure() time.
    if (camera_ingest_) camera_ingest_->set_hybrid_enabled(hybrid_enabled_ && hybrid_cloud_consumed());

    // HYBRID content exclusivity holds regardless of hybrid_enabled_:
    // mode_content_mask(HYBRID) deliberately keeps point_clouds visible
    // (it is HYBRID's own category), so the autonomy rows are cleared here
    // UNCONDITIONALLY -- with the hybrid feed disabled, mode 2 shows
    // bowl+ego and an empty cloud, never the autonomy point-cloud rows.
    // Runs after the gate call above; nothing re-applies the gate later
    // this tick.
    if (render_mode == RenderMode::HYBRID) scene_asm_.point_clouds.clear();

    std::vector<mpviz::PointCloudPoint> hybrid_points;
    if (hybrid_cloud_consumed() && hybrid_enabled_ && camera_ingest_)
    {
        std::vector<mpviz::CameraExtrinsics> ext;
        std::vector<mpviz::CameraIntrinsics> in;
        std::vector<uint32_t> cw, ch;
        camera_ingest_->fill_bowl_intrinsics(ext, in, cw, ch);
        std::vector<const uint8_t*> rgb_bufs;
        camera_ingest_->fill_camera_rgb_buffers(rgb_bufs);

        mpviz::BowlConfig cams{};
        cams.camera_count = static_cast<uint32_t>(ext.size());
        cams.extrinsics = ext.data();
        cams.intrinsics = in.data();
        cams.cam_width = cw.data();
        cams.cam_height = ch.data();

        // ColorizeFromCameras only reads the cloud, and the node's single
        // executor thread already serializes this tick against the
        // PointCloud2 callback -- reading under the lock without a snapshot
        // copy is exactly as safe and pays no ~155k-point copy per tick.
        std::vector<mpviz::PointCloudPoint> colorized;
        size_t cloud_input_count = 0;
        {
            std::lock_guard<std::mutex> lk(cloud_mtx_);
            cloud_input_count = cloud_pts_rig_.size();
            colorized = ColorizeFromCameras(cloud_pts_rig_, cams, rgb_bufs);
        }
        hybrid_points = std::move(colorized);
        // Coverage is measured, not guessed: first-match drops any lidar
        // point no configured camera's frustum covers (lidar_colorize.cpp),
        // so this is usually well under 100%.
        if (cloud_input_count > 0)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000, "hybrid: colorized %zu/%zu lidar points (%.1f%% coverage)",
                hybrid_points.size(), cloud_input_count,
                100.0 * static_cast<double>(hybrid_points.size()) / static_cast<double>(cloud_input_count));
        }

        // rig -> map: same yaw-rotate-then-translate convention as
        // ego_anchor.hpp's compose_ego_anchored_pose (Decision 3's frame
        // convention -- colorize in rig frame, anchor by the SAME ego pose
        // this tick feeds set_scene() below via `scene.ego`, set above).
        if (scene.ego.valid)
        {
            const double c = std::cos(scene.ego.heading_rad);
            const double s = std::sin(scene.ego.heading_rad);
            for (auto& p : hybrid_points)
            {
                const double x = p.position.x, y = p.position.y, z = p.position.z;
                p.position.x = x * c - y * s + scene.ego.position.x;
                p.position.y = x * s + y * c + scene.ego.position.y;
                p.position.z = z + scene.ego.position.z;
            }
        }
        else
        {
            // No valid ego pose this tick -- don't anchor lidar points at a
            // garbage (identity) origin; same "ego.valid==0 hides content
            // rather than mis-placing it" convention ego.cpp's own
            // update_ego_transform() uses for the ego mesh itself.
            hybrid_points.clear();
        }

        // HYBRID's autonomy rows were already cleared unconditionally
        // above; here the colorized cloud is APPENDED, which in
        // FREE_LOOK+Surround-Stitching-hybrid lets mode 3's own
        // PointCloudAdapter rows survive alongside it.
        if (!hybrid_points.empty())
        {
            mpviz::PointCloud row{};
            row.points = hybrid_points.data();
            row.point_count = static_cast<uint32_t>(hybrid_points.size());
            row.last_update_sec = sim_clock_sec_;
            scene_asm_.point_clouds.push_back(row);
        }
    }

    scene_asm_.point_at(scene);
    mpviz::set_scene(renderer_, scene);

    std_msgs::msg::Float64MultiArray ego_state;
    ego_state.data = {scene.ego.position.x,   scene.ego.position.y, scene.ego.position.z,
                      scene.ego.heading_rad,  scene.ego.speed_mps,
                      static_cast<double>(scene.ego.valid)};
    pub_ego_state_->publish(ego_state);

    // Post-cutover (Task 6/VM-095 Step 3): the mux-arbitration early return
    // that used to gate render/readback/publish on `active_mode_ != 3` --
    // i.e. only actually rendering while this node was the mux-selected
    // free-look renderer, deferring modes 1/2 to micropilot_rendering_node
    // (Decision 7) -- is DELETED. This node now renders and publishes every
    // tick regardless of render_mode_; it is the sole rendering authority
    // for all three modes, not one of two arbitrated by a mux.

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
    // call; measured every tick now (post-cutover, this branch always runs).
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

    // ── environment source health poll (VM-063, Decision 11) ────────────────
    // One virtual call + integer compare, safe every tick. The library
    // can't WARN itself (POD-boundary logging convention) and network loss
    // happens mid-run, long after on_activate()'s set_environment_source()
    // call returned true -- this is the node's only window into it. Bool
    // latch, not _ONCE sugar (the Epic 4 Task 1 Step 4 precedent,
    // geo_anchor_logged_): fires exactly once on the STREAMING ->
    // STREAMING_FALLBACK transition, never again for this armed source
    // (one-way, Decision 11 -- no auto-recovery re-arms it on its own; a
    // live environment_source_uri switch resets the latch, finding #18/#37).
    if (!environment_fallback_warned_ &&
        mpviz::environment_source_state(renderer_) ==
            mpviz::EnvironmentSourceState::STREAMING_FALLBACK)
    {
        environment_fallback_warned_ = true;
        // Finding #17: the fallback dir actually in effect is whatever
        // compose_environment_source_uri() put in the composed URI's
        // `fallback=` key -- an inline fallback= on environment_source_uri_
        // always wins over environment_chunks_dir_ (that function's own
        // comment), so name THAT dir, not environment_chunks_dir_ alone, or
        // this WARN can claim "none configured" while a real fallback bake
        // is rendering.
        const std::string fallback_dir = fallback_dir_from_source_uri(compose_environment_source_uri(
            environment_chunks_dir_, environment_source_uri_, environment_tile_cache_dir_));
        if (fallback_dir.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "environment source: network loss detected -- switched to fallback, "
                        "but none configured -- environment now empty");
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "environment source: network loss detected -- switched to fallback dir '%s'",
                        fallback_dir.c_str());
        }
    }

    // ── quality auto-drop governor (VM-040, see backlog Done note) ───────────
    // Opt-in. On a transition, applies it live via mpviz::set_quality() and
    // mirrors quality_ onto the ROS param too.
    if (governor_enabled_)
    {
        const mpviz_node::QualityTransition transition =
            quality_governor_->record_render_ms(render_ms_);
        if (transition != mpviz_node::QualityTransition::NONE)
        {
            const uint32_t new_preset = quality_governor_->current_preset();
            mpviz::set_quality(renderer_, new_preset);
            quality_ = static_cast<int>(new_preset);
            // So `ros2 param get quality` reflects the governor's own change.
            set_parameter(rclcpp::Parameter("quality", quality_));
            RCLCPP_WARN(get_logger(), "quality governor: %s -> preset %u (render_ms p95 over window)",
                        transition == mpviz_node::QualityTransition::DROPPED ? "DROPPED" : "RECOVERED",
                        new_preset);
        }
    }

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
    // above, which still calls CompositeHud() every tick. Task 4/VM-093,
    // USER DIRECTIVE 2026-09-11: BOWL/HYBRID never had a HUD in the CUDA
    // reference (it's a mode-3-only Filament-node addition), so
    // overlays_visible_for_mode() (scene_assembly.hpp) force-suppresses it
    // there regardless of hud_enabled_ -- never touching hud_enabled_
    // itself, so it's restored exactly on returning to FREE_LOOK.
    if (hud_enabled_ && overlays_visible_for_mode(render_mode))
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

    // VM-064 (Epic 6 Task 5) Step 2(a): Google Photorealistic 3D Tiles
    // attribution. Google's Map Tiles terms require this to be visible
    // whenever tiles are displayed -- drawn through the SAME DrawText()
    // primitive callouts already share (no new compositor), independent of
    // hud_enabled_/render_mode_ (buildings themselves render regardless of
    // render_mode_ once armed, on_activate()'s own comment above -- the
    // attribution notice follows that same "wherever the imagery shows"
    // rule, not the HUD's per-mode suppression).
    //
    // environment_attribution_ alone does not draw anything: this ALSO
    // requires environment_source_uri_ to actually be running in
    // materials=original mode (Decision 14) -- a plain OSM-clay/clipped-clay
    // preset draws nothing here even with the knob left on its default --
    // AND the source to actually be STREAMING right now. The state conjunct
    // is what keeps the notice honest: a source that has fallen back to
    // baked chunks (STREAMING_FALLBACK, one-way per Decision 11) shows NO
    // Google imagery and may not carry its credit.
    //
    // VM-096 gate round 1 finding: STREAMING alone is not enough any more --
    // environment_enabled_ now means "visible" (set_environment_visible()),
    // not just "armed", so a hidden-but-still-STREAMING source (the GUI
    // toggle switched off) must also be excluded here, or this line draws
    // attribution over frames with no Google imagery on screen.
    if (environment_enabled_ && environment_attribution_ &&
        environment_source_uri_.find("materials=original") != std::string::npos &&
        mpviz::environment_source_state(renderer_) == mpviz::EnvironmentSourceState::STREAMING)
    {
        const mpviz::HudColors hud_colors = mpviz::get_hud_colors(renderer_);
        // ponytail: a static compliance line, not ion's own live per-tile
        // credits (cesium-native's CreditSystem) -- see
        // docs/visual_mode/cesium.md's Google section for the
        // verify-at-implementation note on confirming/updating this exact
        // wording for a given deployment's ion asset before go-live.
        if (!mpviz_node::DrawText(
                frame_buf_.data(), static_cast<uint32_t>(out_width_),
                static_cast<uint32_t>(out_height_), "3D Tiles data (c) Google", 8.0f,
                static_cast<float>(out_height_) - 8.0f,
                mpviz_node::HudRgb{hud_colors.text_color[0], hud_colors.text_color[1],
                                   hud_colors.text_color[2]},
                hud_colors.scale, hud_font_path_.c_str()) &&
            !environment_attribution_warned_)
        {
            RCLCPP_WARN(get_logger(),
                        "environment attribution: failed to load/use font '%s' -- "
                        "Google attribution not drawn this run",
                        hud_font_path_.c_str());
            environment_attribution_warned_ = true;
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
    // Task 4/VM-093, USER DIRECTIVE 2026-09-11: same force-suppression as
    // the HUD block above (overlays_visible_for_mode()) -- BOWL/HYBRID never
    // had a callout in the CUDA reference, and callouts_enabled_ itself is
    // left untouched.
    if (callouts_enabled_ && overlays_visible_for_mode(render_mode) && scene.ego.valid != 0)
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
                 ogm_rows_.size() + collision_rows_.size() + generic_marker_rows_.size() +
                 point_cloud_rows_.size() + carpet_rows_.size());

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
    for (const auto& pcr : point_cloud_rows_)
        append_row(pcr.topic, pcr.adapter->stats(), pcr.timeout_sec);
    for (const auto& cr : carpet_rows_) append_row(cr.topic, cr.adapter->stats(), cr.timeout_sec);

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
    gps_sub_.reset();
    camera_ingest_.reset();
    geo_anchor_solver_.reset();
    geo_anchor_logged_ = false;
    environment_warned_ = false;
    environment_fallback_warned_ = false;
    environment_attribution_warned_ = false;
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
    point_cloud_subs_.clear();
    point_cloud_rows_.clear();
    carpet_subs_.clear();
    carpet_rows_.clear();
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
    gps_sub_.reset();
    camera_ingest_.reset();
    geo_anchor_solver_.reset();
    geo_anchor_logged_ = false;
    environment_warned_ = false;
    environment_fallback_warned_ = false;
    environment_attribution_warned_ = false;
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
    point_cloud_subs_.clear();
    point_cloud_rows_.clear();
    carpet_subs_.clear();
    carpet_rows_.clear();
    tf_axes_rows_.clear();
    frame_transformer_.reset();
    tf_adapter_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::visualization_app
