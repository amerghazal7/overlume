// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/overlume_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/header.hpp>

#include "overlume_ros/ego_anchor.hpp"
#include "overlume_ros/environment_source_uri.hpp"

namespace overlume::ros {

OverlumeNode::OverlumeNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("overlume_node", options) {
    RCLCPP_INFO(get_logger(), "OverlumeNode constructed — awaiting configure transition.");
}

OverlumeNode::~OverlumeNode() { destroy_renderer_if_any(); }

void OverlumeNode::destroy_renderer_if_any() {
    if (renderer_ != nullptr) {
        overlume::destroy_renderer(renderer_);
        renderer_ = nullptr;
    }
}

OverlumeNode::CallbackReturn OverlumeNode::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "on_configure() called.");

    out_width_ = declare_parameter<int>("out_width", 1280);
    out_height_ = declare_parameter<int>("out_height", 720);
    quality_ = declare_parameter<int>("quality", 1);
    if (quality_ < 0 || quality_ > 2) {
        RCLCPP_ERROR(get_logger(), "quality must be 0 (low), 1 (med), or 2 (high), got %d",
                     quality_);
        return CallbackReturn::FAILURE;
    }

    governor_enabled_ = declare_parameter<bool>("governor_enabled", false);
    overlume::ros::QualityGovernorParams governor_params;
    const int governor_window_size_param = declare_parameter<int>(
        "governor_window_size", static_cast<int>(governor_params.window_size));
    governor_params.drop_threshold_ms =
        declare_parameter<double>("governor_drop_threshold_ms", governor_params.drop_threshold_ms);
    governor_params.recover_threshold_ms = declare_parameter<double>(
        "governor_recover_threshold_ms", governor_params.recover_threshold_ms);
    const int governor_recover_windows_required_param =
        declare_parameter<int>("governor_recover_windows_required",
                               static_cast<int>(governor_params.recover_windows_required));
    const int governor_min_dwell_windows_param = declare_parameter<int>(
        "governor_min_dwell_windows", static_cast<int>(governor_params.min_dwell_windows));
    if (governor_params.drop_threshold_ms <= governor_params.recover_threshold_ms) {
        RCLCPP_ERROR(get_logger(),
                     "governor_drop_threshold_ms (%.3f) must be > "
                     "governor_recover_threshold_ms (%.3f) -- that gap IS the hysteresis",
                     governor_params.drop_threshold_ms, governor_params.recover_threshold_ms);
        return CallbackReturn::FAILURE;
    }
    if (governor_window_size_param < 1 || governor_recover_windows_required_param < 1 ||
        governor_min_dwell_windows_param < 1) {
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
    quality_governor_ = std::make_unique<overlume::ros::QualityGovernor>(
        governor_params, static_cast<uint32_t>(quality_));

    initial_mode_ = declare_parameter<int>("initial_mode", 1);
    if (initial_mode_ < 1 || initial_mode_ > 3) {
        RCLCPP_ERROR(get_logger(), "initial_mode must be 1, 2, or 3, got %d", initial_mode_);
        return CallbackReturn::FAILURE;
    }

    auto vp =
        declare_parameter<std::vector<double>>("virtual_pose", {-4.0, 0.0, 3.5, 2.0, 0.0, -0.5});
    if (vp.size() != 6) {
        RCLCPP_ERROR(get_logger(), "virtual_pose must be 6 floats [eye xyz | target xyz], got %zu",
                     vp.size());
        return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < 3; ++i) pose_.eye[i] = vp[i];
    for (int i = 0; i < 3; ++i) pose_.target[i] = vp[3 + i];
    pose_.vfov_deg = declare_parameter<double>("virtual_vfov_deg", 80.0);

    auto theme_assets_dir_param = declare_parameter<std::string>("theme_assets_dir", "");
    std::string theme_assets_dir = theme_assets_dir_param;
    if (theme_assets_dir.empty()) {
        theme_assets_dir =
            ament_index_cpp::get_package_share_directory("overlume_ros") + "/assets/themes";
    }
    set_parameter(rclcpp::Parameter("theme_assets_dir", theme_assets_dir));

    auto initial_theme = declare_parameter<std::string>("initial_theme", "dark_adas");
    const std::string initial_theme_yaml = theme_assets_dir + "/" + initial_theme + ".yaml";
    if (!std::ifstream(initial_theme_yaml).good()) {
        RCLCPP_ERROR(get_logger(),
                     "initial_theme '%s' not found under theme_assets_dir '%s' (expected '%s')",
                     initial_theme.c_str(), theme_assets_dir.c_str(), initial_theme_yaml.c_str());
        return CallbackReturn::FAILURE;
    }
    set_parameter(rclcpp::Parameter("initial_theme", initial_theme));

    overlume::RenderConfig config{};
    config.width = static_cast<uint32_t>(out_width_);
    config.height = static_cast<uint32_t>(out_height_);
    config.quality = static_cast<uint8_t>(quality_);
    config.theme_assets_dir = theme_assets_dir.c_str();
    config.initial_theme = initial_theme.c_str();
    renderer_ = overlume::create_renderer(config);
    if (renderer_ == nullptr) {
        RCLCPP_ERROR(get_logger(), "overlume::create_renderer() failed (no GPU/EGL?)");
        return CallbackReturn::FAILURE;
    }
    frame_buf_.assign(static_cast<size_t>(out_width_) * out_height_ * 3, 0);

    if (!overlume::theme_assets_loaded(renderer_)) {
        RCLCPP_WARN(get_logger(),
                    "theme assets failed to load from '%s' -- rendering with the "
                    "compiled-in fallback theme instead",
                    theme_assets_dir.c_str());
    }

    auto profile_name = declare_parameter<std::string>("profile", "urban");
    auto profile_dir_param = declare_parameter<std::string>("profile_dir", "");
    std::string profile_dir = profile_dir_param;
    if (profile_dir.empty()) {
        profile_dir = ament_index_cpp::get_package_share_directory("overlume_ros") + "/config";
    }
    const std::string profile_path = profile_dir + "/" + profile_name + "_profile.yaml";
    std::vector<std::string> profile_errors;
    auto profile = overlume::ros::load_profile(profile_path, profile_errors);
    if (!profile.has_value()) {
        RCLCPP_ERROR(get_logger(), "failed to load profile '%s':", profile_path.c_str());
        for (const auto& err : profile_errors) RCLCPP_ERROR(get_logger(), "  %s", err.c_str());
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "profile '%s' loaded (%zu rows) from '%s'", profile->name.c_str(),
                profile->rows.size(), profile_path.c_str());
    for (const auto& err : profile_errors) RCLCPP_WARN(get_logger(), "  %s", err.c_str());
    for (const auto& row : profile->rows) {
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) {
            RCLCPP_INFO(get_logger(), "  (no subscription) -> %s/%s", row.adapter.c_str(),
                        row.role.c_str());
            continue;
        }
        for (const auto& spec : specs) {
            RCLCPP_INFO(get_logger(), "  %s -> %s/%s (qos: %s%s)", spec.topic.c_str(),
                        row.adapter.c_str(), row.role.c_str(),
                        spec.best_effort ? "best_effort" : "reliable",
                        spec.transient_local ? "+transient_local" : "");
        }
    }

    auto ego_model_path = declare_parameter<std::string>("ego_model_path", "");
    if (ego_model_path.empty()) {
        const std::string installed_ego =
            ament_index_cpp::get_package_share_directory("overlume_ros") + "/assets/ego/M02P.glb";
        if (std::ifstream(installed_ego).good()) {
            ego_model_path = installed_ego;
        }
    }
    set_parameter(rclcpp::Parameter("ego_model_path", ego_model_path));
    auto ego_dims = declare_parameter<std::vector<double>>("ego_fallback_dims", {4.5, 2.0, 1.8});
    if (ego_dims.size() != 3) {
        RCLCPP_ERROR(get_logger(),
                     "ego_fallback_dims must be 3 floats [len, width, height], got %zu",
                     ego_dims.size());
        return CallbackReturn::FAILURE;
    }
    overlume::Vec3 ego_fallback_dims{ego_dims[0], ego_dims[1], ego_dims[2]};
    if (!overlume::set_ego_model(renderer_, ego_model_path.c_str(), ego_fallback_dims)) {
        RCLCPP_WARN(get_logger(), "set_ego_model: failed to load '%s' -- using clay-box fallback",
                    ego_model_path.c_str());
    }

    hud_font_path_ = declare_parameter<std::string>("hud_font_path", "");
    if (hud_font_path_.empty()) {
        const std::string installed_font =
            ament_index_cpp::get_package_share_directory("overlume_ros") +
            "/assets/fonts/NotoSans-Regular.ttf";
        if (std::ifstream(installed_font).good()) {
            hud_font_path_ = installed_font;
        }
    }
    set_parameter(rclcpp::Parameter("hud_font_path", hud_font_path_));
    hud_enabled_ = declare_parameter<bool>("hud_enabled", true);
    callouts_enabled_ = declare_parameter<bool>("callouts_enabled", true);

    layer_objects_ = declare_parameter<bool>("layer_objects", true);
    layer_paths_ = declare_parameter<bool>("layer_paths", true);
    layer_map_elements_ = declare_parameter<bool>("layer_map_elements", true);
    layer_grids_ = declare_parameter<bool>("layer_grids", true);
    layer_alerts_ = declare_parameter<bool>("layer_alerts", true);
    layer_markers_ = declare_parameter<bool>("layer_markers", true);
    layer_point_clouds_ = declare_parameter<bool>("layer_point_clouds", true);
    layer_trajectory_carpet_ = declare_parameter<bool>("layer_trajectory_carpet", true);

    render_mode_ = declare_parameter<int>("render_mode", initial_mode_);
    if (render_mode_ < kRenderModeBowl || render_mode_ > kRenderModeFreeLook) {
        RCLCPP_WARN(get_logger(),
                    "render_mode must be 1 (bowl), 2 (hybrid) or 3 (free_look), "
                    "got %d -- defaulting to 3 (free_look)",
                    render_mode_);
        render_mode_ = kRenderModeFreeLook;
    }
    layer_surround_stitching_ = declare_parameter<bool>("layer_surround_stitching", false);
    surround_stitching_profile_ =
        declare_parameter<std::string>("surround_stitching_profile", "bowl");
    if (surround_stitching_profile_ != "bowl" && surround_stitching_profile_ != "hybrid") {
        RCLCPP_WARN(get_logger(),
                    "surround_stitching_profile must be 'bowl' or 'hybrid', got "
                    "'%s' -- defaulting to 'bowl'",
                    surround_stitching_profile_.c_str());
        surround_stitching_profile_ = "bowl";
    }

    auto ego_speed_smoothing_alpha = declare_parameter<double>("ego_speed_smoothing_alpha", 0.2);
    auto flatten_z = declare_parameter<bool>("flatten_z", true);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_adapter_ = std::make_unique<TfAdapter>(*tf_buffer_, "map", "base_link",
                                              ego_speed_smoothing_alpha, flatten_z);
    pub_ego_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/ego_state", 1);

    geo_anchor_solver_ = std::make_unique<GeoAnchorSolver>(*tf_buffer_, "map", "base_link");
    const double geo_datum_lat_deg =
        declare_parameter<double>("geo_datum_lat_deg", std::numeric_limits<double>::quiet_NaN());
    const double geo_datum_lon_deg =
        declare_parameter<double>("geo_datum_lon_deg", std::numeric_limits<double>::quiet_NaN());
    const double geo_datum_heading_deg = declare_parameter<double>(
        "geo_datum_heading_deg", std::numeric_limits<double>::quiet_NaN());
    const double geo_datum_height_m =
        declare_parameter<double>("geo_datum_height_m", std::numeric_limits<double>::quiet_NaN());
    const double geo_anchor_height_offset_m =
        declare_parameter<double>("geo_anchor_height_offset_m", 0.0);
    switch (ClassifyGeoDatum(geo_datum_lat_deg, geo_datum_lon_deg, geo_datum_heading_deg)) {
        case GeoDatumOverride::Complete: {
            geo_anchor_solver_->set_override(geo_datum_lat_deg, geo_datum_lon_deg,
                                             geo_datum_heading_deg);
            const double chosen_height_m =
                (std::isfinite(geo_datum_height_m) ? geo_datum_height_m : 0.0) +
                geo_anchor_height_offset_m;
            geo_anchor_solver_->set_origin_height_m(chosen_height_m);
            geo_anchor_logged_ = true;
            RCLCPP_INFO(get_logger(),
                        "geo-anchor solved (geo_datum override): --anchor-lat %.8f --anchor-lon "
                        "%.8f --anchor-heading-deg %.4f --anchor-height %.2f",
                        geo_datum_lat_deg, geo_datum_lon_deg, geo_datum_heading_deg,
                        chosen_height_m);
            break;
        }
        case GeoDatumOverride::Partial:
            RCLCPP_ERROR(get_logger(),
                         "geo_datum_lat_deg/lon_deg/heading_deg must be given all three or none "
                         "-- ignoring the partial override, sampling from NavSatFix+TF instead");
            break;
        case GeoDatumOverride::None:
            break;
    }
    const std::string gps_topic = declare_parameter<std::string>("gps_topic", "/sim/feedback/gps");
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, rclcpp::QoS(10).best_effort(),
        [this, geo_datum_height_m,
         geo_anchor_height_offset_m](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            geo_anchor_solver_->on_fix(*msg);
            if (!geo_anchor_logged_ && geo_anchor_solver_->solved()) {
                geo_anchor_logged_ = true;
                const double sampled_height_m = geo_anchor_solver_->anchor().origin_height_m;
                geo_anchor_solver_->set_origin_height_m(ChooseAnchorHeightM(
                    geo_datum_height_m, sampled_height_m, geo_anchor_height_offset_m));
                const overlume::GeoAnchor a = geo_anchor_solver_->anchor();
                RCLCPP_INFO(get_logger(),
                            "geo-anchor solved: --anchor-lat %.8f --anchor-lon %.8f "
                            "--anchor-heading-deg %.4f --anchor-height %.2f",
                            a.origin_lat_deg, a.origin_lon_deg, a.heading_rad * 180.0 / M_PI,
                            a.origin_height_m);
            }
        });

    bowl_enabled_ = declare_parameter<bool>("bowl_enabled", false);
    const int n_cameras = declare_parameter<int>("n_cameras", 6);
    if (n_cameras <= 0 || static_cast<uint32_t>(n_cameras) > overlume::kMaxBowlCameras) {
        RCLCPP_ERROR(get_logger(), "n_cameras must be in [1, %u], got %d",
                     overlume::kMaxBowlCameras, n_cameras);
        return CallbackReturn::FAILURE;
    }
    auto image_topics =
        declare_parameter<std::vector<std::string>>("image_topics", std::vector<std::string>{});
    auto info_topics =
        declare_parameter<std::vector<std::string>>("info_topics", std::vector<std::string>{});
    const bool bowl_topics_valid = static_cast<int>(image_topics.size()) == n_cameras &&
                                   static_cast<int>(info_topics.size()) == n_cameras;
    if (bowl_enabled_ && !bowl_topics_valid) {
        RCLCPP_WARN(get_logger(),
                    "bowl_enabled requested but image_topics/info_topics don't have n_cameras (%d) "
                    "entries (%zu/%zu) -- bowl disabled this run",
                    n_cameras, image_topics.size(), info_topics.size());
        bowl_enabled_ = false;
    }
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "");
    declare_parameter<std::string>("config_path", "");
    max_sync_latency_ = declare_parameter<double>("max_sync_latency", 0.12);
    bowl_R0_ = declare_parameter<double>("bowl_R0", bowl_R0_);
    bowl_k_ = declare_parameter<double>("bowl_k", bowl_k_);
    bowl_Rmax_ = declare_parameter<double>("bowl_Rmax", bowl_Rmax_);
    feather_margin_ = declare_parameter<double>("feather_margin", feather_margin_);
    {
        const bool fbz = declare_parameter<bool>("fill_blind_zone", false);
        if (fbz) {
            RCLCPP_WARN(get_logger(),
                        "fill_blind_zone: true requested but forced to false -- no Filament-side "
                        "implementation this epic (Decision 3)");
        }
        fill_blind_zone_ = false;
        const bool em = declare_parameter<bool>("exposure_match", false);
        if (em) {
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

    std::vector<overlume::CameraExtrinsics> camera_extrinsics(static_cast<size_t>(n_cameras));
    if (bowl_enabled_) {
        std::vector<double> ext_default;
        for (int i = 0; i < n_cameras; ++i) {
            const double angle = 2.0 * M_PI * i / n_cameras;
            const double ca = std::cos(angle), sa = std::sin(angle);
            const std::vector<double> row = {ca, -sa, 0, 0,         0,         -1,
                                             sa, ca,  0, 0.55 * ca, 0.55 * sa, 0.55};
            ext_default.insert(ext_default.end(), row.begin(), row.end());
        }
        auto ext_vec = declare_parameter<std::vector<double>>("camera_extrinsics", ext_default);
        if (static_cast<int>(ext_vec.size()) != n_cameras * 12) {
            RCLCPP_ERROR(get_logger(),
                         "camera_extrinsics must have n_cameras*12 = %d floats, got %zu",
                         n_cameras * 12, ext_vec.size());
            return CallbackReturn::FAILURE;
        }
        for (int i = 0; i < n_cameras; ++i) {
            overlume::CameraExtrinsics& ext = camera_extrinsics[static_cast<size_t>(i)];
            const int base = i * 12;
            for (int j = 0; j < 9; ++j) ext.R[j] = ext_vec[base + j];
            for (int j = 0; j < 3; ++j) ext.t[j] = ext_vec[base + 9 + j];
        }
    } else {
        declare_parameter<std::vector<double>>("camera_extrinsics", std::vector<double>{});
    }
    if (bowl_enabled_) {
        camera_ingest_ =
            std::make_unique<CameraIngest>(this, static_cast<uint32_t>(n_cameras), image_topics,
                                           info_topics, odom_topic, camera_extrinsics);
        camera_ingest_->set_renderer(renderer_);
        camera_ingest_->set_bowl_enabled(bowl_enabled_);
        camera_ingest_->set_max_sync_latency(max_sync_latency_);
    }

    hybrid_enabled_ = declare_parameter<bool>("hybrid_enabled", false);
    if (camera_ingest_) camera_ingest_->set_hybrid_enabled(hybrid_enabled_);
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "");
    auto pc_tf = declare_parameter<std::vector<double>>("pointcloud_transform",
                                                        {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0});
    if (hybrid_enabled_) {
        if (pc_tf.size() != 12) {
            RCLCPP_ERROR(get_logger(),
                         "pointcloud_transform must be 12 floats [R(9)|t(3)], got %zu",
                         pc_tf.size());
            return CallbackReturn::FAILURE;
        }
        for (int i = 0; i < 12; ++i) pointcloud_tf_[i] = static_cast<float>(pc_tf[i]);
    }
    splat_radius_ = static_cast<int>(declare_parameter<int>("splat_radius", 3));
    if (hybrid_enabled_ && !pointcloud_topic_.empty()) {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                if (!camera_ingest_ || !hybrid_cloud_consumed()) return;
                int ox = -1, oy = -1, oz = -1;
                for (const auto& f : msg->fields) {
                    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
                    if (f.name == "x")
                        ox = static_cast<int>(f.offset);
                    else if (f.name == "y")
                        oy = static_cast<int>(f.offset);
                    else if (f.name == "z")
                        oz = static_cast<int>(f.offset);
                }
                if (ox < 0 || oy < 0 || oz < 0) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "hybrid: point cloud lacks float32 x/y/z fields; ignoring");
                    return;
                }
                const float* T = pointcloud_tf_;
                const size_t n = static_cast<size_t>(msg->width) * msg->height;
                std::vector<overlume::Vec3> pts;
                pts.reserve(n);
                const uint8_t* base = msg->data.data();
                for (size_t p = 0; p < n; ++p) {
                    const uint8_t* rec = base + p * msg->point_step;
                    float x, y, z;
                    std::memcpy(&x, rec + ox, 4);
                    std::memcpy(&y, rec + oy, 4);
                    std::memcpy(&z, rec + oz, 4);
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                    pts.push_back(overlume::Vec3{T[0] * x + T[1] * y + T[2] * z + T[9],
                                                 T[3] * x + T[4] * y + T[5] * z + T[10],
                                                 T[6] * x + T[7] * y + T[8] * z + T[11]});
                }
                const double stamp = rclcpp::Time(msg->header.stamp).seconds();
                std::lock_guard<std::mutex> lk(cloud_mtx_);
                cloud_pts_rig_.swap(pts);
                cloud_stamp_ = stamp;
                cloud_rx_sec_ = sim_clock_sec_;
            });
        RCLCPP_INFO(get_logger(), "hybrid rendering enabled (point cloud: %s)",
                    pointcloud_topic_.c_str());
    }

    environment_enabled_ = declare_parameter<bool>("environment_enabled", true);
    environment_chunks_dir_ = declare_parameter<std::string>("environment_chunks_dir", "");
    environment_source_uri_ = declare_parameter<std::string>("environment_source_uri", "");
    environment_tile_cache_dir_ = declare_parameter<std::string>("environment_tile_cache_dir", "");
    environment_follow_terrain_ = declare_parameter<bool>("environment_follow_terrain", true);
    environment_ground_bias_m_ = declare_parameter<double>("environment_ground_bias_m", 0.3);
    environment_replaces_ground_ = declare_parameter<bool>("environment_replaces_ground", true);
    environment_max_tilt_deg_ = declare_parameter<double>("environment_max_tilt_deg", 2.0);
    environment_brightness_ = declare_parameter<double>("environment_brightness", 1.0);
    environment_tile_radius_m_ = declare_parameter<double>("environment_tile_radius_m", 700.0);
    environment_attribution_ = declare_parameter<bool>("environment_attribution", true);
    environment_own_asset_uri_ = declare_parameter<std::string>("environment_own_asset_uri", "");

    frame_transformer_ = std::make_unique<FrameTransformer>(*tf_buffer_, "map", flatten_z);
    for (const auto& row : profile->rows) {
        if (row.adapter != "hd_map") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<overlume::ros::HdMapAdapter>(row, *frame_transformer_);
        overlume::ros::HdMapAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        hd_map_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        hd_map_rows_.push_back(HdMapRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "hd_map: %zu row(s) subscribed", hd_map_rows_.size());

    const std::string class_inference_path = profile_dir + "/class_inference.yaml";
    std::vector<std::string> class_inference_errors;
    if (auto table =
            overlume::ros::load_class_inference(class_inference_path, class_inference_errors)) {
        class_inference_ = std::move(*table);
    } else {
        RCLCPP_ERROR(get_logger(),
                     "failed to load class inference table '%s':", class_inference_path.c_str());
        for (const auto& err : class_inference_errors)
            RCLCPP_ERROR(get_logger(), "  %s", err.c_str());
        return CallbackReturn::FAILURE;
    }

    for (const auto& row : profile->rows) {
        if (row.adapter != "dynamic_objects") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<overlume::ros::DynamicObjectsAdapter>(
            row, *frame_transformer_, class_inference_);
        overlume::ros::DynamicObjectsAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        dynamic_objects_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        dynamic_objects_rows_.push_back(
            DynamicObjectsRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "dynamic_objects: %zu row(s) subscribed",
                dynamic_objects_rows_.size());

    for (const auto& row : profile->rows) {
        if (row.adapter != "path") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<overlume::ros::PathAdapter>(row, *frame_transformer_);
        overlume::ros::PathAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        path_subs_.push_back(create_subscription<nav_msgs::msg::Path>(
            spec.topic, qos, [this, adapter_ptr](const nav_msgs::msg::Path::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        path_rows_.push_back(PathRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "path: %zu row(s) subscribed", path_rows_.size());

    for (const auto& row : profile->rows) {
        if (row.adapter != "ogm") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& gridSpec = specs[0];

        auto adapter = std::make_unique<overlume::ros::OgmAdapter>(row, *frame_transformer_);
        overlume::ros::OgmAdapter* adapter_ptr = adapter.get();

        rclcpp::QoS gridQos(10);
        if (gridSpec.best_effort) gridQos.best_effort();
        if (gridSpec.transient_local) gridQos.transient_local();
        ogm_grid_subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
            gridSpec.topic, gridQos,
            [this, adapter_ptr](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));

        if (specs.size() > 1) {
            const auto& updateSpec = specs[1];
            rclcpp::QoS updateQos(10);
            if (updateSpec.best_effort) updateQos.best_effort();
            if (updateSpec.transient_local) updateQos.transient_local();
            ogm_update_subs_.push_back(create_subscription<map_msgs::msg::OccupancyGridUpdate>(
                updateSpec.topic, updateQos,
                [this, adapter_ptr](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
                    adapter_ptr->ingest_update(*msg, sim_clock_sec_);
                }));
        }

        ogm_rows_.push_back(OgmRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "ogm: %zu row(s) subscribed", ogm_rows_.size());

    for (const auto& row : profile->rows) {
        if (row.adapter != "collision") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<overlume::ros::CollisionAdapter>(row, *frame_transformer_);
        overlume::ros::CollisionAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        collision_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        collision_rows_.push_back(CollisionRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "collision: %zu row(s) subscribed", collision_rows_.size());

    for (const auto& row : profile->rows) {
        if (row.adapter != "generic") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter =
            std::make_unique<overlume::ros::GenericMarkerAdapter>(row, *frame_transformer_);
        overlume::ros::GenericMarkerAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        generic_marker_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        generic_marker_rows_.push_back(
            GenericMarkerRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "generic: %zu row(s) subscribed", generic_marker_rows_.size());

    for (const auto& row : profile->rows) {
        if (row.adapter != "point_cloud") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter = std::make_unique<overlume::ros::PointCloudAdapter>(row, *frame_transformer_);
        overlume::ros::PointCloudAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        point_cloud_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
            spec.topic, qos,
            [this, adapter_ptr](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        point_cloud_rows_.push_back(PointCloudRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "point_cloud: %zu row(s) subscribed", point_cloud_rows_.size());
    if (hybrid_enabled_ && !pointcloud_topic_.empty()) {
        for (const auto& pcr : point_cloud_rows_) {
            if (pcr.topic == pointcloud_topic_) {
                RCLCPP_WARN(get_logger(),
                            "pointcloud_topic '%s' matches a profile point_cloud row -- this "
                            "node holds TWO subscriptions to it (hybrid's own cloud_sub_ plus "
                            "this profile row's PointCloudAdapter); the profile row is suppressed "
                            "while hybrid consumes the cloud",
                            pointcloud_topic_.c_str());
            }
        }
    }

    for (const auto& row : profile->rows) {
        if (row.adapter != "trajectory_carpet") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& spec = specs.front();

        auto adapter =
            std::make_unique<overlume::ros::TrajectoryCarpetAdapter>(row, *frame_transformer_);
        overlume::ros::TrajectoryCarpetAdapter* adapter_ptr = adapter.get();
        rclcpp::QoS qos(10);
        if (spec.best_effort) qos.best_effort();
        if (spec.transient_local) qos.transient_local();
        carpet_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
            spec.topic, qos,
            [this, adapter_ptr](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        carpet_rows_.push_back(CarpetRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "trajectory_carpet: %zu row(s) subscribed", carpet_rows_.size());

    for (const auto& row : profile->rows) {
        if (row.adapter != "tf_axes") continue;
        tf_axes_rows_.push_back(std::make_unique<overlume::ros::TfAxesAdapter>(row, *tf_buffer_));
    }
    RCLCPP_INFO(get_logger(), "tf_axes: %zu row(s) configured", tf_axes_rows_.size());

    robot_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/robot/feedback/robot_speed_mps", rclcpp::QoS(10).best_effort(),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            tf_adapter_->set_robot_speed_mps(msg->data);
        });

    pub_image_ = create_publisher<sensor_msgs::msg::Image>("/rendering/image", 1);
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/rendering/camera_info", 1);
    pub_vcam_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/vcam_state", 1);
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 1);

    vcam_ = std::make_unique<Vcam>(this, pose_);

    set_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/rendering/set_mode", rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            if (msg->data != 1 && msg->data != 2 && msg->data != 3) {
                RCLCPP_WARN(get_logger(), "set_mode: expected 1|2|3, got %d", msg->data);
                return;
            }
            render_mode_ = msg->data;
            RCLCPP_INFO(get_logger(), "overlume_node: render_mode -> %d", render_mode_);
            apply_environment_visibility();
        });

    theme_sub_ = create_subscription<std_msgs::msg::String>(
        "~/set_theme", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            if (!overlume::set_theme(renderer_, msg->data.c_str(), sim_clock_sec_, 0.0)) {
                RCLCPP_WARN(get_logger(), "set_theme: unknown theme '%s'", msg->data.c_str());
            }
        });

    layer_param_cb_ = add_on_set_parameters_callback(
        std::bind(&OverlumeNode::on_params, this, std::placeholders::_1));

    if ((render_mode_ == kRenderModeBowl || render_mode_ == kRenderModeHybrid) && !bowl_enabled_ &&
        !bowl_mode_warned_) {
        bowl_mode_warned_ = true;
        RCLCPP_WARN(get_logger(),
                    "render_mode=%d masks the whole autonomy scene but bowl_enabled is false -- "
                    "the frame will be near-empty (sky + ego) until the bowl is configured",
                    render_mode_);
    }

    RCLCPP_INFO(get_logger(),
                "on_configure() succeeded. out=%dx%d quality=%d initial_mode=%d "
                "flatten_z=%s",
                out_width_, out_height_, quality_, initial_mode_, flatten_z ? "true" : "false");
    return CallbackReturn::SUCCESS;
}

void OverlumeNode::apply_environment_visibility() {
    const bool want = environment_effectively_visible(static_cast<RenderMode>(render_mode_),
                                                      environment_enabled_);
    if (overlume::environment_visible(renderer_) == want) return;
    overlume::set_environment_visible(renderer_, want);
    RCLCPP_INFO(get_logger(),
                "environment visibility -> %s (render_mode=%d, environment_enabled=%s)",
                want ? "visible" : "hidden", render_mode_, environment_enabled_ ? "true" : "false");
}

OverlumeNode::CallbackReturn OverlumeNode::on_activate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "on_activate() called.");
    pub_image_->on_activate();
    pub_info_->on_activate();
    pub_vcam_state_->on_activate();
    pub_ego_state_->on_activate();
    pub_diagnostics_->on_activate();

    apply_environment_visibility();
    if (environment_chunks_dir_.empty() && environment_source_uri_.empty()) {
        RCLCPP_WARN(get_logger(),
                    "neither environment_chunks_dir nor environment_source_uri is set -- "
                    "environment layer disabled this run");
    } else if (geo_anchor_solver_->solved()) {
        const std::string source_uri = compose_environment_source_uri(
            environment_chunks_dir_, environment_source_uri_, environment_tile_cache_dir_,
            environment_follow_terrain_, environment_ground_bias_m_, environment_replaces_ground_,
            environment_max_tilt_deg_, environment_brightness_, environment_tile_radius_m_);
        if (!overlume::set_environment_source(renderer_, source_uri.c_str(),
                                              geo_anchor_solver_->anchor())) {
            RCLCPP_WARN(get_logger(),
                        "set_environment_source: failed to open '%s' -- no buildings this run",
                        source_uri.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "environment source armed: '%s' (%s)", source_uri.c_str(),
                        overlume::environment_visible(renderer_) ? "visible" : "hidden");
        }
    } else if (!environment_warned_) {
        environment_warned_ = true;
        RCLCPP_WARN(get_logger(),
                    "no geo-anchor solved yet at on_activate() -- environment layer disabled "
                    "this run (spec Sec.4.5/9)");
    }

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(33ms, [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(), "on_activate() succeeded.");
    return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult OverlumeNode::on_params(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    for (const auto& p : params) {
        const std::string& n = p.get_name();
        try {
            if (n == "layer_objects")
                layer_objects_ = p.as_bool();
            else if (n == "layer_paths")
                layer_paths_ = p.as_bool();
            else if (n == "layer_map_elements")
                layer_map_elements_ = p.as_bool();
            else if (n == "layer_grids")
                layer_grids_ = p.as_bool();
            else if (n == "layer_alerts")
                layer_alerts_ = p.as_bool();
            else if (n == "layer_markers")
                layer_markers_ = p.as_bool();
            else if (n == "layer_point_clouds")
                layer_point_clouds_ = p.as_bool();
            else if (n == "layer_trajectory_carpet")
                layer_trajectory_carpet_ = p.as_bool();
            else if (n == "render_mode") {
                const int v = static_cast<int>(p.as_int());
                if (v < kRenderModeBowl || v > kRenderModeFreeLook) {
                    res.successful = false;
                    res.reason = "render_mode must be 1 (bowl), 2 (hybrid) or 3 (free_look)";
                } else {
                    render_mode_ = v;
                    apply_environment_visibility();
                    const bool bowl_ready =
                        bowl_enabled_ && camera_ingest_ && camera_ingest_->config_applied();
                    if ((render_mode_ == kRenderModeBowl || render_mode_ == kRenderModeHybrid) &&
                        !bowl_ready && !bowl_mode_warned_) {
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
            } else if (n == "layer_surround_stitching") {
                layer_surround_stitching_ = p.as_bool();
                if (layer_surround_stitching_ &&
                    (!camera_ingest_ || !camera_ingest_->config_applied())) {
                    RCLCPP_WARN(get_logger(),
                                "layer_surround_stitching enabled but no bowl is configured "
                                "(bowl_enabled=%s, camera ingest %s) -- nothing will render "
                                "until the bowl is enabled and every camera_info has arrived",
                                bowl_enabled_ ? "true" : "false",
                                camera_ingest_
                                    ? (camera_ingest_->config_applied() ? "configured"
                                                                        : "waiting on camera_info")
                                    : "absent");
                }
            } else if (n == "surround_stitching_profile") {
                const std::string v = p.as_string();
                if (v != "bowl" && v != "hybrid") {
                    res.successful = false;
                    res.reason = "surround_stitching_profile must be 'bowl' or 'hybrid'";
                } else {
                    surround_stitching_profile_ = v;
                }
            } else if (n == "environment_enabled") {
                environment_enabled_ = p.as_bool();
                apply_environment_visibility();
                if (overlume::environment_source_state(renderer_) ==
                    overlume::EnvironmentSourceState::NONE) {
                    RCLCPP_WARN(get_logger(),
                                "environment_enabled set to %s but no environment source is "
                                "configured yet -- nothing to show/hide this run",
                                environment_enabled_ ? "true" : "false");
                }
            } else if (n == "environment_source_uri") {
                const std::string requested = p.as_string();
                if (!geo_anchor_solver_ || !geo_anchor_solver_->solved()) {
                    res.successful = false;
                    res.reason =
                        "environment_source_uri: geo-anchor not solved yet -- "
                        "cannot switch the environment source live";
                } else {
                    apply_environment_visibility();
                    const std::string source_uri = compose_environment_source_uri(
                        environment_chunks_dir_, requested, environment_tile_cache_dir_,
                        environment_follow_terrain_, environment_ground_bias_m_,
                        environment_replaces_ground_, environment_max_tilt_deg_,
                        environment_brightness_, environment_tile_radius_m_);
                    if (source_uri.empty()) {
                        res.successful = false;
                        res.reason = "environment_source_uri: '" + requested +
                                     "' selected but environment_chunks_dir is not configured on "
                                     "this deployment";
                        RCLCPP_WARN(get_logger(), "%s", res.reason.c_str());
                    } else if (overlume::set_environment_source(renderer_, source_uri.c_str(),
                                                                geo_anchor_solver_->anchor())) {
                        environment_source_uri_ = requested;
                        environment_fallback_warned_ = false;
                        environment_attribution_warned_ = false;
                        RCLCPP_INFO(
                            get_logger(), "environment source armed: '%s' (%s)", source_uri.c_str(),
                            overlume::environment_visible(renderer_) ? "visible" : "hidden");
                    } else {
                        res.successful = false;
                        res.reason = "set_environment_source: failed to open '" + source_uri +
                                     "' -- previous environment source left intact";
                        RCLCPP_WARN(get_logger(), "%s", res.reason.c_str());
                    }
                }
            } else if (n == "splat_radius") {
                const int v = static_cast<int>(p.as_int());
                if (v < 0 || v > 30) {
                    res.successful = false;
                    res.reason = "splat_radius must be in [0, 30]";
                } else {
                    splat_radius_ = v;
                }
            } else if (n == "bowl_R0") {
                bowl_R0_ = p.as_double();
                bowl_config_dirty_ = true;
            } else if (n == "bowl_k") {
                bowl_k_ = p.as_double();
                bowl_config_dirty_ = true;
            } else if (n == "bowl_Rmax") {
                bowl_Rmax_ = p.as_double();
                bowl_config_dirty_ = true;
            } else if (n == "feather_margin") {
                feather_margin_ = p.as_double();
                bowl_config_dirty_ = true;
            } else if (n == "bowl_exposure_compensation") {
                bowl_exposure_compensation_ = static_cast<float>(p.as_double());
                bowl_config_dirty_ = true;
            } else if (n == "sky_color") {
                auto v = p.as_double_array();
                if (v.size() == 3) {
                    for (int i = 0; i < 3; ++i) sky_color_[i] = static_cast<float>(v[i]);
                    bowl_config_dirty_ = true;
                }
            } else if (n == "fill_blind_zone") {
                const bool requested = p.as_bool();
                fill_blind_zone_ = false;
                if (requested)
                    RCLCPP_WARN(get_logger(),
                                "fill_blind_zone: true requested but forced to false -- no "
                                "Filament-side implementation this epic (Decision 3)");
            } else if (n == "exposure_match") {
                const bool requested = p.as_bool();
                exposure_match_ = false;
                if (requested)
                    RCLCPP_WARN(get_logger(),
                                "exposure_match: true requested but forced to false -- no "
                                "Filament-side implementation this epic (Decision 3)");
            }
        } catch (const std::exception& e) {
            res.successful = false;
            res.reason = std::string("bad value for ") + n + ": " + e.what();
        }
    }
    return res;
}

namespace {
void warn_on_drop_growth(const rclcpp::Logger& logger, rclcpp::Clock& clock,
                         const std::string& topic, const overlume::ros::AdapterStats& s,
                         uint64_t& warned_malformed, uint64_t& warned_no_tf) {
    if (s.dropped_malformed > warned_malformed || s.dropped_no_tf > warned_no_tf) {
        RCLCPP_WARN_THROTTLE(logger, clock, 5000,
                             "%s: dropped %llu malformed, %llu without TF (cumulative)",
                             topic.c_str(), static_cast<unsigned long long>(s.dropped_malformed),
                             static_cast<unsigned long long>(s.dropped_no_tf));
        warned_malformed = s.dropped_malformed;
        warned_no_tf = s.dropped_no_tf;
    }
}
}

void OverlumeNode::timer_callback() {
    vcam_->advance_tween();
    pose_ = vcam_->pose();

    std_msgs::msg::Float64MultiArray state;
    state.data = {pose_.eye[0],
                  pose_.eye[1],
                  pose_.eye[2],
                  pose_.target[0],
                  pose_.target[1],
                  pose_.target[2],
                  static_cast<double>(vcam_->active_preset()),
                  static_cast<double>(render_mode_),
                  static_cast<double>(render_mode_)};
    pub_vcam_state_->publish(state);

    sim_clock_sec_ += kTimerPeriodSec;

    if (bowl_enabled_ && camera_ingest_) {
        auto apply_bowl_config = [&]() -> bool {
            std::vector<overlume::CameraExtrinsics> ext;
            std::vector<overlume::CameraIntrinsics> in;
            std::vector<uint32_t> w, h;
            camera_ingest_->fill_bowl_intrinsics(ext, in, w, h);
            overlume::BowlConfig bc{};
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
            return overlume::set_bowl_config(renderer_, bc);
        };

        if (!camera_ingest_->config_applied()) {
            if (camera_ingest_->all_info_ready()) {
                if (apply_bowl_config()) {
                    camera_ingest_->mark_bowl_config_applied();
                    RCLCPP_INFO(get_logger(), "bowl: configured");
                } else {
                    RCLCPP_WARN(get_logger(),
                                "set_bowl_config() failed with all CameraInfo present");
                }
            }
        } else if (camera_ingest_->consume_info_dirty()) {
            if (apply_bowl_config())
                RCLCPP_INFO(get_logger(), "bowl: re-baked (CameraInfo changed)");
            else
                RCLCPP_WARN(get_logger(), "bowl: re-bake after CameraInfo change failed");
        } else if (bowl_config_dirty_) {
            if (apply_bowl_config()) {
                bowl_config_dirty_ = false;
                RCLCPP_INFO(get_logger(), "bowl: re-baked (live param change)");
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "bowl: live-param re-bake failed -- retrying");
            }
        }
        camera_ingest_->update_motion_deltas();
    }

    const auto render_mode = static_cast<RenderMode>(render_mode_);
    overlume::set_bowl_visible(renderer_,
                               bowl_visible_for_mode(render_mode, layer_surround_stitching_));

    scene_asm_.clear();
    for (auto& hr : hd_map_rows_) {
        warn_on_drop_growth(get_logger(), *get_clock(), hr.topic, hr.adapter->stats(),
                            hr.warned_malformed, hr.warned_no_tf);
        if (sim_clock_sec_ - hr.adapter->stats().last_msg_sec > hr.timeout_sec) continue;
        hr.adapter->fill(scene_asm_);
    }

    for (auto& dr : dynamic_objects_rows_) {
        if (dr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), dr.topic, dr.adapter->stats(),
                            dr.warned_malformed, dr.warned_no_tf);
        if (sim_clock_sec_ - dr.adapter->stats().last_msg_sec > dr.timeout_sec) {
            dr.adapter->mark_stale_tick();
            continue;
        }
        dr.adapter->fill(scene_asm_);
    }

    for (auto& pr : path_rows_) {
        if (pr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), pr.topic, pr.adapter->stats(),
                            pr.warned_malformed, pr.warned_no_tf);
        if (sim_clock_sec_ - pr.adapter->stats().last_msg_sec > pr.timeout_sec) {
            pr.adapter->mark_stale_tick();
            continue;
        }
        pr.adapter->fill(scene_asm_);
    }

    for (auto& gr : ogm_rows_) {
        if (gr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), gr.topic, gr.adapter->stats(),
                            gr.warned_malformed, gr.warned_no_tf);
        if (sim_clock_sec_ - gr.adapter->stats().last_msg_sec > gr.timeout_sec) {
            gr.adapter->mark_stale_tick();
            continue;
        }
        gr.adapter->fill(scene_asm_);
    }

    for (auto& cr : collision_rows_) {
        if (cr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), cr.topic, cr.adapter->stats(),
                            cr.warned_malformed, cr.warned_no_tf);
        if (sim_clock_sec_ - cr.adapter->stats().last_msg_sec > cr.timeout_sec) {
            cr.adapter->mark_stale_tick();
            continue;
        }
        cr.adapter->fill(scene_asm_);
    }

    for (auto& gmr : generic_marker_rows_) {
        if (gmr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), gmr.topic, gmr.adapter->stats(),
                            gmr.warned_malformed, gmr.warned_no_tf);
        if (sim_clock_sec_ - gmr.adapter->stats().last_msg_sec > gmr.timeout_sec) {
            gmr.adapter->mark_stale_tick();
            continue;
        }
        gmr.adapter->fill(scene_asm_);
    }

    bool suppressed = false;
    for (auto& pcr : point_cloud_rows_) {
        if (hybrid_cloud_consumed() && hybrid_enabled_ && cloud_sub_ &&
            pcr.topic == pointcloud_topic_) {
            suppressed = true;  // hybrid draws this cloud as splats; the row would duplicate it
            continue;
        }
        if (pcr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), pcr.topic, pcr.adapter->stats(),
                            pcr.warned_malformed, pcr.warned_no_tf);
        if (sim_clock_sec_ - pcr.adapter->stats().last_msg_sec > pcr.timeout_sec) {
            pcr.adapter->mark_stale_tick();
            continue;
        }
        pcr.adapter->fill(scene_asm_);
    }

    if (suppressed != hybrid_row_suppressed_) {
        hybrid_row_suppressed_ = suppressed;
        if (suppressed)
            RCLCPP_INFO(get_logger(), "hybrid: profile row %s suppressed", pointcloud_topic_.c_str());
    }

    for (auto& cr : carpet_rows_) {
        if (cr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), cr.topic, cr.adapter->stats(),
                            cr.warned_malformed, cr.warned_no_tf);
        if (sim_clock_sec_ - cr.adapter->stats().last_msg_sec > cr.timeout_sec) {
            cr.adapter->mark_stale_tick();
            continue;
        }
        cr.adapter->fill(scene_asm_);
    }

    for (auto& axes : tf_axes_rows_) {
        axes->fill(scene_asm_, sim_clock_sec_);
    }

    overlume::SceneGraph scene{};
    scene.sim_time_sec = sim_clock_sec_;
    scene.ego = tf_adapter_->update();
    overlume::ros::PopulateHud(scene, render_mode_);

    overlume::ros::respine_velocity_ribbon_onto_local_path(scene_asm_);

    const LayerFlags user_layer_flags{
        layer_objects_, layer_paths_,   layer_map_elements_, layer_grids_,
        layer_alerts_,  layer_markers_, layer_point_clouds_, layer_trajectory_carpet_};
    apply_layer_gates(scene_asm_,
                      compose_layer_gates(user_layer_flags, mode_content_mask(render_mode)));

    if (camera_ingest_)
        camera_ingest_->set_hybrid_enabled(hybrid_enabled_ && hybrid_cloud_consumed());

    // Re-evaluated every tick, so one site covers configure and every live switch.
    hybrid_starved_reason_ =
        overlume::ros::HybridStarvedReason(hybrid_cloud_consumed(), hybrid_enabled_, cloud_sub_ != nullptr);
    if (!hybrid_starved_reason_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "hybrid: render_mode=%d/profile=%s consumes a lidar cloud but %s -- "
                             "splats will not render",
                             render_mode_, surround_stitching_profile_.c_str(),
                             hybrid_starved_reason_.c_str());
    }

    if (render_mode == RenderMode::HYBRID) scene_asm_.point_clouds.clear();

    if (hybrid_cloud_consumed() && hybrid_enabled_ && camera_ingest_) {
        std::vector<overlume::Vec3> pts;
        double stamp = 0.0, rx = 0.0;
        {
            std::lock_guard<std::mutex> lk(cloud_mtx_);
            pts = cloud_pts_rig_;
            stamp = cloud_stamp_;
            rx = cloud_rx_sec_;
        }
        // A frozen lidar must not leave stale splats (profile row timeout_sec parity).
        if (sim_clock_sec_ - rx > 2.0) pts.clear();

        double th, px, py;
        if (camera_ingest_->cloud_motion_delta(stamp, th, px, py)) CompensateCloud(pts, th, px, py);

        // Colourise through the same per-camera motion deltas the bowl shader applies.
        std::vector<overlume::CameraExtrinsics> ext;
        camera_ingest_->fill_compensated_extrinsics(ext);
        std::vector<overlume::CameraExtrinsics> ext_unused;
        std::vector<overlume::CameraIntrinsics> in;
        std::vector<uint32_t> cw, ch;
        camera_ingest_->fill_bowl_intrinsics(ext_unused, in, cw, ch);
        std::vector<const uint8_t*> rgb_bufs;
        camera_ingest_->fill_camera_rgb_buffers(rgb_bufs);

        overlume::BowlConfig cams{};
        cams.camera_count = static_cast<uint32_t>(ext.size());
        cams.extrinsics = ext.data();
        cams.intrinsics = in.data();
        cams.cam_width = cw.data();
        cams.cam_height = ch.data();

        const auto colorized = ColorizeFromCameras(pts, cams, rgb_bufs);
        if (!pts.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "hybrid: colorized %zu/%zu lidar points (%.1f%% coverage)",
                                 colorized.size(), pts.size(),
                                 100.0 * static_cast<double>(colorized.size()) /
                                     static_cast<double>(pts.size()));
        }
        // Rig-frame points: the library anchors them with the bowl's ego transform.
        overlume::set_hybrid_splats(renderer_, colorized.data(),
                                    static_cast<uint32_t>(colorized.size()), hybrid_splat_px());
    } else {
        overlume::set_hybrid_splats(renderer_, nullptr, 0, 0.0f);
    }

    scene_asm_.point_at(scene);
    overlume::set_scene(renderer_, scene);

    std_msgs::msg::Float64MultiArray ego_state;
    ego_state.data = {scene.ego.position.x, scene.ego.position.y,
                      scene.ego.position.z, scene.ego.heading_rad,
                      scene.ego.speed_mps,  static_cast<double>(scene.ego.valid)};
    pub_ego_state_->publish(ego_state);

    overlume::CameraPose render_pose = pose_;
    if (scene.ego.valid) {
        render_pose = compose_ego_anchored_pose(pose_, scene.ego);
    }

    overlume::FrameView view{frame_buf_.data(), static_cast<uint32_t>(out_width_),
                             static_cast<uint32_t>(out_height_)};
    const auto render_start = std::chrono::steady_clock::now();
    if (!overlume::render_frame(renderer_, render_pose, view)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "render_frame() failed");
        render_ms_ = 0.0;
        publish_diagnostics();
        return;
    }
    render_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_start)
            .count();

    if (!environment_fallback_warned_ && overlume::environment_source_state(renderer_) ==
                                             overlume::EnvironmentSourceState::STREAMING_FALLBACK) {
        environment_fallback_warned_ = true;
        const std::string fallback_dir =
            fallback_dir_from_source_uri(compose_environment_source_uri(
                environment_chunks_dir_, environment_source_uri_, environment_tile_cache_dir_,
                environment_follow_terrain_, environment_ground_bias_m_,
                environment_replaces_ground_, environment_max_tilt_deg_, environment_brightness_,
                environment_tile_radius_m_));
        if (fallback_dir.empty()) {
            RCLCPP_WARN(get_logger(),
                        "environment source: network loss detected -- switched to fallback, "
                        "but none configured -- environment now empty");
        } else {
            RCLCPP_WARN(
                get_logger(),
                "environment source: network loss detected -- switched to fallback dir '%s'",
                fallback_dir.c_str());
        }
    }

    if (governor_enabled_) {
        const overlume::ros::QualityTransition transition =
            quality_governor_->record_render_ms(render_ms_);
        if (transition != overlume::ros::QualityTransition::NONE) {
            const uint32_t new_preset = quality_governor_->current_preset();
            overlume::set_quality(renderer_, new_preset);
            quality_ = static_cast<int>(new_preset);
            set_parameter(rclcpp::Parameter("quality", quality_));
            RCLCPP_WARN(
                get_logger(), "quality governor: %s -> preset %u (render_ms p95 over window)",
                transition == overlume::ros::QualityTransition::DROPPED ? "DROPPED" : "RECOVERED",
                new_preset);
        }
    }

    if (hud_enabled_ && overlays_visible_for_mode(render_mode)) {
        const overlume::HudColors hud_colors = overlume::get_hud_colors(renderer_);
        const overlume::ros::HudSnapshot hud_snapshot{scene.hud.speed_mps, scene.hud.active_mode};
        if (!overlume::ros::CompositeHud(
                frame_buf_.data(), static_cast<uint32_t>(out_width_),
                static_cast<uint32_t>(out_height_), hud_snapshot,
                overlume::ros::HudRgb{hud_colors.text_color[0], hud_colors.text_color[1],
                                      hud_colors.text_color[2]},
                overlume::ros::HudRgb{hud_colors.accent_color[0], hud_colors.accent_color[1],
                                      hud_colors.accent_color[2]},
                hud_colors.scale, hud_font_path_.c_str()) &&
            !hud_font_warned_) {
            RCLCPP_WARN(get_logger(),
                        "CompositeHud: failed to load/use font '%s' -- HUD not drawn this run",
                        hud_font_path_.c_str());
            hud_font_warned_ = true;
        }
    }

    if (environment_effectively_visible(render_mode, environment_enabled_) &&
        environment_attribution_ &&
        environment_source_uri_.find("materials=original") != std::string::npos &&
        overlume::environment_source_state(renderer_) ==
            overlume::EnvironmentSourceState::STREAMING) {
        const overlume::HudColors hud_colors = overlume::get_hud_colors(renderer_);
        if (!overlume::ros::DrawText(
                frame_buf_.data(), static_cast<uint32_t>(out_width_),
                static_cast<uint32_t>(out_height_), "3D Tiles data (c) Google", 8.0f,
                static_cast<float>(out_height_) - 8.0f,
                overlume::ros::HudRgb{hud_colors.text_color[0], hud_colors.text_color[1],
                                      hud_colors.text_color[2]},
                hud_colors.scale, hud_font_path_.c_str()) &&
            !environment_attribution_warned_) {
            RCLCPP_WARN(get_logger(),
                        "environment attribution: failed to load/use font '%s' -- "
                        "Google attribution not drawn this run",
                        hud_font_path_.c_str());
            environment_attribution_warned_ = true;
        }
    }

    if (callouts_enabled_ && overlays_visible_for_mode(render_mode) && scene.ego.valid != 0) {
        overlume::ros::Callout callout{};
        if (overlume::ros::BuildNearestCallout(renderer_, scene.alerts, scene.alert_count,
                                               scene.ego.position, callout)) {
            const overlume::HudColors hud_colors = overlume::get_hud_colors(renderer_);
            overlume::ros::DrawCallout(
                frame_buf_.data(), static_cast<uint32_t>(out_width_),
                static_cast<uint32_t>(out_height_), callout,
                overlume::ros::HudRgb{hud_colors.accent_color[0], hud_colors.accent_color[1],
                                      hud_colors.accent_color[2]},
                hud_colors.scale, hud_font_path_.c_str());
        }
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
    info_msg.k[0] = fy;
    info_msg.k[1] = 0;
    info_msg.k[2] = out_width_ / 2.0;
    info_msg.k[3] = 0;
    info_msg.k[4] = fy;
    info_msg.k[5] = out_height_ / 2.0;
    info_msg.k[6] = 0;
    info_msg.k[7] = 0;
    info_msg.k[8] = 1;
    pub_info_->publish(info_msg);

    publish_diagnostics();
}

void OverlumeNode::publish_diagnostics() {
    std::vector<overlume::ros::RowStats> rows;
    rows.reserve(hd_map_rows_.size() + dynamic_objects_rows_.size() + path_rows_.size() +
                 ogm_rows_.size() + collision_rows_.size() + generic_marker_rows_.size() +
                 point_cloud_rows_.size() + carpet_rows_.size());

    auto append_row = [&](const std::string& topic, const overlume::ros::AdapterStats& stats,
                          double timeout_sec) {
        overlume::ros::RowStats rs;
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

    auto msg = overlume::ros::BuildDiagnostics(rows, render_ms_);
    msg.status.push_back(overlume::ros::BuildHybridStatus(hybrid_starved_reason_));
    msg.header.stamp = now();
    pub_diagnostics_->publish(msg);
}

void OverlumeNode::teardown_active() { timer_.reset(); }

OverlumeNode::CallbackReturn OverlumeNode::on_deactivate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "on_deactivate() called.");
    teardown_active();
    pub_image_->on_deactivate();
    pub_info_->on_deactivate();
    pub_vcam_state_->on_deactivate();
    pub_ego_state_->on_deactivate();
    pub_diagnostics_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

OverlumeNode::CallbackReturn OverlumeNode::on_cleanup(const rclcpp_lifecycle::State&) {
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

OverlumeNode::CallbackReturn OverlumeNode::on_shutdown(const rclcpp_lifecycle::State&) {
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

}
