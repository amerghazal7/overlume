// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

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

#include "overlume/api.h"
#include "overlume/scene.h"

#include "overlume_ros/adapters/collision.hpp"
#include "overlume_ros/adapters/dynamic_objects.hpp"
#include "overlume_ros/adapters/generic_marker.hpp"
#include "overlume_ros/adapters/hd_map.hpp"
#include "overlume_ros/adapters/ogm.hpp"
#include "overlume_ros/adapters/path.hpp"
#include "overlume_ros/adapters/point_cloud.hpp"
#include "overlume_ros/adapters/tf_axes.hpp"
#include "overlume_ros/adapters/trajectory_carpet.hpp"
#include "overlume_ros/callouts.hpp"
#include "overlume_ros/camera_ingest.hpp"
#include "overlume_ros/diagnostics.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/geo_anchor.hpp"
#include "overlume_ros/hud_overlay.hpp"
#include "overlume_ros/lidar_colorize.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/quality_governor.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume_ros/tf_adapter.hpp"
#include "overlume_ros/vcam.hpp"

namespace overlume::ros {

class OverlumeNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit OverlumeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~OverlumeNode() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    void timer_callback();
    rcl_interfaces::msg::SetParametersResult on_params(
        const std::vector<rclcpp::Parameter>& params);
    void teardown_active();
    void destroy_renderer_if_any();
    void publish_diagnostics();
    void apply_environment_visibility();

    std::unique_ptr<Vcam> vcam_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pub_vcam_state_;

    int out_width_{1280};
    int out_height_{720};
    int quality_{2};
    int initial_mode_{1};
    overlume::CameraPose pose_{};

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_mode_sub_;

    static constexpr int kRenderModeBowl = 1;
    static constexpr int kRenderModeHybrid = 2;
    static constexpr int kRenderModeFreeLook = 3;
    int render_mode_{kRenderModeFreeLook};

    bool layer_surround_stitching_{false};
    std::string surround_stitching_profile_{"bowl"};
    bool hybrid_cloud_consumed() const {
        return render_mode_ == kRenderModeHybrid ||
               (render_mode_ == kRenderModeFreeLook && layer_surround_stitching_ &&
                surround_stitching_profile_ == "hybrid");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr theme_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<overlume::ros::TfAdapter> tf_adapter_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr robot_speed_sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pub_ego_state_;
    static constexpr double kTimerPeriodSec = 0.033;
    double sim_clock_sec_{0.0};

    struct HdMapRow {
        std::unique_ptr<overlume::ros::HdMapAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::unique_ptr<FrameTransformer> frame_transformer_;
    std::vector<HdMapRow> hd_map_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr> hd_map_subs_;

    struct DynamicObjectsRow {
        std::unique_ptr<overlume::ros::DynamicObjectsAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    overlume::ros::ClassInferenceTable class_inference_;
    std::vector<DynamicObjectsRow> dynamic_objects_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        dynamic_objects_subs_;

    struct PathRow {
        std::unique_ptr<overlume::ros::PathAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::vector<PathRow> path_rows_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subs_;

    struct OgmRow {
        std::unique_ptr<overlume::ros::OgmAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::vector<OgmRow> ogm_rows_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> ogm_grid_subs_;
    std::vector<rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr>
        ogm_update_subs_;

    struct CollisionRow {
        std::unique_ptr<overlume::ros::CollisionAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::vector<CollisionRow> collision_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        collision_subs_;

    struct GenericMarkerRow {
        std::unique_ptr<overlume::ros::GenericMarkerAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::vector<GenericMarkerRow> generic_marker_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
        generic_marker_subs_;

    struct PointCloudRow {
        std::unique_ptr<overlume::ros::PointCloudAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::vector<PointCloudRow> point_cloud_rows_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_subs_;

    struct CarpetRow {
        std::unique_ptr<overlume::ros::TrajectoryCarpetAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::vector<CarpetRow> carpet_rows_;
    std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr> carpet_subs_;

    std::vector<std::unique_ptr<overlume::ros::TfAxesAdapter>> tf_axes_rows_;

    std::unique_ptr<GeoAnchorSolver> geo_anchor_solver_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    bool geo_anchor_logged_{false};

    bool environment_enabled_{true};
    std::string environment_chunks_dir_;
    bool environment_warned_{false};
    std::string environment_source_uri_;
    std::string environment_tile_cache_dir_;
    bool environment_follow_terrain_{true};
    double environment_ground_bias_m_{0.3};
    bool environment_replaces_ground_{true};
    double environment_max_tilt_deg_{2.0};
    double environment_brightness_{1.0};
    double environment_tile_radius_m_{700.0};
    bool environment_fallback_warned_{false};
    bool environment_attribution_{true};
    bool environment_attribution_warned_{false};
    std::string environment_own_asset_uri_;

    SceneAssembly scene_asm_;

    overlume::VisualRenderer* renderer_{nullptr};
    std::vector<uint8_t> frame_buf_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;

    double render_ms_{0.0};
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        pub_diagnostics_;

    bool governor_enabled_{false};
    std::unique_ptr<overlume::ros::QualityGovernor> quality_governor_;

    std::string hud_font_path_;
    bool hud_font_warned_{false};
    bool hud_enabled_{true};

    bool callouts_enabled_{true};

    bool layer_objects_{true};
    bool layer_paths_{true};
    bool layer_map_elements_{true};
    bool layer_grids_{true};
    bool layer_alerts_{true};
    bool layer_markers_{true};
    bool layer_point_clouds_{true};
    bool layer_trajectory_carpet_{true};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr layer_param_cb_;

    bool bowl_enabled_{false};
    bool bowl_mode_warned_{false};
    double max_sync_latency_{0.12};
    double bowl_R0_{6.0}, bowl_k_{0.08}, bowl_Rmax_{20.0};
    double feather_margin_{30.0};
    bool fill_blind_zone_{false};
    bool exposure_match_{false};
    bool bowl_config_dirty_{false};
    float sky_color_[3]{0.53f, 0.70f, 0.92f};
    float bowl_exposure_compensation_{1.56f};
    std::unique_ptr<CameraIngest> camera_ingest_;

    bool hybrid_enabled_{false};
    int splat_radius_{3};
    std::string hybrid_starved_reason_;
    float hybrid_splat_px() const { return 2.0f * splat_radius_ + 1.0f; }
    std::string pointcloud_topic_;
    float pointcloud_tf_[12]{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    std::mutex cloud_mtx_;
    std::vector<overlume::Vec3> cloud_pts_rig_;
    double cloud_stamp_{0.0};   // header stamp of cloud_pts_rig_ (cloud_mtx_)
    double cloud_rx_sec_{0.0};  // sim_clock_sec_ at receipt (cloud_mtx_)
    bool hybrid_row_suppressed_{false};

    rclcpp::TimerBase::SharedPtr timer_;
};

}
