#pragma once
/** @file rendering_node.hpp
 *  @brief LifecycleNode that wraps the micropilot_rendering CUDA reprojector.
 *
 *  Production note: camera extrinsics are read from a ROS parameter
 *  (`camera_extrinsics`) for determinism / smoke-test convenience.  A tf2
 *  lookup is the natural production alternative: wire `tf2_ros::Buffer` in
 *  `on_configure`, call `lookupTransform` in the timer callback, and populate
 *  `CameraParams.R/t` from `geometry_msgs::msg::TransformStamped`.
 */

#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rendering_reprojector/mesh_loader.hpp"
#include "rendering_reprojector/reprojector.hpp"
#include "rendering_reprojector/types.hpp"

#include "micropilot_rendering_node/srv/set_virtual_cam.hpp"

namespace micropilot::rendering_app
{

/// A virtual-camera placement as look-points in the rig frame (x-fwd, y-left,
/// z-up). The pose rotation is rebuilt from these via look_at(), so eased
/// preset switches only need to interpolate the points — never rotations.
struct LookPoint
{
    float eye[3];
    float target[3];
};

class RenderingNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit RenderingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RenderingNode() override = default;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    void timer_callback();
    void teardown_active();
    /// Rasterize the robot mesh from each real camera pose -> self-view masks
    /// (body pixels skipped in reprojection; blind-zone fill covers them).
    void generate_self_masks();

    // robot proxy mesh (kept for self-mask generation) + one-shot mask state
    micropilot::rendering::RobotMesh robot_mesh_;
    bool have_robot_mesh_ = false;
    bool self_masks_done_ = false;
    bool self_view_masks_ = false;

    // ── virtual-camera presets / eased switching ─────────────────────────────
    using SetVirtualCam = micropilot_rendering_node::srv::SetVirtualCam;
    /// Build a camera pose (R columns = right/down/fwd, t = eye) from look-points.
    static void look_at(const float eye[3], const float target[3], float R_out[9]);
    /// Apply a look-point to vcam_ (rebuilds R/t; keeps K/width/height).
    void apply_lookpoint(const LookPoint& lp);
    /// Advance the smoothstep tween one timer tick and update vcam_.
    void advance_tween();
    void on_set_virtual_cam(const std::shared_ptr<SetVirtualCam::Request> req,
                            std::shared_ptr<SetVirtualCam::Response> res);
    /// Handle ~/set_look: 6 floats [eye xyz | target xyz] applied immediately
    /// (no tween) — the generic runtime pose input used for free-look orbiting.
    void on_set_look(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Preset table: [0]=config [1]=reverse_follow [2]=left_side [3]=right_side
    // [4]=top_down. Index i is preset (i+1) in the service request.
    std::array<LookPoint, 5> presets_{};
    static constexpr std::array<const char*, 5> kPresetNames{
        "config", "reverse_follow", "left_side", "right_side", "top_down"};
    LookPoint cur_{}, src_{}, dst_{};
    double tween_t_{1.0};  // [0,1]; 1.0 = settled on dst_. Eased per timer tick.
    int active_preset_{1};  // 1-5 = preset in service numbering; 0 = free look

    // ── live parameter tuning (GUI panel via WS bridge) ──────────────────────
    // Runtime updates for render-tunable params + camera extrinsics; registered
    // in on_configure AFTER the initial declares so it only sees real updates.
    rcl_interfaces::msg::SetParametersResult on_params(
        const std::vector<rclcpp::Parameter>& params);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    rclcpp::Service<SetVirtualCam>::SharedPtr set_vcam_srv_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_look_sub_;
    // vcam telemetry: [eye xyz | target xyz | active_preset], one per render tick.
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pub_vcam_state_;

    // ── configuration ────────────────────────────────────────────────────────
    int n_cameras_{4};
    int out_width_{640};
    int out_height_{480};
    // Render only when all cameras have a new frame whose stamps fall within this
    // window (seconds). Prevents stitching temporally-misaligned async frames.
    double max_sync_latency_{0.12};
    // ── ego-motion time compensation ─────────────────────────────────────────
    // The real cameras free-run with stable phase offsets (~80 ms spread on
    // m2o1). While driving, each camera then projects the ground from where the
    // robot WAS at its own stamp — up to ~0.5 m apart — which reads as a static
    // "calibration" misalignment. If odom_topic is set, each camera's extrinsic
    // is advanced by the rig's odometry twist over (t_ref - t_cam) before
    // upload, so all cameras project from a common reference time.
    std::string odom_topic_;
    // Fill color for genuinely-unseen pixels (above the bowl rim) — sky, not black,
    // so the teleop driving view shows a natural horizon. RGB in [0,1].
    float sky_color_[3]{0.53f, 0.70f, 0.92f};
    micropilot::rendering::BowlParams bowl_{6.0f, 0.08f, 20.0f};
    micropilot::rendering::CameraParams vcam_{};

    std::vector<micropilot::rendering::CameraParams> cam_params_;

    // ── runtime ──────────────────────────────────────────────────────────────
    std::unique_ptr<micropilot::rendering::Reprojector> reprojector_;

    struct PerCamera
    {
        std::optional<cv::Mat> image;        // float32 NHWC staging
        bool info_ready{false};
        rclcpp::Time stamp;                  // header stamp of the staged image
        bool have_new{false};                // a new frame arrived since last render
        std::unique_ptr<std::mutex> mtx;     // unique_ptr keeps vector movable

        PerCamera() : mtx(std::make_unique<std::mutex>()) {}
    };
    std::vector<PerCamera> per_cam_;

    // subscriptions (active only while ACTIVE)
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> img_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;

    // ── odometry twist buffer (ego-motion time compensation) ────────────────
    // Small ring of recent body twists; twist_at() interpolates the planar
    // (vx, vy, wz) twist at an arbitrary stamp. Guarded by odom_mtx_.
    struct StampedTwist
    {
        double t;         // stamp (s)
        double vx, vy;    // body-frame linear velocity (m/s), x-fwd / y-left
        double wz;        // body-frame yaw rate (rad/s), +z up
    };
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::deque<StampedTwist> twists_;
    std::mutex odom_mtx_;
    /// Interpolated planar twist at stamp t (clamps to buffer ends); false if empty.
    bool twist_at(double t, StampedTwist& out);
    /// Integrated planar rig pose delta over [t_from, t_ref]: pose of rig(t_ref)
    /// in the rig(t_from) frame (yaw th, position px/py). False if no odometry.
    bool rig_delta(double t_from, double t_ref, double& th, double& px, double& py);
    /// cam extrinsic advanced by the rig motion over [t_cam, t_ref].
    micropilot::rendering::CameraParams compensate(
        const micropilot::rendering::CameraParams& cp, double t_cam, double t_ref);

    // ── lidar point cloud (hybrid rendering) ─────────────────────────────────
    // If pointcloud_topic_ is set, points are transformed into the rig frame
    // via pointcloud_tf_, colorized on the GPU from the camera images, and
    // splatted over the bowl (render_hybrid) — parallax-correct for objects
    // above the ground that the bowl alone smears.
    std::string pointcloud_topic_;
    float pointcloud_tf_[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};  // [R(9)|t(3)]
    int splat_radius_{2};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    std::vector<float> cloud_pts_;  // rig-frame xyz flat; guarded by cloud_mtx_
    double cloud_stamp_{0.0};       // header stamp (s) of cloud_pts_
    std::mutex cloud_mtx_;
    // Runtime view switch (~/set_render_mode): 1 = bowl-only, 2 = pointcloud
    // hybrid (falls back to bowl when no cloud is buffered). Default hybrid.
    int render_mode_{2};
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_mode_sub_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::TimerBase::SharedPtr timer_;

    // track which cameras had an image update since last upload
    std::vector<bool> img_dirty_;
    // a complete synced set has been uploaded — from then on the node renders
    // EVERY tick (re-rendering the cached set when no new frames arrive, so a
    // paused bag stays tunable live from the GUI)
    bool have_set_ = false;
    // header stamps of the currently-uploaded set (ego-motion compensation
    // must reference THESE, not the latest arrivals, when re-rendering cached
    // images while a new set is still assembling)
    std::vector<rclcpp::Time> up_stamps_;

    // explicit per-camera topic names (size n_cameras_); empty => default
    // "/camera/camN/image_raw" + "/camera/camN/camera_info" pattern.
    std::vector<std::string> image_topics_;
    std::vector<std::string> info_topics_;
};

}  // namespace micropilot::rendering_app
