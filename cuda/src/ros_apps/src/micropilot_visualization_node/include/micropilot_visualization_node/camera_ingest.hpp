#pragma once
/** @file camera_ingest.hpp
 *  @brief VM-091 (unified-engine migration Task 2 Step 6): 6-camera ingest
 *  for the bowl -- per-camera image/CameraInfo subscriptions, the odometry
 *  twist ring buffer + rig_delta/compensate ego-motion math ported from
 *  micropilot_rendering_node/rendering_node.{hpp,cpp} and rewritten against
 *  Task 1's mpviz::CameraExtrinsics (no rendering_reprojector link), and the
 *  frame-sync gate's merged-node semantics (Task 2 Step 6): unlike the old
 *  node, this gate does not withhold uploads or rendering -- it only
 *  parameterizes the per-tick set_camera_motion_delta() re-alignment to a
 *  common reference time t_max.
 *
 *  Split into a pure, ROS-free bookkeeping class (IngestState -- info_ready
 *  gate, per-camera monotonic frame_id, per-camera last-image stamp) and a
 *  thin ROS wrapper (CameraIngest) that owns the subscriptions and calls the
 *  mpviz:: entry points, same "pure math class + ROS-owning wrapper" split
 *  as geo_anchor.hpp's GeoAnchor/GeoAnchorSolver -- IngestState (and the
 *  free functions below it) are what test_camera_ingest.cpp exercises
 *  directly, with no rclcpp node/spin and no GPU.
 */

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "visual_renderer/scene.h"

namespace micropilot::visualization_app
{

// ── Pure math: odometry twist buffer + rig-pose-delta integration ──────────
// Ported verbatim (signed-dt Euler integration, same 0.005s step target) from
// micropilot_rendering_node/rendering_node.cpp:562-604's twist_at()/
// rig_delta(), rewritten to take the twist deque as a plain parameter instead
// of reading a node member (so it needs no ROS type and no mutex to test).
struct StampedTwist
{
    double t;      // stamp (s)
    double vx, vy; // body-frame linear velocity (m/s), x-fwd / y-left
    double wz;     // body-frame yaw rate (rad/s), +z up
};

// Interpolated planar twist at stamp t (clamps to buffer ends); false if
// `twists` is empty.
bool twist_at(const std::deque<StampedTwist>& twists, double t, StampedTwist& out);

// Integrated planar rig pose delta over [t_from, t_ref]: pose of rig(t_ref)
// in the rig(t_from) frame (yaw th, position px/py). False if `twists` is
// empty or the span is degenerate (|t_ref - t_from| < 1e-4).
bool rig_delta(const std::deque<StampedTwist>& twists, double t_from, double t_ref, double& th,
               double& px, double& py);

// Builds the row-major 4x4 ego-motion-delta matrix scene.h's
// set_camera_motion_delta() consumes, for camera stamp t_cam against
// reference time t_ref: rotation block = Rz(th), translation = (px, py, 0),
// with (th, px, py) = rig_delta(t_cam, t_ref). Writes the identity matrix
// (no compensation) when rig_delta() returns false (no odometry, or t_cam
// == t_ref) -- this is what a camera with no odometry, or one whose stamp
// already equals t_max, gets.
void compensation_delta_4x4(const std::deque<StampedTwist>& twists, double t_cam, double t_ref,
                             double out_delta_row_major[16]);

// ── Pure math: extrinsics orthonormalization ────────────────────────────────
// Gram-Schmidt on R's three columns (right, down, fwd) -- bowl.mat's
// fragment shader reconstructs `down` as cross(fwd, right), exact only for
// an orthonormal basis; camera_extrinsics arrives as 12 hand-authored/
// calibration-derived floats, not guaranteed exactly orthonormal. Returns
// the corrected extrinsics (t unchanged) and, via `out_max_correction_rad`,
// the largest angular correction applied to any column (radians) -- the
// caller WARNs once if this exceeds a small threshold (see
// kOrthonormalizeWarnThresholdRad below).
constexpr double kOrthonormalizeWarnThresholdRad = 0.05; // ~3 degrees
mpviz::CameraExtrinsics OrthonormalizeExtrinsics(const mpviz::CameraExtrinsics& in,
                                                  double* out_max_correction_rad);

// ── Pure bookkeeping: info_ready gate + per-camera frame_id/stamp ──────────
class IngestState
{
public:
    // `extrinsics` (camera_count entries, already orthonormalized) is fixed
    // for this object's lifetime -- camera_extrinsics is a ROS param, not
    // something CameraInfo ever updates.
    IngestState(uint32_t camera_count, std::vector<mpviz::CameraExtrinsics> extrinsics);

    // CameraInfo arrival: records K/dist/width/height for `cam_idx`. Returns
    // true when the caller should (re)call set_bowl_config() -- either this
    // camera just completed the FIRST full set (info_ready flips
    // false->true for the last remaining camera) or, once already complete,
    // this call's K/dist/width/height differ from what's already stored (a
    // driver reconnect / live GUI extrinsics edit).
    bool record_camera_info(uint32_t cam_idx, const mpviz::CameraIntrinsics& in, uint32_t width,
                             uint32_t height);
    bool all_info_ready() const;
    uint32_t camera_count() const { return camera_count_; }
    const mpviz::CameraExtrinsics& extrinsics(uint32_t i) const { return cams_[i].extrinsics; }
    const mpviz::CameraIntrinsics& intrinsics(uint32_t i) const { return cams_[i].intrinsics; }
    uint32_t width(uint32_t i) const { return cams_[i].width; }
    uint32_t height(uint32_t i) const { return cams_[i].height; }

    // Image arrival bookkeeping: records the stamp and returns this camera's
    // new monotonic frame_id (starts at 1, never repeats) -- the value the
    // caller passes to set_camera_frame() (the sole dirty gate this node
    // relies on; Task 1's own frame_id-equality check inside the library is
    // only a backstop against a caller bug).
    uint64_t record_image_stamp(uint32_t cam_idx, double stamp_sec);
    bool has_stamp(uint32_t cam_idx) const { return cams_[cam_idx].has_stamp; }
    double stamp(uint32_t cam_idx) const { return cams_[cam_idx].stamp; }
    // Newest last-image stamp across every camera that has ever delivered
    // one -- the frame-sync gate's t_max (Task 2 Step 6). False if no
    // camera has delivered an image yet.
    bool newest_stamp(double& out_t_max) const;

private:
    struct PerCam
    {
        mpviz::CameraExtrinsics extrinsics{};
        mpviz::CameraIntrinsics intrinsics{};
        uint32_t width = 0, height = 0;
        bool info_ready = false;
        double stamp = 0.0;
        bool has_stamp = false;
        uint64_t frame_id = 0;
    };
    uint32_t camera_count_;
    std::vector<PerCam> cams_;
};

// ── ROS wrapper: subscriptions + the mpviz:: call sites ─────────────────────
class CameraIngest
{
public:
    // `extrinsics` is camera_count entries, rig-frame, from the
    // camera_extrinsics ROS param (mirrors rendering_node.cpp's own param,
    // not a tf2 lookup -- Global Constraints); orthonormalized here (Gram-
    // Schmidt, WARN once per camera if the correction exceeds
    // kOrthonormalizeWarnThresholdRad) before it reaches IngestState/
    // BowlConfig. Subscriptions are created immediately (on_configure-time,
    // same as this node's other sensor subs -- e.g. robot_speed_sub_/
    // gps_sub_ -- not gated behind on_activate()).
    CameraIngest(rclcpp_lifecycle::LifecycleNode* node, uint32_t camera_count,
                 std::vector<std::string> image_topics, std::vector<std::string> info_topics,
                 std::string odom_topic, std::vector<mpviz::CameraExtrinsics> extrinsics);

    // Renderer becomes available slightly after this object is constructed
    // (both are built in on_configure(), renderer first) -- calls before
    // set_renderer() safely no-op (image callback still records stamps/
    // frame_ids so the frame-sync gate has real data once the renderer
    // shows up).
    void set_renderer(mpviz::VisualRenderer* r) { renderer_ = r; }
    void set_bowl_enabled(bool enabled) { bowl_enabled_ = enabled; }
    // VM-091 gate close-out finding 3: max_sync_latency was declared/stored
    // on the node but never read anywhere -- update_motion_deltas() below
    // now compares each camera's (t_max - stamp) spread against this window
    // and THROTTLE-WARNs when it's exceeded (carried over from the old
    // node's frame-sync gate, rendering_node.cpp's own spread WARN), even
    // though the merged node's redefined gate semantics (Task 2 Step 6)
    // never withhold the render or the stale camera's texture for it.
    void set_max_sync_latency(double seconds) { max_sync_latency_ = seconds; }

    bool all_info_ready() const { return state_.all_info_ready(); }
    // Fills camera_count/extrinsics/intrinsics/cam_width/cam_height from
    // this ingest's own storage -- caller (VisualizationNode) fills the
    // remaining BowlConfig fields and calls set_bowl_config() itself, then
    // calls mark_bowl_config_applied() on success.
    void fill_bowl_intrinsics(std::vector<mpviz::CameraExtrinsics>& out_ext,
                               std::vector<mpviz::CameraIntrinsics>& out_in,
                               std::vector<uint32_t>& out_w, std::vector<uint32_t>& out_h) const;
    void mark_bowl_config_applied() { config_applied_ = true; info_dirty_ = false; }
    bool config_applied() const { return config_applied_; }
    // True once since the last mark_bowl_config_applied() call -- a NEW
    // CameraInfo changed some camera's K/dist/dims after info was already
    // complete. Consuming clears it (same "read resets" shape as every
    // other one-shot flag in this node, e.g. environment_warned_).
    bool consume_info_dirty();

    // Per render tick: computes t_max (newest last-image stamp across
    // cameras) and calls set_camera_motion_delta(r, i, ...) for every
    // configured camera -- identity when odometry is absent or a camera has
    // no stamp yet. No-op if the renderer/bowl_enabled_/config_applied_
    // gates aren't all satisfied, or no camera has delivered an image yet.
    void update_motion_deltas();

private:
    rclcpp_lifecycle::LifecycleNode* node_;
    IngestState state_;
    mpviz::VisualRenderer* renderer_ = nullptr;
    bool bowl_enabled_ = false;
    bool config_applied_ = false;
    bool info_dirty_ = false;
    double max_sync_latency_ = 0.12;

    std::string odom_topic_;
    std::deque<StampedTwist> twists_;
    std::mutex odom_mtx_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> img_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
};

}  // namespace micropilot::visualization_app
