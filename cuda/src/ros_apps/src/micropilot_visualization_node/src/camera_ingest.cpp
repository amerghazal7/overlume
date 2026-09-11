/** @file camera_ingest.cpp
 *  @brief See camera_ingest.hpp. VM-091 Task 2 Step 6.
 */
#include "micropilot_visualization_node/camera_ingest.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include <cv_bridge/cv_bridge.h>

namespace micropilot::visualization_app
{

// ── Pure math: odometry twist buffer + rig-pose-delta integration ──────────
// Ported from micropilot_rendering_node/rendering_node.cpp:562-604, read in
// full before porting -- unchanged algorithm, rewritten against a plain
// std::deque<StampedTwist> parameter instead of a node member + its own
// mutex (odom_mtx_ locking happens once at the CameraIngest call site, on a
// snapshot, not per twist_at() call).
bool twist_at(const std::deque<StampedTwist>& twists, double t, StampedTwist& out)
{
    if (twists.empty()) return false;
    if (t <= twists.front().t) { out = twists.front(); return true; }
    if (t >= twists.back().t) { out = twists.back(); return true; }
    for (size_t i = 1; i < twists.size(); ++i)
    {
        if (twists[i].t < t) continue;
        const auto& a = twists[i - 1];
        const auto& b = twists[i];
        const double w = (t - a.t) / std::max(b.t - a.t, 1e-9);
        out.t = t;
        out.vx = a.vx + (b.vx - a.vx) * w;
        out.vy = a.vy + (b.vy - a.vy) * w;
        out.wz = a.wz + (b.wz - a.wz) * w;
        return true;
    }
    out = twists.back();
    return true;
}

bool rig_delta(const std::deque<StampedTwist>& twists, double t_from, double t_ref, double& th,
               double& px, double& py)
{
    th = px = py = 0.0;
    const double span = t_ref - t_from;
    if (std::abs(span) < 1e-4) return false;
    const int n = std::max(1, static_cast<int>(std::ceil(std::abs(span) / 0.005)));
    const double dt = span / n;
    for (int i = 0; i < n; ++i)
    {
        StampedTwist tw;
        if (!twist_at(twists, t_from + (i + 0.5) * dt, tw)) return false;
        const double c = std::cos(th), s = std::sin(th);
        px += (c * tw.vx - s * tw.vy) * dt;
        py += (s * tw.vx + c * tw.vy) * dt;
        th += tw.wz * dt;
    }
    return true;
}

void compensation_delta_4x4(const std::deque<StampedTwist>& twists, double t_cam, double t_ref,
                             double out_delta_row_major[16])
{
    double th, px, py;
    if (!rig_delta(twists, t_cam, t_ref, th, px, py))
    {
        const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        std::memcpy(out_delta_row_major, I, sizeof(I));
        return;
    }
    const double c = std::cos(th), s = std::sin(th);
    // Rows 0/1 = Rz(th)'s rows, translation (px, py); row 2 identity-z; row
    // 3 = [0,0,0,1] -- see camera_ingest.hpp's own derivation comment: this
    // is exactly the matrix bowl.cpp's apply_delta_rotation_transposed()
    // expects (its transpose undoes Rz(th), matching
    // rendering_node.cpp's compensate()).
    double m[16] = {c, -s, 0, px, s, c, 0, py, 0, 0, 1, 0, 0, 0, 0, 1};
    std::memcpy(out_delta_row_major, m, sizeof(m));
}

// ── Pure math: extrinsics orthonormalization ────────────────────────────────
namespace
{
using Vec = std::array<double, 3>;
Vec vsub(Vec a, const Vec& b, double s)
{
    a[0] -= s * b[0];
    a[1] -= s * b[1];
    a[2] -= s * b[2];
    return a;
}
Vec vscale(Vec a, double s)
{
    a[0] *= s;
    a[1] *= s;
    a[2] *= s;
    return a;
}
double vdot(const Vec& a, const Vec& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double vnorm(const Vec& a) { return std::sqrt(vdot(a, a)); }
Vec vcross(const Vec& a, const Vec& b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double angle_between(const Vec& orig, const Vec& corrected_unit)
{
    const double n = vnorm(orig);
    if (n < 1e-12) return 0.0;
    const double d = std::clamp(vdot(vscale(orig, 1.0 / n), corrected_unit), -1.0, 1.0);
    return std::acos(d);
}
}  // namespace

mpviz::CameraExtrinsics OrthonormalizeExtrinsics(const mpviz::CameraExtrinsics& in,
                                                  double* out_max_correction_rad)
{
    // R columns = (right, down, fwd) -- bowl_projection.hpp's own convention.
    const Vec right{in.R[0], in.R[3], in.R[6]};
    const Vec down{in.R[1], in.R[4], in.R[7]};
    const Vec fwd{in.R[2], in.R[5], in.R[8]};

    const double right_n = vnorm(right);
    const Vec right_u = right_n > 1e-12 ? vscale(right, 1.0 / right_n) : Vec{1, 0, 0};

    Vec down_orth = vsub(down, right_u, vdot(down, right_u));
    double down_n = vnorm(down_orth);
    Vec down_u = down_n > 1e-12 ? vscale(down_orth, 1.0 / down_n) : vcross(Vec{0, 0, 1}, right_u);

    // fwd reconstructed via the cross product, not a second Gram-Schmidt
    // subtraction -- guarantees an EXACT right-handed orthonormal triple
    // (right x down = fwd, matching bowl.mat's own down = cross(fwd,right)
    // reconstruction cyclically) rather than one merely close to it.
    Vec fwd_u = vcross(right_u, down_u);
    const double fwd_n = vnorm(fwd_u);
    if (fwd_n > 1e-12) fwd_u = vscale(fwd_u, 1.0 / fwd_n);

    if (out_max_correction_rad != nullptr)
    {
        *out_max_correction_rad = std::max({angle_between(right, right_u), angle_between(down, down_u),
                                             angle_between(fwd, fwd_u)});
    }

    mpviz::CameraExtrinsics out = in;
    out.R[0] = right_u[0];
    out.R[3] = right_u[1];
    out.R[6] = right_u[2];
    out.R[1] = down_u[0];
    out.R[4] = down_u[1];
    out.R[7] = down_u[2];
    out.R[2] = fwd_u[0];
    out.R[5] = fwd_u[1];
    out.R[8] = fwd_u[2];
    return out;
}

// ── IngestState ──────────────────────────────────────────────────────────────
IngestState::IngestState(uint32_t camera_count, std::vector<mpviz::CameraExtrinsics> extrinsics)
    : camera_count_(camera_count), cams_(camera_count)
{
    for (uint32_t i = 0; i < camera_count_ && i < extrinsics.size(); ++i)
        cams_[i].extrinsics = extrinsics[i];
}

bool IngestState::record_camera_info(uint32_t cam_idx, const mpviz::CameraIntrinsics& in,
                                      uint32_t width, uint32_t height)
{
    if (cam_idx >= camera_count_) return false;
    PerCam& c = cams_[cam_idx];
    const bool all_ready_before = all_info_ready();
    const bool changed =
        c.width != width || c.height != height || std::memcmp(&c.intrinsics, &in, sizeof(in)) != 0;
    c.intrinsics = in;
    c.width = width;
    c.height = height;
    c.info_ready = true;
    // Not everything was ready before this call -- the only question is
    // whether THIS call completes the set (this camera's own change is
    // irrelevant the first time; there's nothing to compare against yet).
    if (!all_ready_before) return all_info_ready();
    // Every camera was already ready -- a re-bake is warranted only if this
    // message actually changed something.
    return changed;
}

bool IngestState::all_info_ready() const
{
    if (camera_count_ == 0) return false;
    for (const auto& c : cams_)
        if (!c.info_ready) return false;
    return true;
}

uint64_t IngestState::record_image_stamp(uint32_t cam_idx, double stamp_sec)
{
    if (cam_idx >= camera_count_) return 0;
    PerCam& c = cams_[cam_idx];
    c.stamp = stamp_sec;
    c.has_stamp = true;
    return ++c.frame_id;
}

bool IngestState::newest_stamp(double& out_t_max) const
{
    bool any = false;
    double best = 0.0;
    for (const auto& c : cams_)
    {
        if (!c.has_stamp) continue;
        if (!any || c.stamp > best) best = c.stamp;
        any = true;
    }
    if (any) out_t_max = best;
    return any;
}

void IngestState::store_rgb(uint32_t cam_idx, const uint8_t* data, uint32_t width, uint32_t height)
{
    if (cam_idx >= camera_count_ || data == nullptr) return;
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
    cams_[cam_idx].rgb.assign(data, data + n);
}

const uint8_t* IngestState::rgb(uint32_t cam_idx) const
{
    if (cam_idx >= camera_count_ || cams_[cam_idx].rgb.empty()) return nullptr;
    return cams_[cam_idx].rgb.data();
}

// ── CameraIngest ─────────────────────────────────────────────────────────────
namespace
{
std::vector<mpviz::CameraExtrinsics> orthonormalize_all(rclcpp_lifecycle::LifecycleNode* node,
                                                          const std::vector<mpviz::CameraExtrinsics>& in)
{
    std::vector<mpviz::CameraExtrinsics> out(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        double max_corr = 0.0;
        out[i] = OrthonormalizeExtrinsics(in[i], &max_corr);
        if (max_corr > kOrthonormalizeWarnThresholdRad)
        {
            RCLCPP_WARN(node->get_logger(),
                        "camera_extrinsics[%zu]: R was not orthonormal (correction %.4f rad, "
                        "~%.2f deg) -- Gram-Schmidt-corrected before use",
                        i, max_corr, max_corr * 180.0 / M_PI);
        }
    }
    return out;
}
}  // namespace

CameraIngest::CameraIngest(rclcpp_lifecycle::LifecycleNode* node, uint32_t camera_count,
                            std::vector<std::string> image_topics,
                            std::vector<std::string> info_topics, std::string odom_topic,
                            std::vector<mpviz::CameraExtrinsics> extrinsics)
    : node_(node),
      state_(camera_count, orthonormalize_all(node, extrinsics)),
      odom_topic_(std::move(odom_topic))
{
    img_subs_.resize(camera_count);
    info_subs_.resize(camera_count);
    for (uint32_t i = 0; i < camera_count; ++i)
    {
        img_subs_[i] = node_->create_subscription<sensor_msgs::msg::Image>(
            image_topics[i], rclcpp::SensorDataQoS(),
            [this, i](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                // bowl_enabled_ is the STANDING disable knob -- false means
                // this callback does no cv_bridge conversion work at all,
                // not merely skips the upload.
                if (!bowl_enabled_) return;
                cv_bridge::CvImagePtr cv_img;
                try
                {
                    cv_img = cv_bridge::toCvCopy(msg, "rgb8");
                }
                catch (const cv_bridge::Exception& e)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "cv_bridge error cam %u: %s", i, e.what());
                    return;
                }
                const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
                const uint64_t frame_id = state_.record_image_stamp(i, stamp_sec);
                const uint32_t w = static_cast<uint32_t>(cv_img->image.cols);
                const uint32_t h = static_cast<uint32_t>(cv_img->image.rows);
                // VM-094 (Task 5): retain a copy for lidar_colorize.hpp,
                // independent of whether a renderer is attached yet -- a
                // WARN-once-free no-op copy when hybrid rendering is off
                // (the common case, STANDING disable knob).
                if (hybrid_enabled_) state_.store_rgb(i, cv_img->image.data, w, h);
                if (renderer_ == nullptr) return;
                // Release-callback set_camera_frame (Decision resolution 1):
                // hand Filament the SAME buffer cv_bridge already converted
                // into -- one copy total (the toCvCopy conversion), not two.
                // The heap-allocated CvImagePtr copy is just a refcount bump
                // that keeps that buffer alive until Filament's release
                // fires -- ON FILAMENT'S OWN THREAD (scene.h's contract), so
                // a plain `delete` (dropping the shared_ptr, which may free
                // the underlying cv::Mat) is safe: it calls nothing back
                // into this library.
                auto* owned = new cv_bridge::CvImagePtr(cv_img);
                mpviz::set_camera_frame(
                    renderer_, i, cv_img->image.data, w, h, frame_id,
                    [](void*, size_t, void* user)
                    { delete static_cast<cv_bridge::CvImagePtr*>(user); },
                    owned);
            });

        info_subs_[i] = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topics[i], rclcpp::SensorDataQoS(),
            [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                mpviz::CameraIntrinsics in{};
                in.fx = msg->k[0];
                in.fy = msg->k[4];
                in.cx = msg->k[2];
                in.cy = msg->k[5];
                for (size_t j = 0; j < 5; ++j) in.dist[j] = j < msg->d.size() ? msg->d[j] : 0.0;
                if (state_.record_camera_info(i, in, msg->width, msg->height)) info_dirty_ = true;
            });
    }

    if (!odom_topic_.empty())
    {
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                StampedTwist tw;
                tw.t = rclcpp::Time(msg->header.stamp).seconds();
                tw.vx = msg->twist.twist.linear.x;
                tw.vy = msg->twist.twist.linear.y;
                tw.wz = msg->twist.twist.angular.z;
                std::lock_guard<std::mutex> lk(odom_mtx_);
                twists_.push_back(tw);
                while (!twists_.empty() && tw.t - twists_.front().t > 2.0) twists_.pop_front();
            });
    }
}

void CameraIngest::fill_bowl_intrinsics(std::vector<mpviz::CameraExtrinsics>& out_ext,
                                         std::vector<mpviz::CameraIntrinsics>& out_in,
                                         std::vector<uint32_t>& out_w,
                                         std::vector<uint32_t>& out_h) const
{
    const uint32_t n = state_.camera_count();
    out_ext.resize(n);
    out_in.resize(n);
    out_w.resize(n);
    out_h.resize(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        out_ext[i] = state_.extrinsics(i);
        out_in[i] = state_.intrinsics(i);
        out_w[i] = state_.width(i);
        out_h[i] = state_.height(i);
    }
}

void CameraIngest::fill_camera_rgb_buffers(std::vector<const uint8_t*>& out) const
{
    const uint32_t n = state_.camera_count();
    out.resize(n);
    for (uint32_t i = 0; i < n; ++i) out[i] = state_.rgb(i);
}

bool CameraIngest::consume_info_dirty()
{
    if (!info_dirty_) return false;
    info_dirty_ = false;
    return true;
}

void CameraIngest::update_motion_deltas()
{
    if (renderer_ == nullptr || !bowl_enabled_ || !config_applied_) return;
    double t_max;
    if (!state_.newest_stamp(t_max)) return;
    std::deque<StampedTwist> twists_snapshot;
    {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        twists_snapshot = twists_;
    }
    // VM-091 gate close-out finding 3: max_sync_latency was stored on the
    // node but never read anywhere in this file. A camera whose last-image
    // stamp is staler than max_sync_latency_ behind t_max keeps its last-
    // uploaded texture (no upload happens here regardless -- this loop only
    // ever calls set_camera_motion_delta, never set_camera_frame) AND keeps
    // being delta-compensated to t_max below, same as every other camera --
    // the merged node's redefined gate semantics (Task 2 Step 6) never
    // withhold either for it. The one carryover from the old node's gate is
    // this THROTTLED WARN when the spread exceeds the window.
    double max_spread = 0.0;
    for (uint32_t i = 0; i < state_.camera_count(); ++i)
    {
        if (!state_.has_stamp(i)) continue;
        max_spread = std::max(max_spread, t_max - state_.stamp(i));
    }
    if (max_spread > max_sync_latency_)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                              "camera bowl: stamp spread %.3fs exceeds max_sync_latency %.3fs -- "
                              "stalest camera(s) still delta-compensated to t_max, not withheld",
                              max_spread, max_sync_latency_);
    }
    for (uint32_t i = 0; i < state_.camera_count(); ++i)
    {
        double delta[16];
        if (state_.has_stamp(i))
        {
            compensation_delta_4x4(twists_snapshot, state_.stamp(i), t_max, delta);
        }
        else
        {
            const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            std::memcpy(delta, I, sizeof(I));
        }
        mpviz::set_camera_motion_delta(renderer_, i, delta);
    }
}

}  // namespace micropilot::visualization_app
