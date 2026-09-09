#include "micropilot_visualization_node/adapters/trajectory_carpet.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace mpviz_node
{
namespace
{

// visualization_msgs/msg/Marker.msg action + type constants -- not worth a
// dependency on the generated enum names for values used once each (same
// convention as collision.cpp/generic_marker.cpp).
constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kTypeTriangleList = 11;

bool HasNan(double x, double y, double z)
{
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
}

// A marker's own pose is RELATIVE to the header frame -- same pair as every
// other adapter's own copy (hd_map.cpp/dynamic_objects.cpp/collision.cpp/
// generic_marker.cpp; each file keeps its own rather than a shared header).
bool MarkerPoseIsIdentity(const geometry_msgs::msg::Pose& p)
{
    constexpr double kEps = 1e-12;
    return std::abs(p.position.x) < kEps && std::abs(p.position.y) < kEps &&
           std::abs(p.position.z) < kEps && std::abs(p.orientation.x) < kEps &&
           std::abs(p.orientation.y) < kEps && std::abs(p.orientation.z) < kEps &&
           std::abs(p.orientation.w - 1.0) < kEps;
}

bool MarkerPoseHasNan(const geometry_msgs::msg::Pose& p)
{
    return std::isnan(p.position.x) || std::isnan(p.position.y) || std::isnan(p.position.z) ||
           std::isnan(p.orientation.x) || std::isnan(p.orientation.y) ||
           std::isnan(p.orientation.z) || std::isnan(p.orientation.w);
}

}  // namespace

TrajectoryCarpetAdapter::TrajectoryCarpetAdapter(
    const ProfileRow& row, const micropilot::visualization_app::FrameTransformer& tf)
    : row_(row), tf_(tf)
{
}

void TrajectoryCarpetAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg,
                                     double sim_time_sec)
{
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    // ONE lookup for the whole message -- same convention as every other
    // marker adapter in this node.
    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform))
    {
        ++stats_.dropped_no_tf;
        return;  // whole message dropped; previously-stored carpet stays
    }

    for (const auto& m : msg.markers)
    {
        if (m.action == kActionDeleteAll)
        {
            has_data_ = false;
            storage_.clear();
            continue;
        }
        if (m.action != kActionAdd) continue;  // ignore DELETE(ns,id) -- one persistent marker

        if (m.type != kTypeTriangleList || m.points.size() < 3 || m.points.size() % 3 != 0)
        {
            ++stats_.dropped_malformed;
            continue;
        }

        const bool identity_pose = MarkerPoseIsIdentity(m.pose);
        tf2::Transform marker_tf;
        if (!identity_pose)
        {
            if (MarkerPoseHasNan(m.pose))
            {
                ++stats_.dropped_malformed;
                continue;
            }
            tf2::Quaternion q(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                              m.pose.orientation.w);
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();  // rviz parity
            marker_tf = tf2::Transform(
                q, tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
        }

        // A length mismatch means the WHOLE colors[] array is suspect (not
        // a per-point concern) -- every point falls back to the alpha==0
        // sentinel together, same "whole-message fallback" rule this file's
        // header comment states.
        const bool per_point_colors = m.colors.size() == m.points.size();

        std::vector<mpviz::PointCloudPoint> pts;
        pts.reserve(m.points.size());
        bool ok = true;
        for (size_t i = 0; i < m.points.size(); ++i)
        {
            const auto& p = m.points[i];
            const tf2::Vector3 local(p.x, p.y, p.z);
            const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
            const tf2::Vector3 tp = xform * posed;
            const double z = tf_.flatten_z() ? 0.0 : tp.z();
            if (HasNan(tp.x(), tp.y(), z))
            {
                ok = false;
                break;
            }
            mpviz::PointCloudPoint pt{};
            pt.position = {tp.x(), tp.y(), z};
            // colors[] alpha is documented "not yet used" (Marker.msg); force
            // 255 so a supplied colors[i] always reads as "real color", same
            // convention GenericMarkerAdapter's fan_colors already uses.
            pt.rgba = per_point_colors
                          ? PackRgba(static_cast<uint8_t>(m.colors[i].r * 255.0f + 0.5f),
                                     static_cast<uint8_t>(m.colors[i].g * 255.0f + 0.5f),
                                     static_cast<uint8_t>(m.colors[i].b * 255.0f + 0.5f), 255)
                          : 0u;
            pts.push_back(pt);
        }
        if (!ok)
        {
            ++stats_.dropped_malformed;
            continue;
        }

        // REPLACES wholesale, never appends/merges -- same contract as
        // PathAdapter (this file's own header comment).
        storage_ = std::move(pts);
        has_data_ = true;
        last_update_sec_ = sim_time_sec;
    }

    stats_.last_msg_sec = sim_time_sec;
}

void TrajectoryCarpetAdapter::fill(micropilot::visualization_app::SceneAssembly& out) const
{
    if (!has_data_) return;
    mpviz::TrajectoryCarpet tc{};
    tc.points = storage_.data();
    tc.point_count = static_cast<uint32_t>(storage_.size());
    tc.last_update_sec = last_update_sec_;
    out.trajectory_carpets.push_back(tc);
}

}  // namespace mpviz_node
