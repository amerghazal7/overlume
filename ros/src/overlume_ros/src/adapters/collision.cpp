#include "overlume_ros/adapters/collision.hpp"

#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume_node
{
namespace
{

// visualization_msgs/msg/Marker.msg action + type constants -- not worth a
// dependency on the generated enum names for five values used once each
// (same convention as hd_map.cpp/dynamic_objects.cpp).
constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionModify = 1;
constexpr int32_t kActionDelete = 2;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kMarkerTypeLineStrip = 4;

bool HasNan(const overlume::Vec3& p)
{
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

// points[] on a LINE_STRIP are RELATIVE to marker.pose -- identical to
// hd_map.cpp's own MarkerPoseIsIdentity/MarkerPoseHasNan (each adapter file
// keeps its own copy rather than a shared header).
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

double Dist(const overlume::Vec3& a, const overlume::Vec3& b)
{
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Collapses a producer-repeated vertex (either an accidental consecutive
// duplicate, or the whole ring's start point repeated as its end -- for a
// closed ring that repeat is always adjacent in points[], so the same
// pass catches both) -- NOT a geometric simplification tolerance, just
// "the same point twice in a row is one point".
constexpr double kDedupEpsM = 1e-6;

}  // namespace

// Role -> severity table (sourced from config/rviz/urban_config.rviz:175-223). An
// unknown role is a profile-validator bug (profile.cpp's RoleSets already
// rejects any other role for adapter: collision) -- assert that by throwing,
// never default to info.
uint8_t severity_for_role(const std::string& role)
{
    if (role == "collision") return 2;                              // critical
    if (role == "predicted" || role == "merged_object") return 1;    // warning
    if (role == "sweep" || role == "merged_ego") return 0;           // info (ghost alpha)
    throw std::invalid_argument("CollisionAdapter: unknown role '" + role +
                                "' for adapter: collision");
}

CollisionAdapter::CollisionAdapter(const ProfileRow& row,
                                   const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf), severity_(severity_for_role(row.role))
{
}

void CollisionAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec)
{
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    // ONE lookup for the whole message; same convention as
    // hd_map.cpp/dynamic_objects.cpp.
    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform))
    {
        ++stats_.dropped_no_tf;
        return;  // whole message dropped; previously-stored polygons stay
    }

    for (const auto& m : msg.markers)
    {
        if (m.action == kActionDeleteAll)
        {
            storage_.clear();
            continue;
        }
        if (m.action == kActionDelete)
        {
            storage_.erase(Key{m.ns, m.id});
            continue;
        }
        if (m.action != kActionAdd && m.action != kActionModify) continue;

        if (m.type != kMarkerTypeLineStrip || m.points.size() < 3)
        {
            ++stats_.dropped_malformed;
            continue;
        }

        // effective_point = frame_transform * (marker_pose * point) --
        // identical composition order to hd_map.cpp.
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
            // Zero/degenerate quaternion -> identity, matching rviz.
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
            marker_tf = tf2::Transform(
                q, tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
        }

        std::vector<overlume::Vec3> pts;
        pts.reserve(m.points.size());
        bool ok = true;
        for (const auto& p : m.points)
        {
            const tf2::Vector3 local(p.x, p.y, p.z);
            const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
            const tf2::Vector3 tp = xform * posed;
            const overlume::Vec3 v{tp.x(), tp.y(), tf_.flatten_z() ? 0.0 : tp.z()};
            if (HasNan(v))
            {
                ok = false;
                break;
            }
            // Consecutive-duplicate dedup -- collapses BOTH an accidental
            // repeated vertex and a producer-closed ring's repeated first
            // point (always adjacent in points[] for a closed ring).
            if (!pts.empty() && Dist(pts.back(), v) < kDedupEpsM) continue;
            pts.push_back(v);
        }
        if (!ok)
        {
            // A NaN anywhere drops the WHOLE primitive (spec §9), same as
            // every other adapter.
            ++stats_.dropped_malformed;
            continue;
        }

        // A producer that closes the ring by repeating its first point as
        // its LAST point (not adjacent to any other duplicate) needs one
        // more explicit strip -- the loop above only catches ADJACENT
        // repeats, and a ring's start/end repeat is exactly one such pair
        // sitting at the two ends of the array, not consecutive within it.
        if (pts.size() >= 2 && Dist(pts.front(), pts.back()) < kDedupEpsM) pts.pop_back();

        if (pts.size() < 3)
        {
            // Fewer than 3 DISTINCT points survive -- no polygon (spec §9:
            // dropped and counted, never rendered).
            ++stats_.dropped_malformed;
            continue;
        }

        // CLOSED, always, regardless of which convention the producer used.
        // `pts` is now the OPEN distinct-vertex ring (duplicates stripped
        // above); re-close it here so the library's fan triangulation always
        // sees a genuinely closed loop.
        pts.push_back(pts.front());

        StoredPolygon poly;
        poly.points = std::move(pts);
        poly.last_update_sec = sim_time_sec;
        storage_[Key{m.ns, m.id}] = std::move(poly);
    }

    stats_.last_msg_sec = sim_time_sec;
}

void CollisionAdapter::fill(overlume::ros::SceneAssembly& out) const
{
    for (const auto& [key, poly] : storage_)
    {
        (void)key;
        overlume::AlertPolygon a{};
        a.points = poly.points.data();
        a.point_count = static_cast<uint32_t>(poly.points.size());
        a.severity = severity_;
        a.last_update_sec = poly.last_update_sec;
        out.alerts.push_back(a);
    }
}

}  // namespace overlume_node
