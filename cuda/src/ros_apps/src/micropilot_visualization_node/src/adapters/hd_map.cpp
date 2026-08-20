#include "micropilot_visualization_node/adapters/hd_map.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace mpviz_node
{
namespace
{

// visualization_msgs/msg/Marker.msg action + type constants -- not worth a
// dependency on the generated enum names for six values used once each.
constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionModify = 1;
constexpr int32_t kActionDelete = 2;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kMarkerTypeLineStrip = 4;

bool HasNan(const mpviz::Vec3& p)
{
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

// Marker.msg semantics (rviz-parity gap, user report 2026-08-20): points[]
// on a LINE_STRIP are RELATIVE to marker.pose -- rviz always composes
// pose * point before anything else touches the geometry, including dash
// chopping below. Identity pose (every recorded bag marker's pose today)
// skips the multiply entirely, mirroring FrameTransformer's own
// identity-frame shortcut (frame_transform.cpp) so the common case pays
// nothing.
bool MarkerPoseIsIdentity(const geometry_msgs::msg::Pose& p)
{
    constexpr double kEps = 1e-12;
    return std::abs(p.position.x) < kEps && std::abs(p.position.y) < kEps &&
           std::abs(p.position.z) < kEps && std::abs(p.orientation.x) < kEps &&
           std::abs(p.orientation.y) < kEps && std::abs(p.orientation.z) < kEps &&
           std::abs(p.orientation.w - 1.0) < kEps;
}

// A NaN marker.pose is malformed Marker data (existing dropped_malformed
// path) -- a NON-identity pose is normal Marker semantics, not malformed.
bool MarkerPoseHasNan(const geometry_msgs::msg::Pose& p)
{
    return std::isnan(p.position.x) || std::isnan(p.position.y) || std::isnan(p.position.z) ||
           std::isnan(p.orientation.x) || std::isnan(p.orientation.y) ||
           std::isnan(p.orientation.z) || std::isnan(p.orientation.w);
}

// Dashed centerlines (user directive 2026-08-20: undifferentiated lane
// paint reads as "a repeated mess"). Node-side only -- MapElement is
// frozen at {points, point_count, is_polygon}, so dashing has to happen
// as pure geometry before a marker's points ever become a MapElement.
// Fixed pattern; promote to YAML knobs the day someone actually asks for
// a different rhythm.
constexpr double kDashLenM = 1.5;
constexpr double kGapLenM = 1.5;
constexpr double kMinDashLenM = 0.25;  // shorter trailing dash -> dropped

double Dist(const mpviz::Vec3& a, const mpviz::Vec3& b)
{
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Splits one polyline into its kept dash runs by arc length, alternating
// kDashLenM-long "keep" windows with kGapLenM-long gaps starting at s=0.
// Cut points are interpolated exactly (a dash boundary rarely lands on a
// vertex); any ORIGINAL vertex strictly inside a kept window is preserved
// too, so a dash on a curved centerline still follows the curve instead of
// chording straight across it. Worked example (10 m polyline, 1.5/1.5):
// dashes at [0,1.5],[3,4.5],[6,7.5],[9,10] -- 4 runs, the last only 1.0 m
// (kept: 1.0 >= kMinDashLenM). Empty input, a zero-length polyline, or a
// trailing run shorter than kMinDashLenM yields fewer runs (never a
// zero-length MapElement).
std::vector<std::vector<mpviz::Vec3>> ChopIntoDashes(const std::vector<mpviz::Vec3>& pts)
{
    std::vector<std::vector<mpviz::Vec3>> out;
    if (pts.size() < 2) return out;

    std::vector<double> cum(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) cum[i] = cum[i - 1] + Dist(pts[i - 1], pts[i]);
    const double total_len = cum.back();
    if (total_len <= 0.0) return out;

    const auto point_at = [&](double s) -> mpviz::Vec3 {
        s = std::clamp(s, 0.0, total_len);
        // First cumulative-length entry >= s -- cum is sorted ascending by
        // construction, so std::lower_bound is exact, not a heuristic.
        size_t i = static_cast<size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
        if (i == 0) i = 1;
        if (i >= pts.size()) i = pts.size() - 1;
        const double seg_len = cum[i] - cum[i - 1];
        const double t = seg_len > 0.0 ? (s - cum[i - 1]) / seg_len : 0.0;
        const auto& a = pts[i - 1];
        const auto& b = pts[i];
        return mpviz::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    };

    constexpr double kPeriod = kDashLenM + kGapLenM;
    for (double s0 = 0.0; s0 < total_len; s0 += kPeriod)
    {
        const double s1 = std::min(s0 + kDashLenM, total_len);
        if (s1 - s0 < kMinDashLenM) continue;  // trailing partial dash, too short to keep

        std::vector<mpviz::Vec3> dash;
        dash.push_back(point_at(s0));
        for (size_t i = 0; i < pts.size(); ++i)
        {
            if (cum[i] > s0 && cum[i] < s1) dash.push_back(pts[i]);
        }
        dash.push_back(point_at(s1));
        out.push_back(std::move(dash));
    }
    return out;
}

}  // namespace

HdMapAdapter::HdMapAdapter(const ProfileRow& row,
                            const micropilot::visualization_app::FrameTransformer& tf)
    : row_(row), tf_(tf)
{
}

void HdMapAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec)
{
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    // ONE lookup for the whole message (epic2 plan, "Frames") -- every
    // marker in a MarkerArray from one publisher shares one frame_id in
    // practice; looking it up per-marker would be hundreds of redundant
    // buffer walks for zero benefit.
    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform))
    {
        ++stats_.dropped_no_tf;
        return;  // whole message dropped; previously-stored elements stay
    }

    // Rate limit: gates REBUILDS, not receipt. First-ever call always
    // rebuilds (last_rebuild_sec_ starts at -1.0, "no prior rebuild").
    if (row_.max_rate_hz > 0.0 && last_rebuild_sec_ >= 0.0)
    {
        const double min_gap_sec = 1.0 / row_.max_rate_hz;
        if (sim_time_sec - last_rebuild_sec_ < min_gap_sec) return;
    }

    for (const auto& m : msg.markers)
    {
        if (m.action == kActionDeleteAll)
        {
            // ROS Marker semantics: DELETEALL clears every marker this
            // adapter is tracking, regardless of ITS OWN ns/id fields.
            storage_.clear();
            continue;
        }
        if (m.action == kActionDelete)
        {
            storage_.erase(Key{m.ns, m.id});
            continue;
        }
        if (m.action != kActionAdd && m.action != kActionModify) continue;

        const NsRule* rule = match_rule(row_, m.ns);
        const NsRender verdict = rule != nullptr ? rule->render : row_.ns_default;
        if (verdict == NsRender::kDrop)
        {
            // Intentional, not malformed -- see epic2 plan, "Diagnostics
            // counters": centerline_arrows_ alone is ~93% of map volume,
            // and folding this into dropped_malformed would make a
            // healthy system look broken.
            ++stats_.dropped_by_rule;
            continue;
        }

        if (m.type != kMarkerTypeLineStrip || m.points.size() < 2)
        {
            ++stats_.dropped_malformed;
            continue;
        }

        // effective_point = frame_transform * (marker_pose * point) -- the
        // marker pose lives IN the header frame, so it composes INSIDE the
        // frame transform, not outside (rviz-parity gap fix). Built once
        // per marker, before dash chopping or anything else touches points.
        const bool identity_pose = MarkerPoseIsIdentity(m.pose);
        tf2::Transform marker_tf;
        if (!identity_pose)
        {
            if (MarkerPoseHasNan(m.pose))
            {
                ++stats_.dropped_malformed;  // NaN marker pose, not just a NaN point
                continue;
            }
            tf2::Quaternion q(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                              m.pose.orientation.w);
            // Zero/degenerate quaternion -> identity, matching rviz; tf2
            // would NaN every point and silently void the whole marker
            // (review finding 2026-08-20).
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
            marker_tf = tf2::Transform(
                q, tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
        }

        std::vector<mpviz::Vec3> pts;
        pts.reserve(m.points.size());
        bool ok = true;
        for (const auto& p : m.points)
        {
            const tf2::Vector3 local(p.x, p.y, p.z);
            const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
            const tf2::Vector3 tp = xform * posed;
            // flatten_z: 2D HD-map plane -- see frame_transform.hpp.
            const mpviz::Vec3 v{tp.x(), tp.y(), tf_.flatten_z() ? 0.0 : tp.z()};
            if (HasNan(v))
            {
                ok = false;
                break;
            }
            pts.push_back(v);
        }
        if (!ok)
        {
            // A NaN anywhere in the polyline drops the WHOLE primitive
            // (spec §9) -- never a partially-built vertex buffer.
            ++stats_.dropped_malformed;
            continue;
        }

        const uint8_t is_polygon = (verdict == NsRender::kPolygon) ? 1 : 0;
        std::vector<StoredElement> pieces;
        if (rule != nullptr && rule->dashed && verdict == NsRender::kPolyline)
        {
            // Dashing is a marker-ingest-time geometry op, not a rendering
            // concept -- one marker becomes N kept dash runs, still ONE
            // entry in storage_ (still ONE ingested marker for stats: see
            // this function's ++stats_.msgs at the top, incremented once
            // per ingest() call, never per marker/dash).
            for (auto& dash_pts : ChopIntoDashes(pts))
            {
                StoredElement piece;
                piece.points = std::move(dash_pts);
                piece.is_polygon = is_polygon;
                pieces.push_back(std::move(piece));
            }
        }
        else
        {
            StoredElement elem;
            elem.points = std::move(pts);
            elem.is_polygon = is_polygon;
            pieces.push_back(std::move(elem));
        }
        storage_[Key{m.ns, m.id}] = std::move(pieces);
    }

    last_rebuild_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void HdMapAdapter::fill(micropilot::visualization_app::SceneAssembly& out) const
{
    for (const auto& [key, pieces] : storage_)
    {
        (void)key;
        for (const auto& elem : pieces)
        {
            mpviz::MapElement e{};
            e.points = elem.points.data();
            e.point_count = static_cast<uint32_t>(elem.points.size());
            e.is_polygon = elem.is_polygon;
            out.map_elements.push_back(e);
        }
    }
}

}  // namespace mpviz_node
