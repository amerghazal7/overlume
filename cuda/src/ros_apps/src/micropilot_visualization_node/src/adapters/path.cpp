#include "micropilot_visualization_node/adapters/path.hpp"

#include <cmath>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace mpviz_node
{
namespace
{

// Hand-kept mirror of the library's kStaleFadeTimeoutSec (renderer-internal,
// unreachable from node code) -- if that constant moves, this moves with it.
// Same contract as hd_map.cpp's kMapFadeWindowSec.
constexpr double kPathFadeWindowSec = 1.0;

mpviz::PathRole RoleFromString(const std::string& s)
{
    if (s == "behavior") return mpviz::PathRole::BEHAVIOR;
    if (s == "global") return mpviz::PathRole::GLOBAL;
    return mpviz::PathRole::LOCAL;  // profile.cpp's ValidateRow already rejects anything else
}

bool HasNan(double v) { return std::isnan(v); }

}  // namespace

PathAdapter::PathAdapter(const ProfileRow& row,
                         const micropilot::visualization_app::FrameTransformer& tf)
    : row_(row), tf_(tf), role_(RoleFromString(row.role))
{
}

void PathAdapter::ingest(const nav_msgs::msg::Path& msg, double sim_time_sec)
{
    ++stats_.msgs;

    // Empty/single-pose Path has no polyline to draw -- dropped whole,
    // stored ribbon (if any) keeps rendering (spec §9: drop the
    // primitive, never propagate, never erase what still renders).
    if (msg.poses.size() < 2)
    {
        ++stats_.dropped_malformed;
        return;
    }

    // ONE lookup for the whole message; same reasoning as every other adapter
    // -- every pose in the recorded bag shares the Path's header frame/stamp.
    tf2::Transform xform;
    if (!tf_.lookup(msg.header, xform))
    {
        ++stats_.dropped_no_tf;
        return;  // whole message dropped; previously-stored ribbon stays
    }

    // Positions ONLY (pose.orientation is identity on the wire for this
    // topic and PathRibbon is positions-only regardless -- see this file's
    // header comment: don't be clever with orientation).
    std::vector<mpviz::Vec3> next;
    next.reserve(msg.poses.size());
    for (const auto& ps : msg.poses)
    {
        const auto& p = ps.pose.position;
        const tf2::Vector3 v = xform * tf2::Vector3(p.x, p.y, p.z);
        // NaN checked on the TRANSFORMED point (hd_map.cpp's convention): a
        // NaN-bearing /tf entry must also drop the path here and count it,
        // not leak NaN into the library for extrude_polyline to silently truncate.
        if (HasNan(v.x()) || HasNan(v.y()) || HasNan(v.z()))
        {
            ++stats_.dropped_malformed;  // NaN pose/TF -- drop the whole path
            return;  // previously-stored ribbon stays; no partial replace
        }
        // flatten_z: 2D HD-map plane -- see frame_transform.hpp.
        next.push_back({v.x(), v.y(), tf_.flatten_z() ? 0.0 : v.z()});
    }

    // REPLACES the stored path wholesale, never appends/merges
    // (PathChangeReplacesRatherThanAppends).
    points_ = std::move(next);
    // Anchored stamp (flicker root-cause hardening 2026-09-10): the fade
    // window sits in the LAST kPathFadeWindowSec before this row's own
    // timeout cutoff, not kStaleFadeTimeoutSec after raw receipt — the same
    // hd_map.cpp kMapFadeWindowSec pattern (VM-034), which never covered
    // path rows. Raw stamping had ZERO margin here: the recorded
    // /local_vel_path max inter-message gap is 0.461 s vs the library's
    // 0.5 s fade start. profile.cpp validates timeout_sec >= 1.0, so the
    // offset is never negative.
    last_update_sec_ = sim_time_sec + (row_.timeout_sec - kPathFadeWindowSec);
    stats_.last_msg_sec = sim_time_sec;
}

void PathAdapter::fill(micropilot::visualization_app::SceneAssembly& out) const
{
    if (points_.size() < 2) return;  // never received valid data yet

    mpviz::PathRibbon r{};
    r.role = role_;
    r.points = points_.data();
    r.point_count = static_cast<uint32_t>(points_.size());
    r.last_update_sec = last_update_sec_;
    out.paths.push_back(r);
}

}  // namespace mpviz_node
