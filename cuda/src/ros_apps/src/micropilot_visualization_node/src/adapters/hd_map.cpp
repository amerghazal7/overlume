#include "micropilot_visualization_node/adapters/hd_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

// Road-surface fill (Epic 3 Task 1 / VM-036, decision #5): the number of
// stations both boundary rails are resampled to, by normalized arc length,
// before being zipped into a triangle strip library-side. Fixed; ROAD_
// SURFACE's point_count is always 2*kRoadFillSamples.
constexpr uint32_t kRoadFillSamples = 16;

// VM-034 review fix, round 2 (fade-pulse policy, blocking): fill() used to
// stamp e.last_update_sec = last_recv_sec_ straight -- the library's fade
// (SceneBuffer::staleness_alpha) then starts at 0.5s of silence and
// completes at 1.0s (kStaleFadeStartSec/kStaleFadeTimeoutSec,
// visual_renderer/src/renderer_internal.hpp:123-124; NOT "only 1.0s of
// silence fades it," a claim this file and hd_map.hpp used to make and both
// got wrong). Two real rows broke against that 0.5-1.0s window: a row
// received at 1-2 Hz sawtoothed alpha 1.0->0.0 every receipt gap (last_recv_
// sec_ frozen between receipts while sim_time keeps climbing), and a
// publish-once/transient_local row (sim_profile.yaml's /sim/hd_map/markers,
// one latched message, 3725 markers, the sim profile's ONLY full-extent map
// source) faded to alpha 0 by +1.0s even though visualization_node.cpp
// keeps calling fill() for it until its own timeout_sec cutoff (5.0s) --
// "map never appears" for 4 of that row's 5 visible seconds.
//
// Fix: stamp the fade window into the LAST kMapFadeWindowSec before the
// row's OWN timeout_sec cutoff, not kStaleFadeTimeoutSec after the last
// receipt -- e.last_update_sec = last_recv_sec_ + (row_.timeout_sec -
// kMapFadeWindowSec). A row received faster than roughly (timeout_sec -
// kMapFadeWindowSec - kStaleFadeStartSec) apart stamps a last_update_sec
// that stays in the future of "now" every tick, so staleness_alpha's age
// stays negative and the row is continuously opaque (no sawtooth). A row
// that stops receiving (or never receives again, the transient_local case)
// freezes that same stamp, so the fade now plays out in the
// kMapFadeWindowSec right before the node stops calling fill() -- an
// opacity ramp into the cutoff (spec §5), not a pop, and not an early fade
// while the node is still choosing to show the row.
//
// kMapFadeWindowSec mirrors the library's kStaleFadeTimeoutSec (renderer_
// internal.hpp:124) on purpose -- that header is library-internal, not
// reachable from node-side code, so this is a hand-kept copy: if that
// constant ever moves, this one must move with it. profile.cpp's own
// validation (row.timeout_sec >= 1.0) guarantees row_.timeout_sec -
// kMapFadeWindowSec is never negative.
constexpr double kMapFadeWindowSec = 1.0;

// Resamples `pts` to exactly `n_stations` points, evenly spaced by
// NORMALIZED arc length (station k is at s = k/(n_stations-1) * total arc
// length) -- extracted from the pre-Epic3 dash-chopper's own point_at()
// lambda (the arc-length-walk technique is what's reused here, not the
// dash-specific caller, which is gone -- dashing moved renderer-side, see
// map_elements.cpp). Two rails of a lane are NOT guaranteed to carry the
// same point count on real data (verified: lane 955 is 8/9, lane 813 is
// 10/11 in the committed fixture) -- resampling both to the same fixed
// station count is what lets the library zip them into a strip without
// ever indexing past either rail's own point array. Returns empty for
// fewer than 2 input points or n_stations == 0 (malformed guard, mirrors
// every other "not enough data" path in this file).
std::vector<mpviz::Vec3> ResampleByArcLength(const std::vector<mpviz::Vec3>& pts,
                                              uint32_t n_stations)
{
    std::vector<mpviz::Vec3> out;
    if (pts.size() < 2 || n_stations == 0) return out;

    std::vector<double> cum(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i)
    {
        const double dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y,
                     dz = pts[i].z - pts[i - 1].z;
        cum[i] = cum[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    const double total_len = cum.back();

    out.reserve(n_stations);
    for (uint32_t k = 0; k < n_stations; ++k)
    {
        const double s = (n_stations == 1)
                             ? 0.0
                             : total_len * static_cast<double>(k) / static_cast<double>(n_stations - 1);
        size_t i = static_cast<size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
        if (i == 0) i = 1;
        if (i >= pts.size()) i = pts.size() - 1;
        const double seg_len = cum[i] - cum[i - 1];
        const double t = seg_len > 0.0 ? (s - cum[i - 1]) / seg_len : 0.0;
        const auto& a = pts[i - 1];
        const auto& b = pts[i];
        out.push_back(mpviz::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                                   a.z + (b.z - a.z) * t});
    }
    return out;
}

// Kinds that carry a lane_id (the marker's own `id`, per decision #4) --
// crosswalk/stopline/junction/other are not lane-paired. ROAD_EDGE carries
// one because fill()'s promotion pass (below) relabels a LEFT_/RIGHT_
// BOUNDARY in place, keeping its lane_id -- the kind IS emitted today; what
// never produces it is an ingest-time namespace rule.
bool KindCarriesLaneId(mpviz::MapKind kind)
{
    return kind == mpviz::MapKind::CENTERLINE || kind == mpviz::MapKind::LEFT_BOUNDARY ||
           kind == mpviz::MapKind::RIGHT_BOUNDARY || kind == mpviz::MapKind::ROAD_EDGE;
}

// Road-edge detection (Epic 3 Task 1 / VM-036, user directive 2026-09-08):
// "the boundary of the road (most left and most right lines) should not be
// dashed and should be colored differently, usually yellow." ROAD_EDGE was
// reserved (Task 1) but had no producer until now. Detection: a LEFT_
// BOUNDARY/RIGHT_BOUNDARY element promotes to ROAD_EDGE when NO OTHER
// lane's OPPOSITE-side boundary sits within kRoadEdgeCoincidenceThresholdM
// of it -- an interior divider between two adjacent lanes has its own
// near-duplicate polyline recorded on the neighbour's opposite side (the
// SAME painted line, surveyed twice); a lane on the road's outer edge does
// not.
//
// Threshold, MEASURED against the committed hd_map_local_elements_0.yaml
// fixture (not guessed): sampling each boundary's own two endpoints + its
// midpoint, and taking the minimum point-to-polyline distance to every
// OTHER lane's opposite-side boundary, the fixture's 16 real adjacent-lane
// pairs land at 0.00-0.20 m (Epic 2's own prior measurement called this
// "centimeters," and this fixture confirms it -- several pairs are exact
// floating-point duplicates, the survey recorded the same line twice). The
// NEXT-nearest case that is NOT a shared line sits at 1.7955 m (left_
// boundary_1453 vs. its closest non-twin, right_boundary_838) with the bulk
// of true non-adjacent lanes at 3.1-3.7 m (roughly one full lane width
// away). kRoadEdgeCoincidenceThresholdM = 1.0 m sits in the gap between
// 0.1983 m (the largest genuine twin separation) and 1.7955 m (the nearest
// genuine non-twin) -- real headroom on both sides, not a round-number
// guess. On this fixture the result is 15 of the 32 boundary elements
// promoted to ROAD_EDGE (one lane, 934, is fully interior -- paired on both
// sides -- and contributes none); see this epic's plan doc addendum for the
// full worked measurement.
constexpr double kRoadEdgeCoincidenceThresholdM = 1.0;

double PointToPolylineDist2D(const mpviz::Vec3& p, const std::vector<mpviz::Vec3>& poly)
{
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < poly.size(); ++i)
    {
        const auto& a = poly[i];
        const auto& b = poly[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 1e-12)
        {
            t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double cx = a.x + t * dx, cy = a.y + t * dy;
        const double d = std::hypot(p.x - cx, p.y - cy);
        if (d < best) best = d;
    }
    return best;
}

// True when `pts` (this lane's own boundary, `lane_id`) has no coincident
// twin in `opposite_by_lane` (every OTHER lane's opposite-side boundary) --
// i.e. it is the road's outer edge. Sampled at `pts`' own first/mid/last
// point (the plan directive's own sampling choice: "the boundary's midpoint
// and endpoints for robustness"), each checked against a candidate's FULL
// polyline extent (not just ITS endpoints), so a candidate that only grazes
// one of this boundary's three samples still counts as coincident.
bool IsRoadEdge(uint32_t lane_id, const std::vector<mpviz::Vec3>& pts,
                const std::unordered_map<uint32_t, const std::vector<mpviz::Vec3>*>& opposite_by_lane)
{
    if (pts.size() < 2) return false;
    const mpviz::Vec3 samples[3] = {pts.front(), pts[pts.size() / 2], pts.back()};
    for (const auto& [other_lane, other_pts] : opposite_by_lane)
    {
        if (other_lane == lane_id || other_pts == nullptr) continue;
        for (const auto& s : samples)
        {
            if (PointToPolylineDist2D(s, *other_pts) < kRoadEdgeCoincidenceThresholdM) return false;
        }
    }
    return true;
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

    // VM-034 review fix: stamp topic-liveness BEFORE the rate gate below,
    // not after it. fill() stamps every MapElement::last_update_sec from
    // this, not from last_rebuild_sec_/stats_.last_msg_sec (both set only
    // past the gate) -- see hd_map.hpp's Staleness comment for why: a
    // throttled row must stay opaque between accepted rebuilds as long as
    // it keeps receiving, not blink dark on the rebuild cadence.
    last_recv_sec_ = sim_time_sec;

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

        // Crosswalk-hatch fix (Epic 3 Task 1 / VM-036, decision #2): a
        // closed polygon marker arrives with a duplicate closing vertex
        // (point[0] == point[n-1], verified against the real recorded
        // fixture's crosswalk_8043) -- mirrors collision.cpp's own
        // trailing-duplicate dedupe verbatim (kDedupEpsM = 1e-6). Without
        // this, build_crosswalk_hatch()'s n==4 guard never fires on real
        // data (every recorded crosswalk arrives with 5 points).
        if (is_polygon && pts.size() >= 2)
        {
            constexpr double kDedupEpsM = 1e-6;
            const auto& front = pts.front();
            const auto& back = pts.back();
            const double dx = back.x - front.x, dy = back.y - front.y, dz = back.z - front.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < kDedupEpsM) pts.pop_back();
        }

        // kind/lane_id extraction (Epic 3 Task 1 / VM-036, decision #4):
        // `kind` is the matched NsRule's own field (no match -> OTHER,
        // same "no rule -> default" shape ns_default already has for
        // render verdicts); `lane_id` is the marker's own `id` for the
        // lane-paired kinds, 0 otherwise.
        const mpviz::MapKind kind = rule != nullptr ? rule->kind : mpviz::MapKind::OTHER;
        const uint32_t lane_id = KindCarriesLaneId(kind) ? static_cast<uint32_t>(m.id) : 0;

        StoredElement elem;
        elem.points = std::move(pts);
        elem.is_polygon = is_polygon;
        elem.kind = kind;
        elem.lane_id = lane_id;
        storage_[Key{m.ns, m.id}] = std::vector<StoredElement>{std::move(elem)};
    }

    last_rebuild_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void HdMapAdapter::fill(micropilot::visualization_app::SceneAssembly& out) const
{
    road_surface_points_.clear();
    std::unordered_map<uint32_t, const std::vector<mpviz::Vec3>*> left_by_lane, right_by_lane;

    // First pass: index every boundary rail by lane_id. Needed BEFORE any
    // element is emitted -- road-edge detection (below) must see every
    // OTHER lane's opposite-side boundary, and road-surface pairing
    // (further below) needs the same index. One pass over storage_ builds
    // both; a second walk (below) does the actual emitting.
    for (const auto& [key, pieces] : storage_)
    {
        (void)key;
        for (const auto& elem : pieces)
        {
            if (elem.lane_id == 0) continue;
            if (elem.kind == mpviz::MapKind::LEFT_BOUNDARY)
            {
                left_by_lane[elem.lane_id] = &elem.points;
            }
            else if (elem.kind == mpviz::MapKind::RIGHT_BOUNDARY)
            {
                right_by_lane[elem.lane_id] = &elem.points;
            }
        }
    }

    // Second pass: emit one MapElement per stored element, promoting a
    // LEFT_BOUNDARY/RIGHT_BOUNDARY to ROAD_EDGE when IsRoadEdge() finds no
    // coincident twin on the opposite side (see that function's own
    // comment for the measured threshold). The left_by_lane/right_by_lane
    // maps built above are unaffected by this promotion -- road-surface
    // fill still pairs by the ORIGINAL LEFT_BOUNDARY/RIGHT_BOUNDARY kind,
    // regardless of what kind the emitted element ends up carrying.
    for (const auto& [key, pieces] : storage_)
    {
        (void)key;
        for (const auto& elem : pieces)
        {
            mpviz::MapElement e{};
            e.points = elem.points.data();
            e.point_count = static_cast<uint32_t>(elem.points.size());
            e.is_polygon = elem.is_polygon;
            e.kind = elem.kind;
            e.lane_id = elem.lane_id;
            // VM-034 review fix: stamped from last_recv_sec_ (topic
            // liveness), not stats_.last_msg_sec, offset into the last
            // kMapFadeWindowSec of this row's own timeout_sec (see
            // kMapFadeWindowSec's own comment above and hd_map.hpp).
            e.last_update_sec = last_recv_sec_ + (row_.timeout_sec - kMapFadeWindowSec);
            if (e.kind == mpviz::MapKind::LEFT_BOUNDARY &&
                IsRoadEdge(elem.lane_id, elem.points, right_by_lane))
            {
                e.kind = mpviz::MapKind::ROAD_EDGE;
            }
            else if (e.kind == mpviz::MapKind::RIGHT_BOUNDARY &&
                     IsRoadEdge(elem.lane_id, elem.points, left_by_lane))
            {
                e.kind = mpviz::MapKind::ROAD_EDGE;
            }
            out.map_elements.push_back(e);
        }
    }

    // Road-surface fill (decision #5): pair every lane_id present on BOTH
    // rails; a lane_id on only one rail (0 of 16 in the committed fixture,
    // but not provably impossible on other bags) emits nothing for it --
    // silently dropped, not malformed (spec §9's "missing data renders
    // nothing"). road_surface_points_ holds the resampled buffers these
    // synthesized elements point into; cleared and rebuilt at the top of
    // every fill() call, so it stays alive exactly as long as this fill()
    // call's own out.map_elements does.
    for (const auto& [lane_id, left_pts] : left_by_lane)
    {
        const auto it = right_by_lane.find(lane_id);
        if (it == right_by_lane.end()) continue;

        const std::vector<mpviz::Vec3> left_r = ResampleByArcLength(*left_pts, kRoadFillSamples);
        const std::vector<mpviz::Vec3> right_r = ResampleByArcLength(*it->second, kRoadFillSamples);
        if (left_r.empty() || right_r.empty()) continue;  // malformed rail, skip silently

        road_surface_points_.emplace_back();
        std::vector<mpviz::Vec3>& combined = road_surface_points_.back();
        combined.reserve(2 * kRoadFillSamples);
        combined.insert(combined.end(), left_r.begin(), left_r.end());
        combined.insert(combined.end(), right_r.begin(), right_r.end());

        mpviz::MapElement e{};
        e.points = combined.data();
        e.point_count = static_cast<uint32_t>(combined.size());
        e.is_polygon = 0;
        e.kind = mpviz::MapKind::ROAD_SURFACE;
        e.lane_id = lane_id;
        // VM-034 review fix: same offset stamp as every other emitted
        // element above -- the synthesized ROAD_SURFACE element must fade
        // (and ramp-out-before-cutoff) too.
        e.last_update_sec = last_recv_sec_ + (row_.timeout_sec - kMapFadeWindowSec);
        out.map_elements.push_back(e);
    }
}

}  // namespace mpviz_node
