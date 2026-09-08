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
// cum[0] == 0, cum[i] == arc length from pts[0] to pts[i], cum.back() ==
// total polyline length. Shared by ResampleByArcLength (road-surface fill)
// and the junction-cleanup clip/cut machinery below (user directive
// 2026-09-08) -- both need to turn "a point somewhere along this polyline"
// into/from a normalized arc-length station.
std::vector<double> CumulativeArcLength(const std::vector<mpviz::Vec3>& pts)
{
    std::vector<double> cum(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i)
    {
        const double dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y,
                     dz = pts[i].z - pts[i - 1].z;
        cum[i] = cum[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return cum;
}

// Interpolated point at arc length `s` (clamped to [0, cum.back()]) along
// `pts`, using its own precomputed `cum` (CumulativeArcLength(pts)).
mpviz::Vec3 PointAtArcLength(const std::vector<mpviz::Vec3>& pts, const std::vector<double>& cum,
                              double s)
{
    const double total_len = cum.back();
    s = std::clamp(s, 0.0, total_len);
    size_t i = static_cast<size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
    if (i == 0) i = 1;
    if (i >= pts.size()) i = pts.size() - 1;
    const double seg_len = cum[i] - cum[i - 1];
    const double t = seg_len > 0.0 ? (s - cum[i - 1]) / seg_len : 0.0;
    const auto& a = pts[i - 1];
    const auto& b = pts[i];
    return mpviz::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

std::vector<mpviz::Vec3> ResampleByArcLength(const std::vector<mpviz::Vec3>& pts,
                                              uint32_t n_stations)
{
    std::vector<mpviz::Vec3> out;
    if (pts.size() < 2 || n_stations == 0) return out;

    const std::vector<double> cum = CumulativeArcLength(pts);
    const double total_len = cum.back();

    out.reserve(n_stations);
    for (uint32_t k = 0; k < n_stations; ++k)
    {
        const double s = (n_stations == 1)
                             ? 0.0
                             : total_len * static_cast<double>(k) / static_cast<double>(n_stations - 1);
        out.push_back(PointAtArcLength(pts, cum, s));
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

// ---- Junction cleanup (user directive 2026-09-08 + same-day refinement) --
// "the middle area [of a junction]... it would be much nicer if we cut
// [ROAD_EDGE lines] off in these areas and continue along the road after
// the junction" / refinement: "The junction interior is only allowed to
// have the lane[]-separating dashed lines, or make it configurable... but
// enable them by default." Two independent cut mechanisms, both run from
// fill() below:
//   1. JUNCTION-POLYGON CLIP (ClipAgainstJunctions): where the message
//      actually carries MapKind::JUNCTION geometry (today: only
//      /sim/hd_map/markers's `junction` namespace rule -- verified,
//      urban_profile.yaml's /hd_map_local_elements and
//      /hd_map_global_elements rows have no `junction` rule at all, so this
//      mechanism is a no-op for them) a ROAD_EDGE polyline is clipped
//      against the union of that message's JUNCTION rings. LEFT_/RIGHT_
//      BOUNDARY gets the identical clip ONLY when the row's
//      `junction_interior_boundaries` is false (default true -- the
//      refinement's own "enable them by default").
//   2. MUTUAL-CROSSING CUT (SegSegIntersect2D + the window machinery):
//      works with NO junction data at all -- any two ROAD_EDGE polylines
//      from DIFFERENT lane_ids that cross in 2D at a real angle get
//      trimmed back kJunctionCutBackoffM from the crossing point, both
//      sides continuing beyond. CORRECTED (code-review finding, re-derived
//      against hd_map_local_elements_0.yaml): "a real road edge never
//      legitimately crosses another, so this only ever fires inside a
//      junction" is NOT what the original measurement showed -- of the 24
//      raw 2D crossings on that fixture, 21 are shared-endpoint abutments
//      between chained lanelet boundaries (outgoing angle 175.2-179.8 deg,
//      i.e. one continuous straight line through the shared node, or
//      0.5-2.8 deg, i.e. two rails leaving the same node codirectionally --
//      duplicate surveys of the same rail); of the remaining 3 candidate
//      "interior" crossings (not an exact endpoint on either side), only 1
//      is a genuine junction crossing by ANGLE -- the other 2 are also
//      near-parallel/near-antiparallel despite landing away from either
//      polyline's own literal endpoint (a chained polyline can carry an
//      extra sample point close to a node). A lanelet-chain node with a
//      plain survey kink of a fraction of a degree is not a junction.
//      SegSegIntersect2D now rejects near-parallel AND near-antiparallel
//      segment pairs (within kMinCrossingSinAngle's ~15 deg of 0 or 180
//      deg), not just exactly-parallel ones -- see that function's own
//      comment; this is what those 2 additional rejections ride on. See
//      test_hd_map_adapter.cpp's LocalElementsFixtureYieldsLanesAndCrosswalks
//      and this epic's plan doc for the full re-derivation.
//      NEVER applied to boundaries: interior separators legitimately cross
//      connector geometry inside a junction (the refinement's own point).
// O(edges^2 * segments^2) over this row's own promoted-ROAD_EDGE count
// (~15 edges x ~20 segments on the committed urban fixture) is fine at
// this scale -- no spatial index attempted.
constexpr double kJunctionCutBackoffM = 2.0;

// Mirrors map_elements.cpp's own IsBoundaryKind() (library-side, not
// reachable from this node-side translation unit) -- same two kinds, same
// meaning: an interior lane separator, never the road's own outer edge.
bool IsBoundaryKind(mpviz::MapKind kind)
{
    return kind == mpviz::MapKind::LEFT_BOUNDARY || kind == mpviz::MapKind::RIGHT_BOUNDARY;
}

// Even-odd point-in-polygon on the 2D (x,y) ring `poly` -- standard ray-cast
// parity of edge crossings to the right of `p`. `poly` need not repeat its
// first point as its last (the recorded JUNCTION markers do; a hand-built
// ring need not) -- the wraparound `j = poly.size() - 1` always closes it.
bool PointInPolygonEvenOdd(const mpviz::Vec3& p, const std::vector<mpviz::Vec3>& poly)
{
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
    {
        const mpviz::Vec3& a = poly[i];
        const mpviz::Vec3& b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x))
        {
            inside = !inside;
        }
    }
    return inside;
}

bool PointInAnyPolygon(const mpviz::Vec3& p,
                        const std::vector<const std::vector<mpviz::Vec3>*>& polys)
{
    for (const auto* poly : polys)
    {
        if (poly != nullptr && poly->size() >= 3 && PointInPolygonEvenOdd(p, *poly)) return true;
    }
    return false;
}

// Splits `pts` into the sub-polylines that lie OUTSIDE the union of
// `polys`: point-in-polygon at every vertex, a 30-halving bisection (far
// finer than survey precision) locating the entry/exit crossing on any
// segment whose two endpoints disagree. Assumes at most one inside<->
// outside transition per INPUT segment -- true of every recorded boundary/
// junction pair (input points a few meters apart, junction boxes tens of
// meters across); a segment that both enters and exits the same polygon
// keeps its middle crossing unseen, a stated limitation, not something
// this epic's real data exercises. Empty `polys` (or `pts.size() < 2`) is a
// no-op: returns `pts` unchanged as the one surviving piece -- this is what
// makes `junction_interior_boundaries: false` correctly inert on a row with
// no JUNCTION geometry at all (ProfileRow's own documented limitation).
std::vector<std::vector<mpviz::Vec3>> ClipAgainstJunctions(
    const std::vector<mpviz::Vec3>& pts, const std::vector<const std::vector<mpviz::Vec3>*>& polys)
{
    std::vector<std::vector<mpviz::Vec3>> out;
    if (polys.empty() || pts.size() < 2)
    {
        out.push_back(pts);
        return out;
    }

    auto lerp = [](const mpviz::Vec3& a, const mpviz::Vec3& b, double t) {
        return mpviz::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    };

    std::vector<mpviz::Vec3> current;
    bool a_in = PointInAnyPolygon(pts.front(), polys);
    if (!a_in) current.push_back(pts.front());
    for (size_t i = 0; i + 1 < pts.size(); ++i)
    {
        const mpviz::Vec3& a = pts[i];
        const mpviz::Vec3& b = pts[i + 1];
        const bool b_in = PointInAnyPolygon(b, polys);
        if (a_in == b_in)
        {
            if (!b_in) current.push_back(b);
        }
        else
        {
            double lo = 0.0, hi = 1.0;  // lo matches a_in's side, hi matches b_in's
            for (int iter = 0; iter < 30; ++iter)
            {
                const double mid = 0.5 * (lo + hi);
                if (PointInAnyPolygon(lerp(a, b, mid), polys) == a_in)
                    lo = mid;
                else
                    hi = mid;
            }
            const mpviz::Vec3 cross = lerp(a, b, 0.5 * (lo + hi));
            if (a_in && !b_in)
            {
                // Exiting the polygon: start a fresh kept (outside) chain.
                current.clear();
                current.push_back(cross);
                current.push_back(b);
            }
            else
            {
                // Entering the polygon: close the kept chain here.
                current.push_back(cross);
                if (current.size() >= 2) out.push_back(current);
                current.clear();
            }
        }
        a_in = b_in;
    }
    if (current.size() >= 2) out.push_back(current);
    return out;
}

// 2D (x,y) segment-segment intersection, (p1,p2) x (p3,p4). On a genuine
// crossing, writes the parametric position along EACH segment (0..1) to
// t/u respectively and returns true. Rejects near-parallel AND
// near-antiparallel segment pairs, not just exactly-parallel ones: the
// cross-product denom is proportional to sin(angle between the two
// directions), and sin(180 deg - x) == sin(x), so normalizing it by the
// two segment lengths and gating on kMinCrossingSinAngle catches a
// lanelet-chain node's fraction-of-a-degree kink the same way it catches
// two open-road ROAD_EDGE rails running alongside each other -- see this
// function's own callers' comment for the fixture classification
// (175.2-179.8 deg / 0.5-2.8 deg abutments vs. genuine crossings) that
// motivated widening this past the original exact-parallel-only guard.
constexpr double kMinCrossingSinAngle = 0.25881904510252074;  // sin(15 deg)

bool SegSegIntersect2D(const mpviz::Vec3& p1, const mpviz::Vec3& p2, const mpviz::Vec3& p3,
                       const mpviz::Vec3& p4, double& t, double& u)
{
    const double d1x = p2.x - p1.x, d1y = p2.y - p1.y;
    const double d2x = p4.x - p3.x, d2y = p4.y - p3.y;
    const double denom = d1x * d2y - d1y * d2x;
    const double len1 = std::hypot(d1x, d1y);
    const double len2 = std::hypot(d2x, d2y);
    if (len1 < 1e-9 || len2 < 1e-9) return false;
    if (std::abs(denom) < kMinCrossingSinAngle * len1 * len2) return false;
    const double dx = p3.x - p1.x, dy = p3.y - p1.y;
    t = (dx * d2y - dy * d2x) / denom;
    u = (dx * d1y - dy * d1x) / denom;
    return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
}

// Sorts + merges overlapping/touching [start,end] windows in place -- one
// polyline crossed by several others accumulates one raw window per
// crossing before this ever runs.
void MergeWindows(std::vector<std::pair<double, double>>& windows)
{
    if (windows.empty()) return;
    std::sort(windows.begin(), windows.end());
    std::vector<std::pair<double, double>> merged;
    merged.push_back(windows.front());
    for (size_t i = 1; i < windows.size(); ++i)
    {
        if (windows[i].first <= merged.back().second)
        {
            merged.back().second = std::max(merged.back().second, windows[i].second);
        }
        else
        {
            merged.push_back(windows[i]);
        }
    }
    windows = std::move(merged);
}

// Splits `pts` into the sub-polylines that survive after removing every
// already-merged arc-length `windows` entry -- the mutual-crossing cut's
// own geometry op, sibling to ClipAgainstJunctions above but keyed by arc
// length instead of point-in-polygon (no bisection needed: a window
// boundary IS an arc-length value, so the cut point is a direct
// PointAtArcLength() call). Empty `windows` is a no-op: returns `pts`
// unchanged.
std::vector<std::vector<mpviz::Vec3>> ApplyCutWindows(
    const std::vector<mpviz::Vec3>& pts, const std::vector<double>& cum,
    const std::vector<std::pair<double, double>>& windows)
{
    std::vector<std::vector<mpviz::Vec3>> out;
    if (windows.empty())
    {
        out.push_back(pts);
        return out;
    }
    const double total = cum.back();
    double pos = 0.0;
    auto append_kept_range = [&](double from, double to) {
        std::vector<mpviz::Vec3> chain;
        chain.push_back(PointAtArcLength(pts, cum, from));
        for (size_t j = 0; j < pts.size(); ++j)
        {
            if (cum[j] > from && cum[j] < to) chain.push_back(pts[j]);
        }
        chain.push_back(PointAtArcLength(pts, cum, to));
        if (chain.size() >= 2) out.push_back(std::move(chain));
    };
    for (const auto& w : windows)
    {
        const double w_start = std::clamp(w.first, 0.0, total);
        const double w_end = std::clamp(w.second, 0.0, total);
        if (w_start > pos) append_kept_range(pos, w_start);
        pos = std::max(pos, w_end);
    }
    if (pos < total) append_kept_range(pos, total);
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
    junction_cut_points_.clear();
    std::unordered_map<uint32_t, const std::vector<mpviz::Vec3>*> left_by_lane, right_by_lane;
    // Every JUNCTION-kind ring this message carries (empty on urban's local/
    // global rows -- verified, they have no `junction` namespace rule at
    // all). Pointers alias storage_, same lifetime contract as everything
    // else fill() reads from it.
    std::vector<const std::vector<mpviz::Vec3>*> junction_polys;

    // First pass: index every boundary rail by lane_id, and collect every
    // JUNCTION ring. Needed BEFORE any element is emitted -- road-edge
    // detection (below) must see every OTHER lane's opposite-side boundary,
    // and road-surface pairing (further below) needs the same index. One
    // pass over storage_ builds both; a second walk (below) does the actual
    // emitting.
    for (const auto& [key, pieces] : storage_)
    {
        (void)key;
        for (const auto& elem : pieces)
        {
            if (elem.kind == mpviz::MapKind::JUNCTION) junction_polys.push_back(&elem.points);
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
    //
    // Junction cleanup (user directive 2026-09-08): a promoted ROAD_EDGE
    // element is NOT pushed straight to `out` here -- it is clipped against
    // `junction_polys` (a no-op when there are none) and queued in
    // `road_edge_pieces` so every promoted edge, from every marker, can be
    // checked against every OTHER one for the mutual-crossing cut below.
    // LEFT_/RIGHT_BOUNDARY gets the SAME polygon clip, pushed straight to
    // `out` (never queued -- boundaries never get the crossing cut), only
    // when `row_.junction_interior_boundaries` is false; the default (true)
    // leaves boundaries alone entirely, matching the refinement's own
    // "enable them by default".
    struct PendingRoadEdge
    {
        std::vector<mpviz::Vec3> points;
        uint32_t lane_id;
        double last_update_sec;
        uint8_t is_polygon;  // carried through the cut (review 2026-09-08):
                             // unreachable via today's shipped rules (only
                             // polyline boundaries promote), but a future
                             // `render: polygon, kind: road_edge` row must
                             // not silently lose the field.
    };
    std::vector<PendingRoadEdge> road_edge_pieces;

    for (const auto& [key, pieces] : storage_)
    {
        (void)key;
        for (const auto& elem : pieces)
        {
            mpviz::MapElement e{};
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

            if (e.kind == mpviz::MapKind::ROAD_EDGE)
            {
                for (auto& piece : ClipAgainstJunctions(elem.points, junction_polys))
                {
                    road_edge_pieces.push_back(PendingRoadEdge{std::move(piece), elem.lane_id,
                                                               e.last_update_sec, e.is_polygon});
                }
                continue;
            }
            if (IsBoundaryKind(e.kind) && !row_.junction_interior_boundaries &&
                !junction_polys.empty())
            {
                for (auto& piece : ClipAgainstJunctions(elem.points, junction_polys))
                {
                    junction_cut_points_.push_back(std::move(piece));
                    mpviz::MapElement be = e;  // same is_polygon/kind/lane_id/last_update_sec
                    be.points = junction_cut_points_.back().data();
                    be.point_count = static_cast<uint32_t>(junction_cut_points_.back().size());
                    out.map_elements.push_back(be);
                }
                continue;
            }

            e.points = elem.points.data();
            e.point_count = static_cast<uint32_t>(elem.points.size());
            out.map_elements.push_back(e);
        }
    }

    // Mutual-crossing cut (user directive 2026-09-08, decision #2): any two
    // ROAD_EDGE pieces from DIFFERENT lane_ids that cross in 2D each get a
    // kJunctionCutBackoffM window removed around their own crossing arc-
    // length station -- same-lane pairs are skipped (two pieces of the
    // SAME original polyline, split apart by the polygon clip above, are
    // never meant to cut each other). Every crossing a piece is party to
    // adds one raw window; MergeWindows folds overlapping ones from
    // several crossings on the same piece before ApplyCutWindows runs.
    std::vector<std::vector<double>> cum(road_edge_pieces.size());
    for (size_t i = 0; i < road_edge_pieces.size(); ++i)
    {
        cum[i] = CumulativeArcLength(road_edge_pieces[i].points);
    }
    std::vector<std::vector<std::pair<double, double>>> windows(road_edge_pieces.size());
    for (size_t i = 0; i < road_edge_pieces.size(); ++i)
    {
        const auto& pi = road_edge_pieces[i].points;
        for (size_t j = i + 1; j < road_edge_pieces.size(); ++j)
        {
            if (road_edge_pieces[i].lane_id == road_edge_pieces[j].lane_id) continue;
            const auto& pj = road_edge_pieces[j].points;
            for (size_t a = 0; a + 1 < pi.size(); ++a)
            {
                for (size_t b = 0; b + 1 < pj.size(); ++b)
                {
                    double t = 0.0, u = 0.0;
                    if (!SegSegIntersect2D(pi[a], pi[a + 1], pj[b], pj[b + 1], t, u)) continue;
                    const double s_i = cum[i][a] + t * (cum[i][a + 1] - cum[i][a]);
                    const double s_j = cum[j][b] + u * (cum[j][b + 1] - cum[j][b]);
                    windows[i].emplace_back(s_i - kJunctionCutBackoffM, s_i + kJunctionCutBackoffM);
                    windows[j].emplace_back(s_j - kJunctionCutBackoffM, s_j + kJunctionCutBackoffM);
                }
            }
        }
    }
    for (size_t i = 0; i < road_edge_pieces.size(); ++i)
    {
        MergeWindows(windows[i]);
        for (auto& piece : ApplyCutWindows(road_edge_pieces[i].points, cum[i], windows[i]))
        {
            junction_cut_points_.push_back(std::move(piece));
            mpviz::MapElement e{};
            e.points = junction_cut_points_.back().data();
            e.point_count = static_cast<uint32_t>(junction_cut_points_.back().size());
            e.is_polygon = road_edge_pieces[i].is_polygon;
            e.kind = mpviz::MapKind::ROAD_EDGE;
            e.lane_id = road_edge_pieces[i].lane_id;
            e.last_update_sec = road_edge_pieces[i].last_update_sec;
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
