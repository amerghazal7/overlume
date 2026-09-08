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
//      FOLLOW-UP (measurement pass 2026-09-08, user report "yellow
//      boundaries left overs (check junction corners) that looks messy"):
//      a real multi-lane junction crosses one through-edge SEVERAL times in
//      quick succession, not once -- each crossing's own window is
//      independent, and MergeWindows below now folds windows separated by
//      less than kJunctionGapMergeM into one merged cut (see that
//      constant's own comment for the measured bounds) instead of leaving
//      the small real gap between them as its own tiny rendered piece.
// O(edges^2 * segments^2) over this row's own promoted-ROAD_EDGE count
// (~15 edges x ~20 segments on the committed urban fixture) is fine at
// this scale -- no spatial index attempted.
constexpr double kJunctionCutBackoffM = 2.0;

// Junction gap-merge (measurement pass 2026-09-08, user report "yellow
// boundaries left overs (check junction corners) that looks messy"):
// generalizes the mutual-crossing cut above from 2 windows to N. A real
// multi-lane junction's through-edge crosses SEVERAL other ROAD_EDGE
// polylines in quick succession (once per crossing lane of the intersecting
// road); each crossing gets its own independent kJunctionCutBackoffM
// window, and MergeWindows below only merged windows that already
// touch/overlap -- two windows separated by a small real gap each survived
// as their own tiny emitted piece between them. Measured (CORRECTED,
// code-review finding, blocking: an earlier 43-message spot-sample (10s
// steps) reported max 4.010 m and missed the tail -- dense scan, every 8th
// /hd_map_local_elements message across this topic's ENTIRE recorded life,
// bag-relative +10.65s..+128.85s, 504 messages, 4265 emitted
// INTERIOR_SLIVER pieces): these leftover interior slivers range
// 0.020-6.302 m (median 2.709 m, unchanged from the spot-sample -- only the
// max was wrong) -- this IS the messy yellow left the screenshot shows. The
// 6.302 m case: promoted edge lane 792 crossed by lane 1320 (merged window
// [11.555,19.026]) and by lane 685 at 27.328 (window [25.328,29.328]),
// bag-relative +22.49s..+24.65s -- inter-window gap 6.302 m. 36 of the 4265
// slivers are >= 4.0 m, 10 are >= 6.0 m; re-running the dense scan with the
// old 6.0 m constant confirms the residual (4255 of 4265 slivers die, but
// the 6.302 m piece's 10 instances survive). The shortest real road
// fragment ever observed adjacent to a cut (a HEAD_STUB/TAIL_STUB -- real
// continuing road, not a sliver) is 7.027 m; nothing legitimate was ever
// observed between 6.302 m and 7.027 m. kJunctionGapMergeM = 6.6 m sits in
// that gap: ~0.30 m of margin above the largest observed sliver, ~0.43 m of
// margin below the shortest observed legit stub -- verified by rerunning
// the dense scan at gap=6.6: zero interior slivers remain and
// HEAD_STUB/TAIL_STUB/LEGIT are byte-for-byte unchanged (n=2710/2710/7101,
// floors still 7.027/7.863/10.731 -- nothing legitimate eaten). KNOWN
// CEILING: this is a merge-by-proximity heuristic, not a geometric proof --
// with kJunctionCutBackoffM=2.0 m of back-off on each side, it eats any
// real interior span whose two bounding crossings are separated by ~4-10.6
// m (inter-window gap 0-6.6 m, not observed as a legitimate span in this
// data); crossings farther apart than that are untouched. Unlike a bare
// min-piece-length filter, though, it generalizes correctly to a WIDER
// junction made of MORE crossings that are still individually close
// together, rather than needing a bigger constant every time; a genuinely
// large isolated interior (two crossings far apart on a wide multi-lane
// through-road) is out of scope for both approaches.
constexpr double kJunctionGapMergeM = 6.6;

// ---- Arc-aware cut refinement (measurement pass 2026-09-08, user directive:
// "leftover still exist, I suggest that as there are arcs on the inner
// coreners of the junction, start cutting of from the poin the arc starts,
// and if you drive through further another adjecent arc joins there we stop
// cutting off"):
//
// Root cause: the fixed kJunctionCutBackoffM=2.0 m window has no notion of
// where the ROAD_EDGE's own recorded curb geometry actually curves, so the
// boundary it lands on is at an arbitrary distance from any real corner
// fillet nearby -- inside it, at it, or past it, whichever the 2.0 m happens
// to hit for that particular crossing's geometry. CORRECTED (code-review
// finding 2026-09-08, blocking): an earlier version of this comment claimed
// "106 checked crossing-cut boundaries... landed STRICTLY INSIDE the arc's
// own span, never within 0.5 m of its true start/end" -- that population
// scan is not reproducible from anything committed to this repo, and the
// one instance that IS independently verifiable here (this fixture's own
// lane 792 x 685 crossing) contradicts it: the pre-fix boundary at
// arc-length 25.3276 sits 0.5052 m PAST lane 792's real R<20 run
// (17.1136-24.8224), i.e. just OUTSIDE the fillet, not inside it. Whichever
// side of the arc the fixed backoff happens to land on, the underlying
// defect is the same -- a distance-based cut with no notion of curvature at
// all -- and that is what the fix below addresses directly, not a
// direction-specific "always lands inside" claim this file cannot
// substantiate.
//
// Fix: after MergeWindows below folds nearby crossing windows together, an
// independent per-boundary pass (SnapWindowsToArcs) searches each merged
// window's own two boundaries, WITHIN kArcSearchMarginM of it, for a real
// corner arc on the EDGE'S OWN polyline nearby, then (CORRECTED, code-review
// finding 2026-09-08, blocking -- see kArcSearchMarginM's own comment)
// extends that candidate run outward past the search margin, while
// curvature keeps clearing kArcRadiusThresholdM, to its own true first/last
// vertex -- kArcSearchMarginM only LOCATES the candidate, it no longer
// bounds how far the run it belongs to is reached. The boundary then snaps
// OUTWARD (growing the cut, never shrinking it below the existing
// kJunctionCutBackoffM) to that true far edge -- "cut from where the arc
// starts" "...until it straightens again" (a real recorded vertex where
// curvature drops back to the road's own floor, not an interpolated point).
// Snapping w.first and w.second independently is what gives "if you drive
// through further another adjacent arc joins there we stop cutting off" for
// free, with no cross-polyline pairing logic: each boundary looks at its OWN
// nearby geometry only, so a window whose far side abuts a second, adjacent
// crossing's own arc grows to meet it just the same as any other arc.
//
// Composition -- AUGMENTS the existing crossing-cut + gap-merge, does NOT
// replace either: 75.2% of cut edges (measured) carry no arc at all (an
// open-pavement crossing with no curb connecting the two roads), and
// FindArcSpanNear returns false for every one of them, leaving that window
// exactly as MergeWindows produced it -- the fixed-backoff mechanism is
// still doing the right job there, unmodified. The refinement never touches
// an edge with no cut window at all (a LEGIT, never-crossed ROAD_EDGE that
// already renders whole, arc or not) -- it only ever adjusts a boundary
// that ALREADY exists, and only ever grows it.
constexpr double kArcRadiusThresholdM = 20.0;   // see FindArcSpanNear's own
                                                 // comment for the measured
                                                 // corner-vs-floor margin.
constexpr double kArcMinTotalTurnDeg = 15.0;    // ditto.
constexpr double kArcSearchMarginM = 6.0;       // ditto.

// Circumradius of the 2D (x,y) triangle (A,B,C) -- the library's own
// road-plane is flat, same "ignore z" choice every other 2D helper in this
// file already makes (PointInPolygonEvenOdd, SegSegIntersect2D). Returns
// +inf for (near-)collinear points: a straight run has no meaningful
// circumradius, and "infinite" correctly never clears
// kArcRadiusThresholdM, so a dead-straight stretch never counts as an arc.
double CircumradiusXY(const mpviz::Vec3& A, const mpviz::Vec3& B, const mpviz::Vec3& C)
{
    const double abx = B.x - A.x, aby = B.y - A.y;
    const double acx = C.x - A.x, acy = C.y - A.y;
    const double cross2 = std::abs(abx * acy - aby * acx);  // 2x triangle area
    if (cross2 < 1e-9) return std::numeric_limits<double>::infinity();
    const double a = std::hypot(C.x - B.x, C.y - B.y);
    const double b = std::hypot(C.x - A.x, C.y - A.y);
    const double c = std::hypot(B.x - A.x, B.y - A.y);
    return (a * b * c) / (2.0 * cross2);  // R = abc / (4*Area) = abc / (2*cross2)
}

// Turn angle (degrees, always >= 0) at vertex B between the incoming
// (A->B) and outgoing (B->C) directions.
double TurnAngleDeg(const mpviz::Vec3& A, const mpviz::Vec3& B, const mpviz::Vec3& C)
{
    const double d1x = B.x - A.x, d1y = B.y - A.y;
    const double d2x = C.x - B.x, d2y = C.y - B.y;
    return std::abs(std::atan2(d1x * d2y - d1y * d2x, d1x * d2x + d1y * d2y)) * 180.0 / M_PI;
}

// Searches `pts`'s own recorded vertices whose arc-length station (`cum`)
// falls within [b - kArcSearchMarginM, b + kArcSearchMarginM] for the
// longest contiguous run of INTERIOR vertices (index 1..size-2, each needs
// both neighbours to define curvature) whose own CircumradiusXY is under
// kArcRadiusThresholdM AND whose accumulated |TurnAngleDeg| over the run is
// at least kArcMinTotalTurnDeg. On a match, `lo`/`hi` are the run's own
// endpoint vertices' arc-length stations (real recorded points, never
// interpolated); returns false when nothing in the search window qualifies.
//
// Thresholds -- RE-MEASURED (code-review finding 2026-09-08, blocking: the
// original population/floor scans above compared each constant against the
// wrong population -- a per-corner or cross-population summary, not the
// quantity the gate actually gates). Re-derived directly against the
// per-INTERIOR-VERTEX values this code computes (stride-60 scan, 68
// messages of /hd_map_local_elements, 2313 interior vertices of promoted
// ROAD_EDGE geometry; run classification: 520 maximal contiguous
// R<kArcRadiusThresholdM vertex runs, 446 of them also clearing
// kArcMinTotalTurnDeg):
//   kArcRadiusThresholdM = 20.0 m -- gates the per-vertex CircumradiusXY,
//     not a per-corner summary radius. Measured: circumradius is a
//     continuum from 2.288 m to 332.009 m, with ZERO vertices (0/2313)
//     exactly collinear -- "every long polyline is dead straight" does not
//     hold in this data. 1503 vertices sit under 20 m, 594 in [20,60) m,
//     216 at/above 60 m, and 313 land in the dense [12,22] m band straddling
//     this threshold. The largest sub-threshold vertex is 19.4259 m and the
//     smallest supra-threshold vertex is 20.7210 m -- a 1.3 m (~6%) margin,
//     not the "~1.6x headroom" an earlier (wrong) per-corner comparison
//     claimed. 20.0 m is kept on that thin-but-real margin: it still
//     separates the two populations correctly on every vertex measured.
//   kArcMinTotalTurnDeg = 15.0 deg -- gates the accumulated |TurnAngleDeg|
//     of a maximal R<kArcRadiusThresholdM run, not a single vertex's own
//     kink. Measured: the largest REJECTED run (R<20 m throughout, still
//     under 15 deg total) turns 10.58 deg; the smallest ACCEPTED
//     (qualifying) run turns 17.05 deg -- a ~1.14x margin, not the "~5.4x"
//     an earlier (wrong) comparison against SegSegIntersect2D's own
//     kMinCrossingSinAngle population (a 2.8 deg lanelet-chain-node kink --
//     a DIFFERENT gate's own adversary, not this one's) claimed. 15.0 deg
//     is kept on that real, if narrower, margin.
//   kArcSearchMarginM = 6.0 m -- LOCATES a candidate arc vertex to search
//     from; it does NOT bound how far the run it belongs to actually
//     extends (that is real curb geometry, measured separately above).
//     CORRECTED (code-review finding 2026-09-08, blocking): an earlier
//     version of this comment described a KNOWN CEILING here -- "the snap
//     reaches only the reachable-within-margin vertex, not the run's true
//     first/last one" (measured: 7/141 snaps bag-wide truncated this way,
//     worst shortfall 5.065 m on lane 792's own 73.96 deg run, the
//     committed fixture's only crossing) -- but that ceiling described what
//     the CODE did, not what the surrounding comments (this block's own
//     "does not bound how far the run... extends", the top-of-file
//     refinement comment's "cut from where the arc starts... until it
//     straightens again") already promised. FindArcSpanNear's `close_run`
//     now actually does what those comments always claimed: once a run is
//     anchored inside the +/-6.0 m window, it extends outward vertex by
//     vertex, past the window, for as long as CircumradiusXY keeps clearing
//     kArcRadiusThresholdM -- so a run's own true start/end is always
//     reached regardless of how far it lies from the fixed-backoff
//     boundary; only kArcMinTotalTurnDeg qualification still has to occur
//     inside the window (that is what "LOCATES a candidate" means). Because
//     a snap can now land farther than kJunctionGapMergeM=6.6 m from its own
//     original boundary, fill() re-runs MergeWindows after SnapWindowsToArcs
//     (see that call site's own comment) instead of relying on a
//     never-cross-a-neighbour bound that no longer holds.
bool FindArcSpanNear(const std::vector<mpviz::Vec3>& pts, const std::vector<double>& cum, double b,
                     double& lo, double& hi)
{
    if (pts.size() < 3) return false;
    const double lo_bound = b - kArcSearchMarginM;
    const double hi_bound = b + kArcSearchMarginM;

    bool best_found = false;
    double best_lo = 0.0, best_hi = 0.0;
    bool in_run = false;
    size_t run_start = 0;
    double run_turn_deg = 0.0;

    auto close_run = [&](size_t run_end_vertex) {
        if (in_run && run_turn_deg >= kArcMinTotalTurnDeg)
        {
            // Extend the qualifying run outward past the search-window
            // bound (kArcSearchMarginM only LOCATED this run -- see that
            // constant's own comment) while the next vertex still clears
            // kArcRadiusThresholdM, so a run whose own true start/end lies
            // farther than the margin is still reached in full (code-review
            // finding 2026-09-08, blocking).
            size_t ext_start = run_start;
            while (ext_start > 1 &&
                   CircumradiusXY(pts[ext_start - 2], pts[ext_start - 1], pts[ext_start]) <
                       kArcRadiusThresholdM)
            {
                --ext_start;
            }
            size_t ext_end = run_end_vertex;
            while (ext_end + 2 < pts.size() &&
                   CircumradiusXY(pts[ext_end], pts[ext_end + 1], pts[ext_end + 2]) <
                       kArcRadiusThresholdM)
            {
                ++ext_end;
            }
            const double r_lo = cum[ext_start], r_hi = cum[ext_end];
            if (!best_found || (r_hi - r_lo) > (best_hi - best_lo))
            {
                best_lo = r_lo;
                best_hi = r_hi;
                best_found = true;
            }
        }
        in_run = false;
        run_turn_deg = 0.0;
    };

    for (size_t i = 1; i + 1 < pts.size(); ++i)
    {
        const bool in_window = cum[i] >= lo_bound && cum[i] <= hi_bound;
        const bool is_arc_vertex =
            in_window && CircumradiusXY(pts[i - 1], pts[i], pts[i + 1]) < kArcRadiusThresholdM;
        if (is_arc_vertex)
        {
            if (!in_run)
            {
                in_run = true;
                run_start = i;
                run_turn_deg = 0.0;
            }
            run_turn_deg += TurnAngleDeg(pts[i - 1], pts[i], pts[i + 1]);
        }
        else
        {
            close_run(i - 1);
        }
    }
    close_run(pts.size() - 2);

    if (best_found)
    {
        lo = best_lo;
        hi = best_hi;
    }
    return best_found;
}

// Snaps each of `windows`'s own boundaries outward to the far edge of a
// real corner arc found near it (FindArcSpanNear above), independently per
// boundary -- see this refinement's own top-of-block comment for the
// mechanism and composition decision. min()/max() below are what make this
// GROW-ONLY: a boundary with no qualifying arc nearby, or one that already
// sits at or beyond the arc's own far edge, is left untouched.
void SnapWindowsToArcs(const std::vector<mpviz::Vec3>& pts, const std::vector<double>& cum,
                       std::vector<std::pair<double, double>>& windows)
{
    for (auto& w : windows)
    {
        double lo = 0.0, hi = 0.0;
        if (FindArcSpanNear(pts, cum, w.first, lo, hi)) w.first = std::min(w.first, lo);
        if (FindArcSpanNear(pts, cum, w.second, lo, hi)) w.second = std::max(w.second, hi);
    }
}

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

// Sorts + merges [start,end] windows in place whose gap is under
// kJunctionGapMergeM (touching/overlapping windows, gap <= 0, always
// qualify) -- one polyline crossed by several others accumulates one raw
// window per crossing before this ever runs; see kJunctionGapMergeM's own
// comment for why a bare touch/overlap test left slivers between
// closely-spaced junction crossings.
void MergeWindows(std::vector<std::pair<double, double>>& windows)
{
    if (windows.empty()) return;
    std::sort(windows.begin(), windows.end());
    std::vector<std::pair<double, double>> merged;
    merged.push_back(windows.front());
    for (size_t i = 1; i < windows.size(); ++i)
    {
        if (windows[i].first <= merged.back().second + kJunctionGapMergeM)
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
        // Arc-aware refinement (user directive 2026-09-08, follow-up -- see
        // that constant block's own comment above): snaps each merged
        // window's own boundaries outward to a real corner arc found nearby
        // on THIS edge's own polyline. Runs after MergeWindows (so it only
        // ever refines an already-decided set of cut regions, never changes
        // which crossings get cut) and before ApplyCutWindows (so the
        // extended boundaries are what actually gets removed). CORRECTED
        // (code-review finding 2026-09-08, blocking): FindArcSpanNear now
        // extends a qualifying run past kArcSearchMarginM out to its own
        // true start/end (see that function's own comment), so a snap is no
        // longer bounded to the +/-6.0 m search window -- the
        // ascending-by-start argument this comment used to make (every
        // snap stays under kJunctionGapMergeM=6.6 m of its own original
        // boundary) no longer holds, and a window's boundary CAN now cross
        // a neighbour's original boundary. Re-running MergeWindows below
        // (it sorts + merges) is what restores both invariants
        // ApplyCutWindows relies on, rather than re-deriving a bound on how
        // far a single snap can reach.
        SnapWindowsToArcs(road_edge_pieces[i].points, cum[i], windows[i]);
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
