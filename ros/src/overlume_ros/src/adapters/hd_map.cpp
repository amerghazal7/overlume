// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/hd_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume_node {
namespace {

// visualization_msgs/msg/Marker.msg action + type constants -- not worth a
// dependency on the generated enum names for six values used once each.
constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionModify = 1;
constexpr int32_t kActionDelete = 2;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kMarkerTypeLineStrip = 4;

bool HasNan(const overlume::Vec3& p) {
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

// Marker.msg semantics: points[] on a LINE_STRIP are RELATIVE to
// marker.pose -- composed as pose * point before anything else touches the
// geometry (rviz parity), including dash chopping below. Identity pose (the
// common case) skips the multiply, mirroring FrameTransformer's own
// identity-frame shortcut (frame_transform.cpp).
bool MarkerPoseIsIdentity(const geometry_msgs::msg::Pose& p) {
    constexpr double kEps = 1e-12;
    return std::abs(p.position.x) < kEps && std::abs(p.position.y) < kEps &&
           std::abs(p.position.z) < kEps && std::abs(p.orientation.x) < kEps &&
           std::abs(p.orientation.y) < kEps && std::abs(p.orientation.z) < kEps &&
           std::abs(p.orientation.w - 1.0) < kEps;
}

// A NaN marker.pose is malformed Marker data (existing dropped_malformed
// path) -- a NON-identity pose is normal Marker semantics, not malformed.
bool MarkerPoseHasNan(const geometry_msgs::msg::Pose& p) {
    return std::isnan(p.position.x) || std::isnan(p.position.y) || std::isnan(p.position.z) ||
           std::isnan(p.orientation.x) || std::isnan(p.orientation.y) ||
           std::isnan(p.orientation.z) || std::isnan(p.orientation.w);
}

// The number of stations both boundary rails are resampled to, by
// normalized arc length, before being zipped into a triangle strip
// library-side. Fixed; ROAD_SURFACE's point_count is always
// 2*kRoadFillSamples.
constexpr uint32_t kRoadFillSamples = 16;

// e.last_update_sec is stamped last_recv_sec_ + (row_.timeout_sec -
// kMapFadeWindowSec): the fade window is anchored to the LAST
// kMapFadeWindowSec before the row's own timeout_sec cutoff, not to
// kStaleFadeTimeoutSec after the last receipt. This keeps a fast-received
// row continuously opaque (no sawtooth on TF receipt gaps) and turns a
// stopped/transient_local row's fade into an opacity ramp ending exactly at
// its own timeout_sec cutoff, instead of a pop.
//
// kMapFadeWindowSec MIRRORS the library's kStaleFadeTimeoutSec
// (overlume/src/renderer_internal.hpp:124, library-internal, not
// reachable from node-side code) -- keep in sync with that constant.
// profile.cpp's row.timeout_sec >= 1.0 validation guarantees
// row_.timeout_sec - kMapFadeWindowSec is never negative.
constexpr double kMapFadeWindowSec = 1.0;

// cum[0] == 0, cum[i] == arc length from pts[0] to pts[i], cum.back() ==
// total polyline length. Shared by ResampleByArcLength (road-surface fill)
// below and the junction-cleanup clip/cut machinery further down -- both
// need to convert "a point somewhere along this polyline" into/from a
// normalized arc-length station.
std::vector<double> CumulativeArcLength(const std::vector<overlume::Vec3>& pts) {
    std::vector<double> cum(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
        const double dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y,
                     dz = pts[i].z - pts[i - 1].z;
        cum[i] = cum[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return cum;
}

// Interpolated point at arc length `s` (clamped to [0, cum.back()]) along
// `pts`, using its own precomputed `cum` (CumulativeArcLength(pts)).
overlume::Vec3 PointAtArcLength(const std::vector<overlume::Vec3>& pts,
                                const std::vector<double>& cum, double s) {
    const double total_len = cum.back();
    s = std::clamp(s, 0.0, total_len);
    size_t i = static_cast<size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
    if (i == 0) i = 1;
    if (i >= pts.size()) i = pts.size() - 1;
    const double seg_len = cum[i] - cum[i - 1];
    const double t = seg_len > 0.0 ? (s - cum[i - 1]) / seg_len : 0.0;
    const auto& a = pts[i - 1];
    const auto& b = pts[i];
    return overlume::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

// Resamples `pts` to exactly `n_stations` points, evenly spaced by
// normalized arc length. Two rails of a lane are not guaranteed to carry
// the same point count on real data (e.g. lane 955 is 8/9 points, lane 813
// is 10/11, in the committed fixture) -- resampling both to a fixed
// station count is what lets the library zip them into a strip without
// indexing past either rail's own array. Returns empty for fewer than 2
// input points or n_stations == 0.
std::vector<overlume::Vec3> ResampleByArcLength(const std::vector<overlume::Vec3>& pts,
                                                uint32_t n_stations) {
    std::vector<overlume::Vec3> out;
    if (pts.size() < 2 || n_stations == 0) return out;

    const std::vector<double> cum = CumulativeArcLength(pts);
    const double total_len = cum.back();

    out.reserve(n_stations);
    for (uint32_t k = 0; k < n_stations; ++k) {
        const double s = (n_stations == 1) ? 0.0
                                           : total_len * static_cast<double>(k) /
                                                 static_cast<double>(n_stations - 1);
        out.push_back(PointAtArcLength(pts, cum, s));
    }
    return out;
}

// Kinds that carry a lane_id (the marker's own `id`) -- crosswalk/stopline/
// junction/other are not lane-paired. ROAD_EDGE carries one too: fill()'s
// promotion pass relabels a LEFT_/RIGHT_BOUNDARY to ROAD_EDGE in place,
// keeping its lane_id; no ingest-time namespace rule ever produces
// ROAD_EDGE directly.
bool KindCarriesLaneId(overlume::MapKind kind) {
    return kind == overlume::MapKind::CENTERLINE || kind == overlume::MapKind::LEFT_BOUNDARY ||
           kind == overlume::MapKind::RIGHT_BOUNDARY || kind == overlume::MapKind::ROAD_EDGE;
}

// Road-edge detection: a LEFT_BOUNDARY/RIGHT_BOUNDARY promotes to ROAD_EDGE
// when no OTHER lane's opposite-side boundary sits within
// kRoadEdgeCoincidenceThresholdM of it (sampled at its own two endpoints +
// midpoint) -- an interior divider has a near-duplicate survey on the
// neighbour's opposite side (the SAME painted line, surveyed twice); an
// outer edge does not.
//
// kRoadEdgeCoincidenceThresholdM = 1.0 m: measured against the committed
// hd_map_local_elements_0.yaml fixture, genuine twin separations run
// 0.00-0.20 m (several are exact float duplicates) and the nearest genuine
// non-twin sits at 1.7955 m (bulk of non-adjacent lanes at 3.1-3.7 m, ~one
// lane width) -- see plan 2026-08-18-visual-mode-epic3.md for the full
// worked measurement. On this fixture: 15 of 32 boundary elements promote
// to ROAD_EDGE (lane 934 is fully interior -- paired both sides -- and
// contributes none).
constexpr double kRoadEdgeCoincidenceThresholdM = 1.0;

double PointToPolylineDist2D(const overlume::Vec3& p, const std::vector<overlume::Vec3>& poly) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        const auto& a = poly[i];
        const auto& b = poly[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 1e-12) {
            t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double cx = a.x + t * dx, cy = a.y + t * dy;
        const double d = std::hypot(p.x - cx, p.y - cy);
        if (d < best) best = d;
    }
    return best;
}

// Arc-length station (into `cum`, `poly`'s own precomputed
// CumulativeArcLength) of the closest point on `poly` to `p` -- same
// nearest-point-on-segment search as PointToPolylineDist2D above, but
// returning WHERE the closest point sits rather than how far away it is.
// Used by SnapWindowsToNeighborArcDepartures to project a neighbour piece's
// own corner-arc departure vertex onto this piece's polyline: two
// independently-surveyed curbs sharing a junction node run near-coincident
// right up to where one peels away, so the nearest point on the straight
// piece is, to survey precision, the same physical place.
double NearestStationOnPolyline(const overlume::Vec3& p, const std::vector<overlume::Vec3>& poly,
                                const std::vector<double>& cum) {
    double best_d2 = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        const auto& a = poly[i];
        const auto& b = poly[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 1e-12) {
            t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double cx = a.x + t * dx, cy = a.y + t * dy;
        const double d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
        if (d2 < best_d2) {
            best_d2 = d2;
            best_s = cum[i] + t * (cum[i + 1] - cum[i]);
        }
    }
    return best_s;
}

// True when `pts` (this lane's own boundary, `lane_id`) has no coincident
// twin in `opposite_by_lane` (every OTHER lane's opposite-side boundary) --
// i.e. it is the road's outer edge. Sampled at pts' own first/mid/last
// point, each checked against a candidate's FULL polyline extent (not just
// its endpoints), so a candidate that only grazes one sample still counts
// as coincident.
bool IsRoadEdge(
    uint32_t lane_id, const std::vector<overlume::Vec3>& pts,
    const std::unordered_map<uint32_t, const std::vector<overlume::Vec3>*>& opposite_by_lane) {
    if (pts.size() < 2) return false;
    const overlume::Vec3 samples[3] = {pts.front(), pts[pts.size() / 2], pts.back()};
    for (const auto& [other_lane, other_pts] : opposite_by_lane) {
        if (other_lane == lane_id || other_pts == nullptr) continue;
        for (const auto& s : samples) {
            if (PointToPolylineDist2D(s, *other_pts) < kRoadEdgeCoincidenceThresholdM) return false;
        }
    }
    return true;
}

// ---- Junction cleanup: two independent cut mechanisms, both run from
// fill() below:
//   1. JUNCTION-POLYGON CLIP (ClipAgainstJunctions): where the message
//      carries MapKind::JUNCTION geometry (today: only /sim/hd_map/
//      markers's `junction` namespace rule -- urban_profile.yaml has none)
//      a ROAD_EDGE polyline is clipped against the union of that message's
//      JUNCTION rings. LEFT_/RIGHT_BOUNDARY gets the identical clip only
//      when the row's `junction_interior_boundaries` is false (default
//      true).
//   2. MUTUAL-CROSSING CUT (SegSegIntersect2D + the window machinery):
//      works with no junction data at all -- any two ROAD_EDGE polylines
//      from DIFFERENT lane_ids that cross in 2D at a real angle get
//      trimmed back kJunctionCutBackoffM from the crossing point, both
//      sides continuing beyond. Never applied to boundaries: interior
//      separators legitimately cross connector geometry inside a junction.
//      SegSegIntersect2D rejects near-parallel AND near-antiparallel
//      segment pairs (not just exactly-parallel ones -- see that
//      function's own comment for the crossing-angle classification that
//      motivated it) so a lanelet-chain node's ordinary survey kink at a
//      shared node is never mistaken for a crossing. MergeWindows below
//      folds windows separated by less than kJunctionGapMergeM into one cut
//      (see that constant's own comment) instead of leaving small real
//      gaps between close-together crossings as their own tiny rendered
//      pieces. See plan 2026-08-18-visual-mode-epic3.md for the full
//      derivation of both mechanisms.
// O(edges^2 * segments^2) over this row's own promoted-ROAD_EDGE count
// (~15 edges x ~20 segments on the committed urban fixture) is fine at
// this scale -- no spatial index attempted.
constexpr double kJunctionCutBackoffM = 2.0;

// Junction gap-merge: generalizes the mutual-crossing cut above from 2
// windows to N -- a multi-lane junction's through-edge crosses several
// other ROAD_EDGE polylines in quick succession, and a bare touch/overlap
// merge left the small real gaps between those windows as their own tiny
// rendered slivers.
//
// kJunctionGapMergeM = 6.6 m: measured via a dense scan (every 8th
// /hd_map_local_elements message across the topic's full recorded life, 504
// messages) of leftover interior-sliver lengths (0.020-6.302 m, median
// 2.709 m) against the shortest real road fragment ever observed adjacent
// to a cut (7.027 m) -- 6.6 m sits in that gap (~0.30 m margin above the
// largest sliver, ~0.43 m below the shortest legit stub); see plan
// 2026-08-18-visual-mode-epic3.md for the full scan. ponytail: a
// merge-by-proximity heuristic, not a geometric proof -- it eats any real
// interior span whose bounding crossings are separated by roughly 4-10.6 m
// (not observed as legitimate in this data); farther-apart crossings are
// untouched. Generalizes to a wider junction of more closely-spaced
// crossings without a bigger constant; a genuinely large isolated interior
// on a wide multi-lane through-road would need one, upgrade then.
constexpr double kJunctionGapMergeM = 6.6;

// ---- Arc-aware cut refinement: the fixed kJunctionCutBackoffM=2.0 m
// window has no notion of where the ROAD_EDGE's own curb geometry actually
// curves, so the cut boundary lands at an arbitrary distance from any real
// corner fillet nearby.
//
// Fix: after MergeWindows below folds nearby crossing windows together, an
// independent per-boundary pass (SnapWindowsToArcs) searches each merged
// window's own two boundaries, within kArcSearchMarginM of it, for a real
// corner arc on the edge's own polyline nearby, then extends that
// candidate run outward -- while curvature keeps clearing
// kArcRadiusThresholdM -- to its own true first/last vertex.
// kArcSearchMarginM only LOCATES the candidate; it does not bound how far
// the run it belongs to is reached. The boundary then snaps OUTWARD
// (growing the cut, never shrinking it below kJunctionCutBackoffM) to that
// true far edge: a real recorded vertex where curvature drops back to the
// road's own floor, never interpolated. Snapping each window's two ends
// independently is what lets an adjacent crossing's own arc "join up" for
// free, with no cross-polyline pairing logic.
//
// Composition: augments the existing crossing-cut + gap-merge, does not
// replace either -- 75.2% of cut edges (measured) carry no arc at all, and
// FindArcSpanNear returns false for those, leaving the window exactly as
// MergeWindows produced it. Never touches an edge with no cut window at
// all. See plan 2026-08-18-visual-mode-epic3.md for the full derivation.
constexpr double kArcRadiusThresholdM = 20.0;  // see FindArcSpanNear's own
                                               // comment for the measured
                                               // corner-vs-floor margin.
constexpr double kArcMinTotalTurnDeg = 15.0;   // ditto.
constexpr double kArcSearchMarginM = 6.0;      // ditto.

// Circumradius of the 2D (x,y) triangle (A,B,C) -- the library's own
// road-plane is flat, same "ignore z" choice every other 2D helper in this
// file already makes (PointInPolygonEvenOdd, SegSegIntersect2D). Returns
// +inf for (near-)collinear points: a straight run has no meaningful
// circumradius, and "infinite" correctly never clears
// kArcRadiusThresholdM, so a dead-straight stretch never counts as an arc.
double CircumradiusXY(const overlume::Vec3& A, const overlume::Vec3& B, const overlume::Vec3& C) {
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
double TurnAngleDeg(const overlume::Vec3& A, const overlume::Vec3& B, const overlume::Vec3& C) {
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
// Once a run is anchored inside the +/-kArcSearchMarginM window, it extends
// outward vertex by vertex past the window, for as long as CircumradiusXY
// keeps clearing kArcRadiusThresholdM, so a run's own true start/end is
// always reached regardless of how far it lies from the fixed-backoff
// boundary; only the turn-angle qualification has to occur inside the
// window. A snap can therefore land farther than kJunctionGapMergeM=6.6 m
// from its own original boundary -- fill() re-runs MergeWindows after
// SnapWindowsToArcs for exactly this reason.
//
// Thresholds, measured against a stride-60 scan of /hd_map_local_elements
// (68 messages, 2313 interior vertices of promoted ROAD_EDGE geometry; see
// plan 2026-08-18-visual-mode-epic3.md for the full scan):
//   kArcRadiusThresholdM = 20.0 m -- per-vertex circumradius ranges 2.288-
//     332.009 m with zero exactly-collinear vertices; largest sub-threshold
//     vertex 19.4259 m, smallest supra-threshold 20.7210 m (~6% margin).
//   kArcMinTotalTurnDeg = 15.0 deg -- gates the accumulated turn of a
//     maximal R<20m run; largest rejected run turns 10.58 deg, smallest
//     accepted run turns 17.05 deg (~1.14x margin).
//   kArcSearchMarginM = 6.0 m -- only locates a candidate arc vertex to
//     search from; does not bound how far the run it belongs to extends
//     (that is real curb geometry, measured above).
bool FindArcSpanNear(const std::vector<overlume::Vec3>& pts, const std::vector<double>& cum,
                     double b, double& lo, double& hi) {
    if (pts.size() < 3) return false;
    const double lo_bound = b - kArcSearchMarginM;
    const double hi_bound = b + kArcSearchMarginM;

    bool best_found = false;
    double best_lo = 0.0, best_hi = 0.0;
    bool in_run = false;
    size_t run_start = 0;
    double run_turn_deg = 0.0;

    auto close_run = [&](size_t run_end_vertex) {
        if (in_run && run_turn_deg >= kArcMinTotalTurnDeg) {
            // Extend the qualifying run outward past the search-window
            // bound (kArcSearchMarginM only locates this run) while the
            // next vertex still clears kArcRadiusThresholdM, so a run's own
            // true start/end is reached in full even when farther than the
            // margin.
            size_t ext_start = run_start;
            while (ext_start > 1 && CircumradiusXY(pts[ext_start - 2], pts[ext_start - 1],
                                                   pts[ext_start]) < kArcRadiusThresholdM) {
                --ext_start;
            }
            size_t ext_end = run_end_vertex;
            while (ext_end + 2 < pts.size() &&
                   CircumradiusXY(pts[ext_end], pts[ext_end + 1], pts[ext_end + 2]) <
                       kArcRadiusThresholdM) {
                ++ext_end;
            }
            const double r_lo = cum[ext_start], r_hi = cum[ext_end];
            if (!best_found || (r_hi - r_lo) > (best_hi - best_lo)) {
                best_lo = r_lo;
                best_hi = r_hi;
                best_found = true;
            }
        }
        in_run = false;
        run_turn_deg = 0.0;
    };

    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        const bool in_window = cum[i] >= lo_bound && cum[i] <= hi_bound;
        const bool is_arc_vertex =
            in_window && CircumradiusXY(pts[i - 1], pts[i], pts[i + 1]) < kArcRadiusThresholdM;
        if (is_arc_vertex) {
            if (!in_run) {
                in_run = true;
                run_start = i;
                run_turn_deg = 0.0;
            }
            run_turn_deg += TurnAngleDeg(pts[i - 1], pts[i], pts[i + 1]);
        } else {
            close_run(i - 1);
        }
    }
    close_run(pts.size() - 2);

    if (best_found) {
        lo = best_lo;
        hi = best_hi;
    }
    return best_found;
}

// A promoted-but-not-yet-cut ROAD_EDGE piece, queued in fill() below so
// every promoted edge from every marker can be checked against every other
// one for the mutual-crossing cut, arc-snap, and redundant-arc-tail trim.
struct PendingRoadEdge {
    std::vector<overlume::Vec3> points;
    uint32_t lane_id;
    double last_update_sec;
    uint8_t is_polygon;  // carried through the cut: unreachable via today's
                         // shipped rules (only polyline boundaries
                         // promote), but a future `render: polygon,
                         // kind: road_edge` row must not silently lose it.
};

// Snaps each of `windows`'s own boundaries outward to the far edge of a
// real corner arc found near it (FindArcSpanNear above), independently per
// boundary -- see this refinement's own top-of-block comment for the
// mechanism and composition decision. min()/max() below are what make this
// GROW-ONLY: a boundary with no qualifying arc nearby, or one that already
// sits at or beyond the arc's own far edge, is left untouched.
void SnapWindowsToArcs(const std::vector<overlume::Vec3>& pts, const std::vector<double>& cum,
                       std::vector<std::pair<double, double>>& windows) {
    for (auto& w : windows) {
        double lo = 0.0, hi = 0.0;
        if (FindArcSpanNear(pts, cum, w.first, lo, hi)) w.first = std::min(w.first, lo);
        if (FindArcSpanNear(pts, cum, w.second, lo, hi)) w.second = std::max(w.second, hi);
    }
}

// ---- Redundant arc-tail trim: the arc-bearing piece's OWN recorded tail
// continues past its own corner arc's rejoin vertex (the same "straightens
// again" point SnapWindowsToArcs already looks for), running the rest of
// its length within kRoadEdgeCoincidenceThresholdM of a DIFFERENT,
// independently-promoted ROAD_EDGE piece, before the two converge at an
// EXACT shared vertex (two independent surveys of the same physical curb
// past that point).
//
// Discriminator (measured against all 19 distinct bag-wide instances, no
// ambiguous middle case; see plan 2026-08-18-visual-mode-epic3.md for the
// full derivation): (a) this piece's own last vertex sits within
// kSharedNodeEpsM of a DIFFERENT piece's own endpoint -- this alone does no
// discriminating work (16/19 instances, true and false alike, sit at an
// identical 0.0000 m match here); its only real job is excluding the 3
// genuinely free-floating tails (nearest 3.43 m). (b) The vertex just
// before that is ALSO within kRoadEdgeCoincidenceThresholdM of that SAME
// other piece's full polyline (true matches 0.0055-0.8883 m; nearest false
// match 1.5366 m -- a 1.73x margin). Condition (b), tested existentially
// over every condition-(a)-tied candidate (not just the nearest one -- see
// TrimRedundantArcTails' own comment on why order-independence matters
// here), is what actually separates the 14 real cases from the 5 false
// ones.
//
// Trim point: exactly the arc's own rejoin vertex -- a real recorded point,
// symmetric to SnapWindowsToArcs' own "cut from where the arc starts" now
// applied to where it straightens back out. Structurally cannot fire on the
// arc's own upstream approach: it only looks at vertices after the arc's
// own rejoin vertex.
//
// Composition: a separate per-piece scan of the piece's OWN geometry
// (FindLastArcRunEnd below), independent of there being a nearby crossing
// window at all. It only ever appends/extends one more window on the tail
// side of an already-qualifying arc; never changes what counts as a
// crossing, never re-opens the gap-merge, never touches a piece with no
// qualifying arc.
constexpr double kSharedNodeEpsM = 0.10;

// ---- Straight-neighbour overshoot snap: symmetric counterpart of
// SnapWindowsToArcs above. That function snaps a window on the piece that
// OWNS a nearby arc outward to the arc's own far edge; this one snaps a
// window on the STRAIGHT NEIGHBOUR that shares the arc's own departure node
// back to that same vertex, so the two meet exactly where the curb starts
// curving away instead of at an arbitrary fixed distance
// (kJunctionCutBackoffM has no notion of where an adjacent lane's curb
// starts curving). GROW-ONLY: only ever pulls a boundary back toward the
// shared node (std::min/std::max against the boundary's own already-cut
// position), so a boundary already at or before the departure vertex is
// left untouched.
//
// No new threshold: reuses kSharedNodeEpsM (does this candidate share the
// node at all) and kArcRadiusThresholdM/kArcMinTotalTurnDeg (is that
// neighbour actually curving away there) -- both already measured
// elsewhere in this file for a different purpose.
//
// Self-limiting the same way TrimRedundantArcTails is: only ever tightens a
// window that already exists from the ordinary crossing-cut, and only
// tightens a boundary that is actually past the projected departure
// vertex.
bool FindArcDepartureFromEnd(const std::vector<overlume::Vec3>& pts, bool from_back, size_t& idx) {
    const size_t n = pts.size();
    if (n < 3) return false;
    // Walks `pts` from whichever end `from_back` selects, via an index
    // remap (`at`), so the exact same first-qualifying-run scan works
    // symmetrically from either end -- `k` counts vertices IN from that
    // end, `at(k)` is the corresponding real index into `pts`.
    auto at = [&](size_t k) -> const overlume::Vec3& {
        return from_back ? pts[n - 1 - k] : pts[k];
    };

    bool in_run = false;
    size_t run_start_k = 0;
    double run_turn_deg = 0.0;
    for (size_t k = 1; k + 1 < n; ++k) {
        if (CircumradiusXY(at(k - 1), at(k), at(k + 1)) < kArcRadiusThresholdM) {
            if (!in_run) {
                in_run = true;
                run_start_k = k;
                run_turn_deg = 0.0;
            }
            run_turn_deg += TurnAngleDeg(at(k - 1), at(k), at(k + 1));
        } else if (in_run) {
            // Unlike FindLastArcRunEnd (which deliberately keeps
            // overwriting to find the LAST run for the redundant-tail
            // trim), this returns the FIRST qualifying run and stops -- a
            // neighbour sharing THIS end's own node only cares about the
            // nearest corner peeling away from it, never one further down
            // the piece.
            if (run_turn_deg >= kArcMinTotalTurnDeg) {
                idx = from_back ? (n - 1 - run_start_k) : run_start_k;
                return true;
            }
            in_run = false;
            run_turn_deg = 0.0;
        }
    }
    if (in_run && run_turn_deg >= kArcMinTotalTurnDeg) {
        idx = from_back ? (n - 1 - run_start_k) : run_start_k;
        return true;
    }
    return false;
}

// Searches every OTHER promoted piece for one that (a) shares `node`
// (`self`'s own start or end vertex) at one of its own two endpoints, within
// kSharedNodeEpsM, and (b) carries a corner-arc run departing from that same
// endpoint (FindArcDepartureFromEnd above). On a match, `station_out` is
// that departure vertex projected onto `self`'s own polyline
// (NearestStationOnPolyline) -- the station self's own crossing-cut
// boundary at node's end must not run past. First qualifying match wins:
// at most one neighbour sharing a junction node is expected to carry a
// corner arc departing from it.
bool FindNeighborArcDepartureStation(const std::vector<PendingRoadEdge>& pieces, size_t self,
                                     const std::vector<std::vector<double>>& cum,
                                     const overlume::Vec3& node, double& station_out) {
    const auto& pts_self = pieces[self].points;
    for (size_t j = 0; j < pieces.size(); ++j) {
        if (j == self || pieces[j].points.size() < 3) continue;
        const auto& pts_j = pieces[j].points;
        const double d_front = std::hypot(node.x - pts_j.front().x, node.y - pts_j.front().y);
        const double d_back = std::hypot(node.x - pts_j.back().x, node.y - pts_j.back().y);
        bool coincident_back = false;
        if (d_front < kSharedNodeEpsM) {
            coincident_back = false;
        } else if (d_back < kSharedNodeEpsM) {
            coincident_back = true;
        } else {
            continue;
        }
        size_t dep_idx = 0;
        if (!FindArcDepartureFromEnd(pts_j, coincident_back, dep_idx)) continue;
        station_out = NearestStationOnPolyline(pts_j[dep_idx], pts_self, cum[self]);
        return true;
    }
    return false;
}

// Applies the snap above to `pieces[self]`'s own two OUTER window
// boundaries only (windows.front().first, windows.back().second) -- the
// ones adjoining pieces[self]'s own literal start/end vertex, the only ones
// a shared-node neighbour's own departure vertex is relevant to. A no-op
// when `windows` is empty (never introduces a cut on a piece the ordinary
// crossing-cut never touched at all). Merges directly into place:
// tightening only the two outermost boundaries can never overlap an inner
// window (already sorted, non-overlapping) and never needs another
// MergeWindows pass.
void SnapWindowsToNeighborArcDepartures(const std::vector<PendingRoadEdge>& pieces, size_t self,
                                        const std::vector<std::vector<double>>& cum,
                                        std::vector<std::pair<double, double>>& windows) {
    if (windows.empty()) return;
    const auto& pts_self = pieces[self].points;
    if (pts_self.empty()) return;

    double station = 0.0;
    if (FindNeighborArcDepartureStation(pieces, self, cum, pts_self.front(), station)) {
        windows.front().first = std::min(windows.front().first, station);
    }
    if (FindNeighborArcDepartureStation(pieces, self, cum, pts_self.back(), station)) {
        windows.back().second = std::max(windows.back().second, station);
    }
}

// Same per-vertex circumradius/turn-angle gates as FindArcSpanNear above,
// but scanning the WHOLE polyline (not bounded to a +/- kArcSearchMarginM
// window around an existing cut boundary) for its LAST (highest-station)
// qualifying corner-arc run -- TrimRedundantArcTails needs the piece's own
// last arc regardless of whether any crossing-cut window touches it at
// all. `hi_idx` is that run's own far vertex INDEX (a real recorded
// point); returns false when no qualifying run exists anywhere in `pts`.
bool FindLastArcRunEnd(const std::vector<overlume::Vec3>& pts, size_t& hi_idx) {
    if (pts.size() < 3) return false;
    bool found = false;
    bool in_run = false;
    double run_turn_deg = 0.0;

    auto close_run = [&](size_t run_end_vertex) {
        if (in_run && run_turn_deg >= kArcMinTotalTurnDeg) {
            hi_idx = run_end_vertex;  // later (higher-station) runs overwrite on purpose
            found = true;
        }
        in_run = false;
        run_turn_deg = 0.0;
    };

    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        if (CircumradiusXY(pts[i - 1], pts[i], pts[i + 1]) < kArcRadiusThresholdM) {
            if (!in_run) {
                in_run = true;
                run_turn_deg = 0.0;
            }
            run_turn_deg += TurnAngleDeg(pts[i - 1], pts[i], pts[i + 1]);
        } else {
            close_run(i - 1);
        }
    }
    close_run(pts.size() - 2);
    return found;
}

// Appends (or extends) one more trim window covering `pieces[self]`'s own
// tail past its last corner arc's rejoin vertex, when that tail is the
// redundant-duplicate class measured above. `windows` is `pieces[self]`'s
// own already-sorted, non-overlapping cut-window list (post arc-snap); a
// no-op when no qualifying arc exists in this piece, or when the
// discriminator does not fire. ponytail: the "other" endpoint search
// compares against every OTHER promoted ROAD_EDGE piece's own two literal
// endpoints, pre-cut -- the real convergence vertex is a piece's own outer
// endpoint, never inside any of its own cut windows, so this is exactly as
// accurate as comparing against final post-cut pieces and needs no
// ordering dependency on when piece j's own cut runs.
void TrimRedundantArcTails(const std::vector<PendingRoadEdge>& pieces, size_t self,
                           const std::vector<std::vector<double>>& cum,
                           std::vector<std::pair<double, double>>& windows) {
    const auto& pts = pieces[self].points;
    size_t hi_idx = 0;
    if (!FindLastArcRunEnd(pts, hi_idx)) return;

    const overlume::Vec3& far = pts.back();
    const overlume::Vec3& penult = pts[pts.size() - 2];

    // Order-independent by design: fires when ANY other piece satisfies both
    // conditions, not just a single nearest-by-condition-(a) pick. 16 of the
    // 19 measured bag-wide instances tie at an identical 0.0000 m
    // condition-(a) match (up to 4 pieces tied inside kSharedNodeEpsM), so
    // picking "the nearest" would be decided by iteration order over
    // `pieces` (walked from storage_, an unordered_map) rather than by
    // geometry -- a message-order shuffle could silently flip the same
    // corner between trimmed and un-trimmed frame to frame. Testing every
    // condition-(a)-tied candidate and firing on any qualifying match is
    // deterministic given the SET of pieces present, and is the
    // geometrically correct question: "is this tail a duplicate of SOME
    // other independently-promoted piece", not "of whichever piece is
    // closest by endpoint alone".
    bool duplicate_found = false;
    for (size_t j = 0; j < pieces.size() && !duplicate_found; ++j) {
        // Same-lane pieces never cut each other -- the same rule the
        // mutual-crossing cut applies (two pieces of one polyline split
        // apart by the polygon clip are not independent surveys).
        if (j == self || pieces[j].lane_id == pieces[self].lane_id || pieces[j].points.size() < 2)
            continue;
        const auto& other = pieces[j].points;
        const double cond_a = std::min(std::hypot(far.x - other.front().x, far.y - other.front().y),
                                       std::hypot(far.x - other.back().x, far.y - other.back().y));
        if (cond_a >= kSharedNodeEpsM) continue;
        if (PointToPolylineDist2D(penult, other) < kRoadEdgeCoincidenceThresholdM) {
            duplicate_found = true;
        }
    }
    if (!duplicate_found) return;

    // Merge directly into place (never via MergeWindows -- see this
    // block's own top comment): the trim always runs to the piece's own
    // literal end, so either it extends an already-later-reaching last
    // window (no new entry needed) or it appends a new one strictly past
    // it -- both keep `windows` sorted/non-overlapping for ApplyCutWindows.
    const double trim_start = cum[self][hi_idx];
    const double piece_end = cum[self].back();
    if (!windows.empty() && windows.back().second >= trim_start) {
        windows.back().second = piece_end;
    } else {
        windows.emplace_back(trim_start, piece_end);
    }
}

// Mirrors map_elements.cpp's own IsBoundaryKind() (library-side, not
// reachable from this node-side translation unit) -- same two kinds, same
// meaning: an interior lane separator, never the road's own outer edge.
bool IsBoundaryKind(overlume::MapKind kind) {
    return kind == overlume::MapKind::LEFT_BOUNDARY || kind == overlume::MapKind::RIGHT_BOUNDARY;
}

// Even-odd point-in-polygon on the 2D (x,y) ring `poly` -- standard ray-cast
// parity of edge crossings to the right of `p`. `poly` need not repeat its
// first point as its last (the recorded JUNCTION markers do; a hand-built
// ring need not) -- the wraparound `j = poly.size() - 1` always closes it.
bool PointInPolygonEvenOdd(const overlume::Vec3& p, const std::vector<overlume::Vec3>& poly) {
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const overlume::Vec3& a = poly[i];
        const overlume::Vec3& b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool PointInAnyPolygon(const overlume::Vec3& p,
                       const std::vector<const std::vector<overlume::Vec3>*>& polys) {
    for (const auto* poly : polys) {
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
std::vector<std::vector<overlume::Vec3>> ClipAgainstJunctions(
    const std::vector<overlume::Vec3>& pts,
    const std::vector<const std::vector<overlume::Vec3>*>& polys) {
    std::vector<std::vector<overlume::Vec3>> out;
    if (polys.empty() || pts.size() < 2) {
        out.push_back(pts);
        return out;
    }

    auto lerp = [](const overlume::Vec3& a, const overlume::Vec3& b, double t) {
        return overlume::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    };

    std::vector<overlume::Vec3> current;
    bool a_in = PointInAnyPolygon(pts.front(), polys);
    if (!a_in) current.push_back(pts.front());
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const overlume::Vec3& a = pts[i];
        const overlume::Vec3& b = pts[i + 1];
        const bool b_in = PointInAnyPolygon(b, polys);
        if (a_in == b_in) {
            if (!b_in) current.push_back(b);
        } else {
            double lo = 0.0, hi = 1.0;  // lo matches a_in's side, hi matches b_in's
            for (int iter = 0; iter < 30; ++iter) {
                const double mid = 0.5 * (lo + hi);
                if (PointInAnyPolygon(lerp(a, b, mid), polys) == a_in)
                    lo = mid;
                else
                    hi = mid;
            }
            const overlume::Vec3 cross = lerp(a, b, 0.5 * (lo + hi));
            if (a_in && !b_in) {
                // Exiting the polygon: start a fresh kept (outside) chain.
                current.clear();
                current.push_back(cross);
                current.push_back(b);
            } else {
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
// near-antiparallel segment pairs (kMinCrossingSinAngle = sin(15 deg)): the
// cross-product denom is proportional to sin(angle between the two
// directions), and sin(180 deg - x) == sin(x), so a lanelet-chain node's
// fraction-of-a-degree kink (abutting at 175-180 deg or 0-3 deg, measured
// against hd_map_local_elements_0.yaml) is rejected the same way two
// open-road rails running alongside each other are. See plan
// 2026-08-18-visual-mode-epic3.md for the fixture classification.
constexpr double kMinCrossingSinAngle = 0.25881904510252074;  // sin(15 deg)

bool SegSegIntersect2D(const overlume::Vec3& p1, const overlume::Vec3& p2, const overlume::Vec3& p3,
                       const overlume::Vec3& p4, double& t, double& u) {
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
void MergeWindows(std::vector<std::pair<double, double>>& windows) {
    if (windows.empty()) return;
    std::sort(windows.begin(), windows.end());
    std::vector<std::pair<double, double>> merged;
    merged.push_back(windows.front());
    for (size_t i = 1; i < windows.size(); ++i) {
        if (windows[i].first <= merged.back().second + kJunctionGapMergeM) {
            merged.back().second = std::max(merged.back().second, windows[i].second);
        } else {
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
std::vector<std::vector<overlume::Vec3>> ApplyCutWindows(
    const std::vector<overlume::Vec3>& pts, const std::vector<double>& cum,
    const std::vector<std::pair<double, double>>& windows) {
    std::vector<std::vector<overlume::Vec3>> out;
    if (windows.empty()) {
        out.push_back(pts);
        return out;
    }
    const double total = cum.back();
    double pos = 0.0;
    auto append_kept_range = [&](double from, double to) {
        std::vector<overlume::Vec3> chain;
        chain.push_back(PointAtArcLength(pts, cum, from));
        for (size_t j = 0; j < pts.size(); ++j) {
            if (cum[j] > from && cum[j] < to) chain.push_back(pts[j]);
        }
        chain.push_back(PointAtArcLength(pts, cum, to));
        if (chain.size() >= 2) out.push_back(std::move(chain));
    };
    for (const auto& w : windows) {
        const double w_start = std::clamp(w.first, 0.0, total);
        const double w_end = std::clamp(w.second, 0.0, total);
        if (w_start > pos) append_kept_range(pos, w_start);
        pos = std::max(pos, w_end);
    }
    if (pos < total) append_kept_range(pos, total);
    return out;
}

}  // namespace

HdMapAdapter::HdMapAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf) {}

void HdMapAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec) {
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    // ONE lookup for the whole message (epic2 plan, "Frames") -- every
    // marker in a MarkerArray from one publisher shares one frame_id in
    // practice; looking it up per-marker would be hundreds of redundant
    // buffer walks for zero benefit.
    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform)) {
        ++stats_.dropped_no_tf;
        return;  // whole message dropped; previously-stored elements stay
    }

    // Stamps topic-liveness BEFORE the rate gate below, not after it: fill()
    // stamps every MapElement::last_update_sec from this, not from
    // last_rebuild_sec_/stats_.last_msg_sec (both set only past the gate) --
    // see hd_map.hpp's Staleness comment for why a throttled row must stay
    // opaque between accepted rebuilds as long as it keeps receiving.
    last_recv_sec_ = sim_time_sec;

    // Rate limit: gates REBUILDS, not receipt. First-ever call always
    // rebuilds (last_rebuild_sec_ starts at -1.0, "no prior rebuild").
    if (row_.max_rate_hz > 0.0 && last_rebuild_sec_ >= 0.0) {
        const double min_gap_sec = 1.0 / row_.max_rate_hz;
        if (sim_time_sec - last_rebuild_sec_ < min_gap_sec) return;
    }

    for (const auto& m : msg.markers) {
        if (m.action == kActionDeleteAll) {
            // ROS Marker semantics: DELETEALL clears every marker this
            // adapter is tracking, regardless of ITS OWN ns/id fields.
            storage_.clear();
            continue;
        }
        if (m.action == kActionDelete) {
            storage_.erase(Key{m.ns, m.id});
            continue;
        }
        if (m.action != kActionAdd && m.action != kActionModify) continue;

        const NsRule* rule = match_rule(row_, m.ns);
        const NsRender verdict = rule != nullptr ? rule->render : row_.ns_default;
        if (verdict == NsRender::kDrop) {
            // Intentional, not malformed -- see epic2 plan, "Diagnostics
            // counters": centerline_arrows_ alone is ~93% of map volume,
            // and folding this into dropped_malformed would make a
            // healthy system look broken.
            ++stats_.dropped_by_rule;
            continue;
        }

        if (m.type != kMarkerTypeLineStrip || m.points.size() < 2) {
            ++stats_.dropped_malformed;
            continue;
        }

        // effective_point = frame_transform * (marker_pose * point) -- the
        // marker pose lives IN the header frame, so it composes INSIDE the
        // frame transform, not outside (rviz-parity gap fix). Built once
        // per marker, before dash chopping or anything else touches points.
        const bool identity_pose = MarkerPoseIsIdentity(m.pose);
        tf2::Transform marker_tf;
        if (!identity_pose) {
            if (MarkerPoseHasNan(m.pose)) {
                ++stats_.dropped_malformed;  // NaN marker pose, not just a NaN point
                continue;
            }
            tf2::Quaternion q(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                              m.pose.orientation.w);
            // Zero/degenerate quaternion -> identity, matching rviz; tf2
            // would otherwise NaN every point and silently void the whole
            // marker.
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
            marker_tf = tf2::Transform(
                q, tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
        }

        std::vector<overlume::Vec3> pts;
        pts.reserve(m.points.size());
        bool ok = true;
        for (const auto& p : m.points) {
            const tf2::Vector3 local(p.x, p.y, p.z);
            const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
            const tf2::Vector3 tp = xform * posed;
            // flatten_z: 2D HD-map plane -- see frame_transform.hpp.
            const overlume::Vec3 v{tp.x(), tp.y(), tf_.flatten_z() ? 0.0 : tp.z()};
            if (HasNan(v)) {
                ok = false;
                break;
            }
            pts.push_back(v);
        }
        if (!ok) {
            // A NaN anywhere in the polyline drops the WHOLE primitive
            // (spec §9) -- never a partially-built vertex buffer.
            ++stats_.dropped_malformed;
            continue;
        }

        const uint8_t is_polygon = (verdict == NsRender::kPolygon) ? 1 : 0;

        // Crosswalk-hatch fix: a closed polygon marker arrives with a
        // duplicate closing vertex (point[0] == point[n-1], e.g. the real
        // recorded fixture's crosswalk_8043) -- mirrors collision.cpp's own
        // trailing-duplicate dedupe (kDedupEpsM = 1e-6). Without this,
        // build_crosswalk_hatch()'s n==4 guard never fires on real data
        // (every recorded crosswalk arrives with 5 points).
        if (is_polygon && pts.size() >= 2) {
            constexpr double kDedupEpsM = 1e-6;
            const auto& front = pts.front();
            const auto& back = pts.back();
            const double dx = back.x - front.x, dy = back.y - front.y, dz = back.z - front.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < kDedupEpsM) pts.pop_back();
        }

        // kind/lane_id extraction: `kind` is the matched NsRule's own field
        // (no match -> OTHER, same "no rule -> default" shape ns_default
        // already has for render verdicts); `lane_id` is the marker's own
        // `id` for the lane-paired kinds, 0 otherwise.
        const overlume::MapKind kind = rule != nullptr ? rule->kind : overlume::MapKind::OTHER;
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

void HdMapAdapter::fill(overlume::ros::SceneAssembly& out) const {
    road_surface_points_.clear();
    junction_cut_points_.clear();
    std::unordered_map<uint32_t, const std::vector<overlume::Vec3>*> left_by_lane, right_by_lane;
    // Every JUNCTION-kind ring this message carries (empty on urban's local/
    // global rows -- verified, they have no `junction` namespace rule at
    // all). Pointers alias storage_, same lifetime contract as everything
    // else fill() reads from it.
    std::vector<const std::vector<overlume::Vec3>*> junction_polys;

    // First pass: index every boundary rail by lane_id, and collect every
    // JUNCTION ring. Needed BEFORE any element is emitted -- road-edge
    // detection (below) must see every OTHER lane's opposite-side boundary,
    // and road-surface pairing (further below) needs the same index. One
    // pass over storage_ builds both; a second walk (below) does the actual
    // emitting.
    for (const auto& [key, pieces] : storage_) {
        (void)key;
        for (const auto& elem : pieces) {
            if (elem.kind == overlume::MapKind::JUNCTION) junction_polys.push_back(&elem.points);
            if (elem.lane_id == 0) continue;
            if (elem.kind == overlume::MapKind::LEFT_BOUNDARY) {
                left_by_lane[elem.lane_id] = &elem.points;
            } else if (elem.kind == overlume::MapKind::RIGHT_BOUNDARY) {
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
    // Junction cleanup: a promoted ROAD_EDGE element is NOT pushed straight
    // to `out` here -- it is clipped against `junction_polys` (a no-op when
    // there are none) and queued in `road_edge_pieces` so every promoted
    // edge, from every marker, can be checked against every OTHER one for
    // the mutual-crossing cut below. LEFT_/RIGHT_BOUNDARY gets the SAME
    // polygon clip, pushed straight to `out` (never queued -- boundaries
    // never get the crossing cut), only when
    // `row_.junction_interior_boundaries` is false; the default (true)
    // leaves boundaries alone entirely.
    std::vector<PendingRoadEdge> road_edge_pieces;

    for (const auto& [key, pieces] : storage_) {
        (void)key;
        for (const auto& elem : pieces) {
            overlume::MapElement e{};
            e.is_polygon = elem.is_polygon;
            e.kind = elem.kind;
            e.lane_id = elem.lane_id;
            // Stamped from last_recv_sec_ (topic liveness), not
            // stats_.last_msg_sec, offset into the last kMapFadeWindowSec of
            // this row's own timeout_sec (see kMapFadeWindowSec's own
            // comment above and hd_map.hpp).
            e.last_update_sec = last_recv_sec_ + (row_.timeout_sec - kMapFadeWindowSec);
            if (e.kind == overlume::MapKind::LEFT_BOUNDARY &&
                IsRoadEdge(elem.lane_id, elem.points, right_by_lane)) {
                e.kind = overlume::MapKind::ROAD_EDGE;
            } else if (e.kind == overlume::MapKind::RIGHT_BOUNDARY &&
                       IsRoadEdge(elem.lane_id, elem.points, left_by_lane)) {
                e.kind = overlume::MapKind::ROAD_EDGE;
            }

            if (e.kind == overlume::MapKind::ROAD_EDGE) {
                for (auto& piece : ClipAgainstJunctions(elem.points, junction_polys)) {
                    road_edge_pieces.push_back(PendingRoadEdge{std::move(piece), elem.lane_id,
                                                               e.last_update_sec, e.is_polygon});
                }
                continue;
            }
            if (IsBoundaryKind(e.kind) && !row_.junction_interior_boundaries &&
                !junction_polys.empty()) {
                for (auto& piece : ClipAgainstJunctions(elem.points, junction_polys)) {
                    junction_cut_points_.push_back(std::move(piece));
                    overlume::MapElement be = e;  // same is_polygon/kind/lane_id/last_update_sec
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

    // Mutual-crossing cut: any two ROAD_EDGE pieces from DIFFERENT lane_ids
    // that cross in 2D each get a kJunctionCutBackoffM window removed
    // around their own crossing arc-length station -- same-lane pairs are
    // skipped (two pieces of the SAME original polyline, split apart by the
    // polygon clip above, are never meant to cut each other). Every
    // crossing a piece is party to adds one raw window; MergeWindows folds
    // overlapping ones from several crossings on the same piece before
    // ApplyCutWindows runs.
    std::vector<std::vector<double>> cum(road_edge_pieces.size());
    for (size_t i = 0; i < road_edge_pieces.size(); ++i) {
        cum[i] = CumulativeArcLength(road_edge_pieces[i].points);
    }
    std::vector<std::vector<std::pair<double, double>>> windows(road_edge_pieces.size());
    for (size_t i = 0; i < road_edge_pieces.size(); ++i) {
        const auto& pi = road_edge_pieces[i].points;
        for (size_t j = i + 1; j < road_edge_pieces.size(); ++j) {
            if (road_edge_pieces[i].lane_id == road_edge_pieces[j].lane_id) continue;
            const auto& pj = road_edge_pieces[j].points;
            for (size_t a = 0; a + 1 < pi.size(); ++a) {
                for (size_t b = 0; b + 1 < pj.size(); ++b) {
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
    for (size_t i = 0; i < road_edge_pieces.size(); ++i) {
        MergeWindows(windows[i]);
        // Arc-aware refinement (see that constant block's own comment
        // above): snaps each merged window's own boundaries outward to a
        // real corner arc found nearby on THIS edge's own polyline. Runs
        // after MergeWindows (so it only ever refines an already-decided
        // set of cut regions, never changes which crossings get cut) and
        // before ApplyCutWindows (so the extended boundaries are what
        // actually gets removed). A snap can land farther than
        // kJunctionGapMergeM=6.6 m from its own original boundary (see
        // FindArcSpanNear's own comment), so re-running MergeWindows below
        // (it sorts + merges) is what restores the sorted/non-overlapping
        // invariant ApplyCutWindows relies on.
        SnapWindowsToArcs(road_edge_pieces[i].points, cum[i], windows[i]);
        MergeWindows(windows[i]);
        // Straight-neighbour overshoot snap (see
        // SnapWindowsToNeighborArcDepartures's own top-of-block comment):
        // the symmetric counterpart of SnapWindowsToArcs just above -- that
        // one snaps THIS piece's own window outward to an arc it OWNS; this
        // one snaps it back to a NEIGHBOUR piece's own arc-departure vertex
        // when this piece is the straight edge running past it. Runs after
        // this loop's own MergeWindows (only ever tightens an
        // already-decided cut region) and only ever touches the two
        // outermost boundaries in place -- see that function's own comment
        // for why no further MergeWindows call is needed.
        SnapWindowsToNeighborArcDepartures(road_edge_pieces, i, cum, windows[i]);
        // Redundant arc-tail trim (see TrimRedundantArcTails's own
        // top-of-block comment): appends one more trim window on THIS
        // piece's own tail past its last corner arc's rejoin vertex, when
        // that tail duplicates another independently-promoted ROAD_EDGE
        // piece. Runs after this loop's own MergeWindows (so it only ever
        // adds to an already-decided set of cut regions) and merges
        // directly into `windows[i]` in place (never via another
        // MergeWindows call -- see that function's own comment for why), so
        // ApplyCutWindows below sees the trim already folded in.
        TrimRedundantArcTails(road_edge_pieces, i, cum, windows[i]);
        for (auto& piece : ApplyCutWindows(road_edge_pieces[i].points, cum[i], windows[i])) {
            junction_cut_points_.push_back(std::move(piece));
            overlume::MapElement e{};
            e.points = junction_cut_points_.back().data();
            e.point_count = static_cast<uint32_t>(junction_cut_points_.back().size());
            e.is_polygon = road_edge_pieces[i].is_polygon;
            e.kind = overlume::MapKind::ROAD_EDGE;
            e.lane_id = road_edge_pieces[i].lane_id;
            e.last_update_sec = road_edge_pieces[i].last_update_sec;
            out.map_elements.push_back(e);
        }
    }

    // Road-surface fill: pair every lane_id present on BOTH rails; a
    // lane_id on only one rail (0 of 16 in the committed fixture, but not
    // provably impossible on other bags) emits nothing for it -- silently
    // dropped, not malformed (spec §9's "missing data renders nothing").
    // road_surface_points_ holds the resampled buffers these
    // synthesized elements point into; cleared and rebuilt at the top of
    // every fill() call, so it stays alive exactly as long as this fill()
    // call's own out.map_elements does.
    for (const auto& [lane_id, left_pts] : left_by_lane) {
        const auto it = right_by_lane.find(lane_id);
        if (it == right_by_lane.end()) continue;

        const std::vector<overlume::Vec3> left_r = ResampleByArcLength(*left_pts, kRoadFillSamples);
        const std::vector<overlume::Vec3> right_r =
            ResampleByArcLength(*it->second, kRoadFillSamples);
        if (left_r.empty() || right_r.empty()) continue;  // malformed rail, skip silently

        road_surface_points_.emplace_back();
        std::vector<overlume::Vec3>& combined = road_surface_points_.back();
        combined.reserve(2 * kRoadFillSamples);
        combined.insert(combined.end(), left_r.begin(), left_r.end());
        combined.insert(combined.end(), right_r.begin(), right_r.end());

        overlume::MapElement e{};
        e.points = combined.data();
        e.point_count = static_cast<uint32_t>(combined.size());
        e.is_polygon = 0;
        e.kind = overlume::MapKind::ROAD_SURFACE;
        e.lane_id = lane_id;
        // Same offset stamp as every other emitted element above -- the
        // synthesized ROAD_SURFACE element must fade (and
        // ramp-out-before-cutoff) too.
        e.last_update_sec = last_recv_sec_ + (row_.timeout_sec - kMapFadeWindowSec);
        out.map_elements.push_back(e);
    }
}

}  // namespace overlume_node
