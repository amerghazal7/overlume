// polyline.hpp — shared CPU polyline/polygon geometry helper: predicted
// paths, ribbons, and alert polygons all reuse this instead of each
// writing their own extruder.
//
// Deliberately Filament-free: test targets get `-I src` and
// `-I ${STB_DIR}` only, not visual_renderer's PRIVATE Filament include dir
// (see renderer_internal.hpp) — a header here that transitively names
// `Vertex` (filament::math::float3/float4) would fail
// tests/test_polyline.cpp at the first #include. Everything below
// returns/accepts only `mpviz::Vec3` (scene.h, itself Filament-free) or
// plain integer types. The Vec3->Vertex conversion happens at the
// Filament call site (map_elements.cpp/ribbon.cpp/objects.cpp/
// alert_polygons.cpp).
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "visual_renderer/scene.h"

namespace mpviz::detail {

// Mitre-joined triangle-strip extrusion of a polyline into a flat ribbon of
// `half_width`, each vertex lifted `z_lift` above its source point's Z
// (avoids z-fighting with the ground plane it's painted onto). Returns
// 2*m positions, interleaved [left_0, right_0, left_1, right_1, ...] where
// pair i straddles the i-th surviving point (see cleaning rules below) —
// left/right meaning "rotate the local direction +90d/-90d about +Z".
//
// Input cleaning (never propagate a NaN vertex, never divide by a
// zero-length segment direction):
//  - nullptr or n<2 -> empty.
//  - a NaN in any component of pts[i] truncates the polyline at i (points
//    before it are still extruded; points at/after it are dropped) rather
//    than propagating the NaN into a vertex buffer.
//  - a point identical to the immediately preceding surviving point (a
//    zero-length segment) is dropped, not kept as a degenerate duplicate.
//  - if fewer than 2 points survive cleaning, returns empty.
//
// Interior joins are mitred (average of the two adjacent segments' normals,
// scaled by 1/cos(half the turn angle)) — exact for the common shallow-
// curve case (lane centerlines), and the mitre length is clamped to
// `half_width * kMaxMiterRatio` so a near-reversal corner can't shoot a
// vertex arbitrarily far out (see the .cpp for the clamp value).
std::vector<Vec3> extrude_polyline(const Vec3* pts, uint32_t n, float half_width, float z_lift);

// Triangle-strip index list for the 2*point_count vertices a `point_count`-
// point call to extrude_polyline() produced (point_count is the SURVIVING
// count, i.e. the length of whatever extrude_polyline actually returned,
// halved). point_count < 2 -> empty; otherwise 6*(point_count-1) indices,
// two triangles per segment.
std::vector<uint16_t> extrude_polyline_indices(uint32_t point_count);

// Fan-triangulates a convex polygon about its first vertex, each output
// vertex lifted `z_lift` above its source point's Z. Returns a flat
// triangle LIST (3*(n-2) positions — no separate index buffer; a fresh
// small vertex buffer per polygon with sequential 0..count-1 indices is
// already the simplest correct thing for the handful of points a crosswalk
// polygon has). n<3, nullptr, or any NaN point among the first n -> empty
// (a malformed convex polygon has no safe partial-fan reading, unlike a
// polyline's "truncate and keep the prefix" rule).
std::vector<Vec3> triangulate_convex_polygon(const Vec3* pts, uint32_t n, float z_lift);

// Filament-free "where to cut" helper for the uint16_t index ceiling: index
// buffers are uint16_t (a hard 65535-vertex ceiling PER MESH), and each
// polyline point emits 2 vertices via extrude_polyline -> kMaxPointsPerMesh
// points/mesh keeps every mesh comfortably under that ceiling. Returns
// half-open [start, end) ranges covering [0, n), each <= kMaxPointsPerMesh,
// consecutive ranges OVERLAPPING BY ONE POINT so the strips extruded from
// each chunk join with no gap and no duplicated segment. n<2 -> empty (same
// "nothing to extrude" floor as extrude_polyline itself).
inline constexpr uint32_t kMaxPointsPerMesh = 32000;  // 2 verts/pt, < 65535/2
std::vector<std::pair<uint32_t, uint32_t>> polyline_chunks(uint32_t n);

// Ego-proximity clip: promoted out of ribbon.cpp (VM-077 carpet-as-ribbon
// redirect) so a second caller (the velocity ribbon) shares the identical
// "never render behind the ego" mechanism instead of a second arc-length
// walk. Values unchanged from ribbon.cpp's originals, renamed only for
// their shared home.

// Closest-approach arc-station to `ego` (2D, map frame); returns {station,
// min lateral distance}. n<2 -> {0, +inf} (fails the proximity gate below).
std::pair<double, double> closest_arc_station(const Vec3* pts, uint32_t n, const Vec3& ego);

// Proximity gate: only clip a polyline the ego is actually near (a far-away
// route must render whole). Generous vs. the ~0.1-0.5m half-widths ribbons
// draw at -- "is the ego riding this route", not a precise offset.
inline constexpr float kPolylineEgoClipLateralM = 5.0f;
// Clip station granularity, so a parked ego causes zero signature changes.
inline constexpr float kPolylineClipQuantizeM = 0.5f;

struct PolylineClip {
    bool active = false;
    int64_t quantized_units = 0;  // station / kPolylineClipQuantizeM, rounded -- meaningful iff active
    double station_m = 0.0;       // quantized_units * kPolylineClipQuantizeM -- meaningful iff active
};

// Clip decision for a polyline the ego may be riding. Caller gates on ego
// validity first -- this function has no notion of EgoState::valid.
PolylineClip compute_polyline_clip(const Vec3* pts, uint32_t n, const Vec3& ego_position);

// Truncates `pts`/`n` to the forward half starting at arc-length `s0`, with
// an interpolated cut point (not a snap to the nearest vertex). n<2 -> empty.
std::vector<Vec3> clip_polyline_forward(const Vec3* pts, uint32_t n, double s0);

// Lazy crosswalk hatch: painted bars with ground visible in the gaps
// between them -- the visual differentiation is the geometry, not a
// second material. Bilinear-interpolated stripes between the quad's two
// long edges, so each bar's long axis runs along the quad's short
// (travel) axis and bars stack across the crossing width; stripe count is
// pitch-derived, not fixed (see map_elements.cpp's own ponytail note on
// why a general convex-polygon clip was skipped). Returns empty for
// n != 4; the caller falls back to triangulate_convex_polygon()'s plain
// fan fill.
// Declared here (not map_elements.hpp, which pulls in Filament) so it's
// reachable from Filament-free tests.
std::vector<Vec3> build_crosswalk_hatch(const Vec3* pts, uint32_t n, float z_lift);

}  // namespace mpviz::detail
