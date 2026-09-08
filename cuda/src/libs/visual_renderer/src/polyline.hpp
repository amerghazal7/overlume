// polyline.hpp — Epic 2 Task 2 (VM-024): the shared CPU polyline/polygon
// geometry helper Tasks 4 (predicted paths), 5 (ribbons) and 7 (alert
// polygons) all reuse instead of each writing their own extruder (see the
// plan's "Library: internal seams" — a reviewer finding a second extrusion
// implementation in this epic is a blocking duplication finding).
//
// Deliberately Filament-free: test targets get `-I src` and `-I ${STB_DIR}`
// only, NOT visual_renderer's PRIVATE Filament include dir
// (renderer_internal.hpp's own comment, CMakeLists.txt:246-252) — a header
// here that transitively names `Vertex` (filament::math::float3/float4)
// would fail tests/test_polyline.cpp at the first #include, before a single
// assertion runs. Everything below returns/accepts only `mpviz::Vec3`
// (scene.h, itself Filament-free) or plain integer types. The Vec3->Vertex
// conversion (positions + the flat +Z tangent frame) happens at the
// Filament call site in map_elements.cpp (and, in later tasks, ribbon.cpp /
// objects.cpp / alert_polygons.cpp).
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
// pair i straddles the i-th SURVIVING point (see cleaning rules below) —
// left/right meaning "rotate the local direction +90d/-90d about +Z".
//
// Input cleaning (spec §9 — never propagate a NaN vertex, never divide by a
// zero-length segment direction):
//  - nullptr or n<2 -> empty.
//  - a NaN in any component of pts[i] TRUNCATES the polyline at i (points
//    before it are still extruded; points at/after it are dropped) rather
//    than propagating the NaN into a vertex buffer.
//  - a point identical to the immediately preceding SURVIVING point (a
//    zero-length segment) is dropped, not kept as a degenerate duplicate.
//  - if fewer than 2 points survive cleaning, returns empty.
//
// Interior joins are mitred (average of the two adjacent segments' normals,
// scaled by 1/cos(half the turn angle)) — exact for the common shallow-
// curve case (lane centerlines), and the mitre length is clamped to
// `half_width * kMaxMiterRatio` so a near-reversal corner can't shoot a
// vertex arbitrarily far out (the classic "bowtie" extrusion bug becomes a
// bounded pinch instead of an unbounded spike; see the .cpp for the clamp
// value).
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

// Lazy crosswalk hatch (Epic 2 Task 2 / VM-024; Epic 3 Task 1 / VM-036
// decision #2 fixes its n==4 guard to actually fire on real recorded
// geometry). Painted bars with ground visible in the gaps between them --
// the visual differentiation IS the geometry, not a second material.
// Bilinear-interpolated stripes between the quad's two SHORT edges (a
// general convex-polygon clip was skipped as unneeded complexity -- see
// map_elements.cpp's own ponytail note). Returns empty for n != 4; the
// caller falls back to triangulate_convex_polygon()'s plain fan fill.
// Declared here (not map_elements.hpp, which pulls in Filament) so it's
// reachable from Filament-free tests, same reasoning as every function
// above it in this header.
std::vector<Vec3> build_crosswalk_hatch(const Vec3* pts, uint32_t n, float z_lift);

}  // namespace mpviz::detail
