// point_cloud_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_point_cloud.cpp links only against `visual_renderer` and has
// no access to its PRIVATE Filament include dir, so it can't include
// point_cloud.hpp/renderer_internal.hpp directly. Declared only against
// api.h's opaque VisualRenderer — defined in point_cloud.cpp, where the
// state they read actually lives.
#pragma once

#include <cstddef>

#include "visual_renderer/api.h"

namespace mpviz::testing {

// Number of Filament meshes backing point-cloud slot `slot` (0-indexed
// into active().point_clouds) — Filament-free: a size_t, not a
// Mesh/Entity. >1 only when the slot's point_count exceeded
// polyline_chunks()'s kMaxPointsPerMesh chunk ceiling (polyline.hpp,
// reused unmodified — POINTS is 1 vertex/point, no 2x). 0 if `r` is null
// or `slot` is out of range.
size_t point_cloud_mesh_count(mpviz::VisualRenderer* r, size_t slot);

// Total vertex count summed across every mesh backing slot `slot` — proves
// no point is lost at the chunk-split seam. 0 if `r` is null or `slot` is
// out of range.
size_t point_cloud_vertex_count(mpviz::VisualRenderer* r, size_t slot);

// The alpha value last passed to the ONE shared point_cloud.mat instance's
// "alpha" parameter (mirrored CPU copy — MaterialInstance has no getter).
// 1.0f if `r` is null.
float point_cloud_material_alpha(mpviz::VisualRenderer* r);

}  // namespace mpviz::testing
