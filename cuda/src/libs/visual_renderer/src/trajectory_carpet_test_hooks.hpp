// trajectory_carpet_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_trajectory_carpet.cpp links only against `visual_renderer` and
// has no access to its PRIVATE Filament include dir, so it can't include
// trajectory_carpet.hpp/renderer_internal.hpp directly. Declared only
// against api.h's opaque VisualRenderer — defined in trajectory_carpet.cpp,
// where the state they read actually lives. Mirrors point_cloud_test_hooks.hpp.
#pragma once

#include <cstddef>
#include <cstdint>

#include "visual_renderer/api.h"

namespace mpviz::testing {

// Number of Filament meshes backing trajectory-carpet slot `slot` (0-indexed
// into active().trajectory_carpets) — Filament-free: a size_t, not a
// Mesh/Entity. >1 only when the slot's point_count exceeded
// polyline_chunks()'s kMaxPointsPerMesh chunk ceiling. 0 if `r` is null or
// `slot` is out of range.
size_t trajectory_carpet_mesh_count(mpviz::VisualRenderer* r, size_t slot);

// Total vertex count summed across every mesh backing slot `slot`. Chunk
// ranges are triangle-aligned and non-overlapping (trajectory_carpet.cpp's
// triangle_chunks(), NOT polyline.hpp's polyline_chunks() — a triangle
// list has no shared join vertex between chunks the way a line strip
// does), so this must equal the source point_count exactly: no vertex
// dropped, none double-counted at a chunk seam. 0 if `r` is null or `slot`
// is out of range.
size_t trajectory_carpet_vertex_count(mpviz::VisualRenderer* r, size_t slot);

// The alpha value last passed to the ONE shared trajectory_carpet.mat
// instance's "alpha" parameter (mirrored CPU copy — MaterialInstance has no
// getter). 1.0f if `r` is null.
float trajectory_carpet_material_alpha(mpviz::VisualRenderer* r);

// The packed rgba of vertex `vertex_idx` within slot `slot`'s FIRST built
// mesh, as it was actually written into the vertex buffer (post
// resolve_rgba() substitution) — Filament-free readback for the
// alpha-zero-sentinel test. 0 if `r`/`slot`/`vertex_idx` is out of range.
uint32_t trajectory_carpet_vertex_rgba(mpviz::VisualRenderer* r, size_t slot,
                                        size_t vertex_idx);

// The actual world-space z (INCLUDING the renderer's own z-stack lift --
// see trajectory_carpet.cpp's kTrajectoryCarpetZLiftM) vertex `vertex_idx`
// within slot `slot`'s first built mesh was given. 0.0f if `r`/`slot`/
// `vertex_idx` is out of range.
float trajectory_carpet_vertex_z(mpviz::VisualRenderer* r, size_t slot, size_t vertex_idx);

}  // namespace mpviz::testing
