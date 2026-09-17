// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// trajectory_carpet_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_trajectory_carpet.cpp links only against `overlume` and
// has no access to its PRIVATE Filament include dir, so it can't include
// trajectory_carpet.hpp/renderer_internal.hpp directly. Declared only
// against api.h's opaque VisualRenderer — defined in trajectory_carpet.cpp,
// where the state they read actually lives. Mirrors point_cloud_test_hooks.hpp.
#pragma once

#include <cstddef>
#include <cstdint>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// Number of Filament meshes backing trajectory-carpet slot `slot` (0-indexed
// into active().trajectory_carpets) — Filament-free: a size_t, not a
// Mesh/Entity. >1 only when the slot's point_count exceeded
// polyline_chunks()'s kMaxPointsPerMesh chunk ceiling. 0 if `r` is null or
// `slot` is out of range.
size_t trajectory_carpet_mesh_count(overlume::VisualRenderer* r, size_t slot);

// Total vertex count summed across every mesh backing slot `slot`. Chunk
// ranges are triangle-aligned and non-overlapping (trajectory_carpet.cpp's
// triangle_chunks(), NOT polyline.hpp's polyline_chunks() — a triangle
// list has no shared join vertex between chunks the way a line strip
// does), so this must equal the source point_count exactly: no vertex
// dropped, none double-counted at a chunk seam. 0 if `r` is null or `slot`
// is out of range.
size_t trajectory_carpet_vertex_count(overlume::VisualRenderer* r, size_t slot);

// The alpha value last passed to the ONE shared trajectory_carpet.mat
// instance's "alpha" parameter (mirrored CPU copy — MaterialInstance has no
// getter). 1.0f if `r` is null.
float trajectory_carpet_material_alpha(overlume::VisualRenderer* r);

// The packed rgba of vertex `vertex_idx` within slot `slot`'s FIRST built
// mesh, as it was actually written into the vertex buffer (post
// resolve_rgba() substitution) — Filament-free readback for the
// alpha-zero-sentinel test. 0 if `r`/`slot`/`vertex_idx` is out of range.
uint32_t trajectory_carpet_vertex_rgba(overlume::VisualRenderer* r, size_t slot, size_t vertex_idx);

// The actual world-space z (INCLUDING the renderer's own z-stack lift --
// see trajectory_carpet.cpp's velocity-ribbon z-lift) vertex `vertex_idx`
// within slot `slot`'s first built mesh was given. 0.0f if `r`/`slot`/
// `vertex_idx` is out of range.
float trajectory_carpet_vertex_z(overlume::VisualRenderer* r, size_t slot, size_t vertex_idx);

// The actual half-width (theme ribbon.margin_velocity_m, clamped)
// build_slot_meshes() used for slot `slot`'s geometry the last time it
// rebuilt -- not a Filament AABB query, same "mirror what was really built"
// reasoning as ribbon_test_hooks.hpp's ribbon_slot_half_width_m(). 0.0f if
// `r` is null or `slot` is out of range.
float trajectory_carpet_half_width_m(overlume::VisualRenderer* r, size_t slot);

// Total number of times ANY trajectory-carpet slot has rebuilt its geometry
// (content, half-width, or ego-clip station changed) since create_renderer()
// -- the H2 flicker-fix regression pin: a publish that changes only
// per-vertex color, with identical station positions, must NOT bump this.
// 0 if `r` is null.
uint64_t trajectory_carpet_rebuild_count(overlume::VisualRenderer* r);

// The current effective first vertex (left rail) slot `slot` is actually
// showing -- post ego-clip collapse, if any. Written to `*out`; returns
// false (out unchanged) if `r` is null, `slot` is out of range, or the
// slot has never built geometry. Mirrors ribbon_test_hooks.hpp's
// ribbon_slot_first_point().
bool trajectory_carpet_slot_first_point(overlume::VisualRenderer* r, size_t slot,
                                        overlume::Vec3* out);

}  // namespace overlume::testing
