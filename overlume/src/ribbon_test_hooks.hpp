// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// ribbon_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_ribbon.cpp links only against `overlume` and has no
// access to its PRIVATE Filament include dir, so it can't include
// ribbon.hpp/renderer_internal.hpp directly. Declared against api.h's
// opaque VisualRenderer, scene.h's POD PathRole, and theme.hpp's
// Filament-free Float3 — defined in ribbon.cpp, where the state actually
// lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// The color last passed to role `role`'s MaterialInstance "baseColor"
// param (rgb only — Filament has no getter for a set parameter, so this
// isn't a GPU read-back). {0,0,0} if `r` is null.
overlume::detail::Float3 ribbon_role_base_color(overlume::VisualRenderer* r,
                                                overlume::PathRole role);

// Number of Filament meshes backing ribbon slot `slot` (0-indexed into
// active().paths) — Filament-free: a size_t, not a Mesh/Entity. >1 only
// when the slot's point_count exceeded polyline_chunks()'s
// uint16-index-buffer ceiling (kMaxPointsPerMesh, polyline.hpp). 0 if `r`
// is null or `slot` is out of range.
size_t ribbon_mesh_count(overlume::VisualRenderer* r, size_t slot);

// Total vertex count summed across every mesh backing slot `slot` —
// proves no point is lost at the chunk-split seam
// (LongPathSplitsAcrossMeshesWithoutTruncation compares this against
// polyline_chunks()'s own math, computed independently). 0 if `r` is null
// or `slot` is out of range.
size_t ribbon_vertex_count(overlume::VisualRenderer* r, size_t slot);

// Which material `slot`'s renderable is currently bound to, read via
// RenderableManager::getMaterialInstanceAt() (a real GPU-state getter,
// unlike MaterialInstance's write-only params). `bound_to_translucent` is
// only meaningful for GLOBAL/LOCAL (BEHAVIOR never swaps instances — it
// fades by setting ribbon_emissive.mat's own alpha, see ribbon.cpp).
// `alpha` mirrors the CPU-stored fade alpha: for BEHAVIOR this is what was
// last pushed to the shared ribbonMaterial[BEHAVIOR] baseColor.a; for
// GLOBAL/LOCAL it is only meaningful while bound_to_translucent.
struct RibbonMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
RibbonMaterialInfo ribbon_slot_material_info(overlume::VisualRenderer* r, size_t slot);

// The half-width (build_effective_half_width(), ribbon.cpp — derived from
// theme.ribbon.lane_width_m and the role's own margin) slot `slot`'s
// geometry was actually built with the last time it rebuilt — not a
// Filament AABB query (add_mesh() gives every mesh the same hard-coded
// declared culling box, unrelated to the strip's real extent; same
// reasoning as ego.cpp's egoFallbackDims). WidthChangeRebuildsGeometry
// reads this to prove a width-only set_theme() actually rebuilt the strip,
// since vertex count alone can't tell (it's 2*point_count regardless of
// width). 0.0f if `r` is null or `slot` is out of range.
float ribbon_slot_half_width_m(overlume::VisualRenderer* r, size_t slot);

// The first geometry point slot `slot`'s build_slot_meshes() call actually
// used (ego-proximity ribbon clip) -- written to `*out`, returns false
// (out unchanged) if `r` is null, `slot` is out of range, or the slot has
// never built geometry. Proves a clipped ribbon's mesh starts at the
// interpolated clip point, not the original PathRibbon::points[0] — see
// RibbonSlot::firstPointM (renderer_internal.hpp).
bool ribbon_slot_first_point(overlume::VisualRenderer* r, size_t slot, overlume::Vec3* out);

// Total number of ribbon slot rebuilds (content, role, or ego-clip station
// changed) since `r` was created -- same cache-miss-counter convention as
// map_element_rebuild_count(). 0 if `r` is null.
uint64_t ribbon_rebuild_count(overlume::VisualRenderer* r);

}  // namespace overlume::testing
