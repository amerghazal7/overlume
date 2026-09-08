// ribbon_test_hooks.hpp — internal-only, `-I src` visibility, not installed,
// not POD (Epic 2 Task 5 / VM-023). Same reasoning as
// map_elements_test_hooks.hpp/objects_test_hooks.hpp: tests/test_ribbon.cpp
// links only against `visual_renderer` and is never granted that target's
// PRIVATE Filament include dir, so it can't include ribbon.hpp/
// renderer_internal.hpp directly. These hooks are declared against nothing
// but api.h's opaque mpviz::VisualRenderer, scene.h's POD PathRole, and
// theme.hpp's Filament-free mpviz::detail::Float3 — defined in ribbon.cpp,
// where the state they read actually lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// The color value last actually passed to role `role`'s MaterialInstance's
// "baseColor" setParameter call (rgb only — NOT a read-back of live GPU
// state, Filament's MaterialInstance exposes no getter for a set
// parameter), mirroring lane_material_base_color()'s/object_class_tint()'s
// own pattern. {0,0,0} if `r` is null.
mpviz::detail::Float3 ribbon_role_base_color(mpviz::VisualRenderer* r, mpviz::PathRole role);

// Number of Filament meshes backing ribbon SLOT `slot` (0-indexed into
// active().paths — the plan's "keyed by slot index" decision, Task 5 Step
// 4) — Filament-free: a size_t, not a Mesh/Entity. >1 only when the slot's
// point_count exceeded polyline_chunks()'s uint16-index-buffer ceiling
// (kMaxPointsPerMesh, polyline.hpp). 0 if `r` is null or `slot` is out of
// range.
size_t ribbon_mesh_count(mpviz::VisualRenderer* r, size_t slot);

// Total vertex count summed across every mesh backing slot `slot` — proves
// no point is lost at the chunk-split seam (Ribbon.
// LongPathSplitsAcrossMeshesWithoutTruncation compares this against
// mpviz::detail::polyline_chunks()'s own math, computed independently in
// the test). 0 if `r` is null or `slot` is out of range.
size_t ribbon_vertex_count(mpviz::VisualRenderer* r, size_t slot);

// Which material slot `slot`'s renderable is CURRENTLY bound to, read via
// RenderableManager::getMaterialInstanceAt() (a real GPU-state getter,
// unlike MaterialInstance's own parameters, which are write-only) — same
// shape as objects_test_hooks.hpp's ObjectMaterialInfo. `bound_to_
// translucent` is only meaningful for GLOBAL/LOCAL (BEHAVIOR never swaps
// instances — it fades by setting ribbon_emissive.mat's own alpha, see
// ribbon.cpp). `alpha` mirrors the CPU-stored fade alpha (MaterialInstance
// has no parameter getter): for BEHAVIOR this is what was last pushed to
// the shared ribbonMaterial[BEHAVIOR] baseColor.a; for GLOBAL/LOCAL it is
// only meaningful while bound_to_translucent.
struct RibbonMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
RibbonMaterialInfo ribbon_slot_material_info(mpviz::VisualRenderer* r, size_t slot);

// The half-width (build_effective_half_width(), ribbon.cpp: derived from
// theme.ribbon.lane_width_m and the role's own margin, per user directive
// 2026-09-08 ITEM 3 -- was a flat theme.ribbon.width_m * 0.5, ITEM 1) slot
// `slot`'s geometry was actually built with the last time it rebuilt --
// NOT a Filament AABB query, same "mirrors the actual value used, not a
// bounding-box read-back" reasoning as ego.cpp's
// egoFallbackDims (add_mesh() gives every mesh the same hard-coded declared
// culling box, unrelated to the strip's real extent). Ribbon.
// WidthChangeRebuildsGeometry reads this back to prove a width-only
// set_theme() (no PathRibbon point data touched) actually rebuilt the
// strip at the new width -- the vertex COUNT alone can't tell (it's
// 2*point_count regardless of width). 0.0f if `r` is null or `slot` is out
// of range.
float ribbon_slot_half_width_m(mpviz::VisualRenderer* r, size_t slot);

// The first geometry point slot `slot`'s build_slot_meshes() call actually
// used (user directive 2026-09-08, ego-proximity ribbon clip) -- written to
// `*out`, function returns false (out unchanged) if `r` is null, `slot` is
// out of range, or the slot has never built any geometry. Proves a clipped
// ribbon's mesh starts at the interpolated clip point, not the original
// PathRibbon::points[0] -- see RibbonSlot::firstPointM's own comment
// (renderer_internal.hpp).
bool ribbon_slot_first_point(mpviz::VisualRenderer* r, size_t slot, mpviz::Vec3* out);

// Total number of ribbon slot rebuilds (content, role, OR ego-clip station
// changed) since `r` was created -- same "cache-miss counter" convention as
// map_elements_test_hooks.hpp's map_element_rebuild_count(). 0 if `r` is
// null.
uint64_t ribbon_rebuild_count(mpviz::VisualRenderer* r);

}  // namespace mpviz::testing
