// generic_markers_test_hooks.hpp — internal-only, `-I src` visibility, not
// installed, not POD (Epic 2 Task 8 / VM-027). Same reasoning as
// alert_polygons_test_hooks.hpp/objects_test_hooks.hpp: tests/
// test_generic_markers.cpp links only against `visual_renderer` and is
// never granted that target's PRIVATE Filament include dir, so it can't
// include generic_markers.hpp/renderer_internal.hpp directly. These hooks
// are declared against nothing but api.h's opaque mpviz::VisualRenderer,
// scene.h's POD types, and theme.hpp's Filament-free mpviz::detail::Float3
// — defined in generic_markers.cpp, where the state they read actually
// lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// Number of live generic-marker slots (0-indexed into active().markers,
// the plan's "diff by marker index" decision) — 0 if `r` is null.
size_t generic_marker_slot_count(mpviz::VisualRenderer* r);

// The pool's monotonically-increasing GPU/ECS-allocation counter
// (renderer_internal.hpp's genericMarkerAllocCount) — bumped only when a
// slot allocates a NEW entity/mesh/glTF asset, never on a transform-only
// or material-swap-only update. GenericMarkers.
// PooledRenderablesNoPerFrameAllocation reads this across 30
// unchanged-scene renders: it must freeze after the first. 0 if `r` is
// null.
uint32_t generic_marker_alloc_count(mpviz::VisualRenderer* r);

// Count of markers skipped THIS FRAME for an out-of-range primitive value
// (GenericMarkers.UnknownOrUnsupportedPrimitiveIsSkippedAndCounted). 0 if
// `r` is null.
uint32_t generic_marker_unknown_count(mpviz::VisualRenderer* r);

// True iff slot `slot` is a MESH marker currently rendering the
// load-failure clay-box fallback rather than a loaded glTF asset
// (GenericMarkers.MeshPathLoadFailureFallsBackToClayBoxWarnOnce). False if
// `r` is null, `slot` is out of range, or the slot isn't a MESH marker.
bool generic_marker_mesh_is_fallback(mpviz::VisualRenderer* r, size_t slot);

// The rgb currently applied to slot `slot`'s material (the theme-neutral
// default, or the marker's own supplied color) — mirrors object_class_
// tint()'s/alert_severity_base_color()'s own "MaterialInstance has no
// getter" pattern. {0,0,0} if `r` is null or `slot` is out of range.
mpviz::detail::Float3 generic_marker_tint(mpviz::VisualRenderer* r, size_t slot);

// Which material slot `slot`'s renderable(s) are CURRENTLY bound to, read
// via RenderableManager::getMaterialInstanceAt() (a real GPU-state getter,
// unlike MaterialInstance's own parameters, which are write-only) — same
// shape as AlertMaterialInfo/ObjectMaterialInfo. `alpha` mirrors the
// CPU-stored baseColor.a last actually applied to this slot
// (MaterialInstance has no parameter getter): 1.0 while fresh,
// staleness_alpha() while fading.
struct GenericMarkerMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
GenericMarkerMaterialInfo generic_marker_material_info(mpviz::VisualRenderer* r, size_t slot);

}  // namespace mpviz::testing
