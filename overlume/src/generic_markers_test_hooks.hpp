// generic_markers_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_generic_markers.cpp links only against `visual_renderer` and
// has no access to its PRIVATE Filament include dir, so it can't include
// generic_markers.hpp/renderer_internal.hpp directly. These hooks are
// declared only against api.h's opaque VisualRenderer, scene.h's POD
// types, and theme.hpp's Filament-free Float3 — defined in
// generic_markers.cpp, where the state they read actually lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// Number of live generic-marker slots (0-indexed into active().markers) —
// 0 if `r` is null.
size_t generic_marker_slot_count(mpviz::VisualRenderer* r);

// The pool's monotonically-increasing GPU/ECS-allocation counter
// (renderer_internal.hpp's genericMarkerAllocCount) — bumped only when a
// slot allocates a new entity/mesh/glTF asset, never on a transform-only
// or material-swap-only update. Must freeze across unchanged-scene
// renders after the first. 0 if `r` is null.
uint32_t generic_marker_alloc_count(mpviz::VisualRenderer* r);

// Count of markers skipped this frame for an out-of-range primitive
// value. 0 if `r` is null.
uint32_t generic_marker_unknown_count(mpviz::VisualRenderer* r);

// True iff slot `slot` is a MESH marker currently rendering the
// load-failure clay-box fallback rather than a loaded glTF asset. False
// if `r` is null, `slot` is out of range, or the slot isn't a MESH
// marker.
bool generic_marker_mesh_is_fallback(mpviz::VisualRenderer* r, size_t slot);

// The rgb currently applied to slot `slot`'s material (the theme-neutral
// default, or the marker's own supplied color) — mirrors
// object_class_tint()'s/alert_severity_base_color()'s "MaterialInstance
// has no getter" pattern. {0,0,0} if `r` is null or `slot` is out of
// range.
mpviz::detail::Float3 generic_marker_tint(mpviz::VisualRenderer* r, size_t slot);

// Which material slot `slot`'s renderable(s) are currently bound to, read
// via RenderableManager::getMaterialInstanceAt() (a real GPU-state getter,
// unlike MaterialInstance's write-only params) — same shape as
// AlertMaterialInfo/ObjectMaterialInfo. `alpha` mirrors the CPU-stored
// baseColor.a last applied: 1.0 while fresh, staleness_alpha() while
// fading.
struct GenericMarkerMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
GenericMarkerMaterialInfo generic_marker_material_info(mpviz::VisualRenderer* r, size_t slot);

}  // namespace mpviz::testing
