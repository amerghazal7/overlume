// map_elements_test_hooks.hpp — internal-only, not installed, not POD.
// Same reasoning as ego_test_hooks.hpp: tests/test_map_elements.cpp links
// only against `visual_renderer` and has no access to its PRIVATE Filament
// include dir, so it can't include renderer_internal.hpp directly. These
// hooks are declared only against api.h's opaque VisualRenderer and
// theme.hpp's Filament-free Float3 — defined where the state they read
// actually lives (renderer.cpp: the ego-following ground/grid patch and
// the lane MaterialInstance's theming are both wired up there).
#pragma once

#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// Current world-space translation of the ego-following ground/grid patch
// — {0,0,0} if `r` is null or the patch has never been given a
// TransformManager component (should not happen).
mpviz::Vec3 ground_patch_centre(mpviz::VisualRenderer* r);

// The color last passed to the lane MaterialInstance's "baseColor" param
// (Filament has no getter, so this isn't a GPU read-back); mirrors
// whatever push_theme_to_scene() last pushed, updated at the call site so
// a bug that skips theming the lane material shows up here as clay.mat's
// compiled-in zero default, not a false-positive pass. {0,0,0} if `r` is
// null.
mpviz::detail::Float3 lane_material_base_color(mpviz::VisualRenderer* r);

// The per-kind MaterialInstance dispatch's base color, read the same way
// lane_material_base_color() reads laneMaterial's -- returns whichever
// *MaterialBaseColor mirror field push_theme_to_scene() last pushed for
// `kind` (mirrors material_for_kind()'s switch field-for-field, so a bug
// that swaps or collapses the dispatch shows up as a wrong color).
// {0,0,0} if `r` is null.
mpviz::detail::Float3 map_kind_base_color(mpviz::VisualRenderer* r, mpviz::MapKind kind);

// Incremented once per adopt_or_build() cache-miss branch in
// update_map_elements() (map_elements.cpp) -- a rebuild-every-frame bug
// shows up here as a nonzero delta across two identical-content
// set_scene() calls. 0 if `r` is null.
uint64_t map_element_rebuild_count(mpviz::VisualRenderer* r);

// How many live meshes update_map_elements() currently holds -- same
// mesh-count-through-a-Filament-free-hook shape as ribbon.cpp's
// ribbon_mesh_count. Used to distinguish one solid CENTERLINE mesh chunk
// from N dash-run mesh chunks for a BOUNDARY of the same geometry,
// without a full-frame SSIM. 0 if `r` is null.
size_t map_element_mesh_count(mpviz::VisualRenderer* r);

// Total vertex count summed across every mesh update_map_elements()
// currently holds -- used to prove a CENTERLINE element built dot discs
// (many small triangle fans) rather than a solid ribbon strip (few
// triangles), without a full-frame SSIM. 0 if `r` is null.
size_t map_element_total_vertex_count(mpviz::VisualRenderer* r);

// The staleness-fade state of "the" live map-element mesh — same
// shape/reasoning as AlertMaterialInfo/ObjectMaterialInfo/
// GenericMarkerMaterialInfo (MaterialInstance has no getter, so the
// CPU-stored alpha is what a test reads; `bound_to_translucent` via
// RenderableManager::getMaterialInstanceAt(), a real GPU read). Unlike
// those three, map elements have no caller-stable per-element index to
// key this by (see renderer_internal.hpp's mapElementMeshes comment), so
// this hook is meaningful only when map_element_mesh_count() == 1 (one
// MapElement in, one mesh chunk out). With 0 or >1 live meshes it returns
// the same {false, 1.0f} default a null `r` gives -- "which one?" has no
// answer, so it declines to guess.
struct MapElementMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
MapElementMaterialInfo map_element_material_info(mpviz::VisualRenderer* r);

}  // namespace mpviz::testing
