// map_elements_test_hooks.hpp — internal-only, `-I src` visibility, not
// installed, not POD (Epic 2 Task 2 / VM-024). Same reasoning as Task 4's
// ego_test_hooks.hpp: tests/test_map_elements.cpp links only against
// `visual_renderer` and is never granted that target's PRIVATE Filament
// include dir, so it can't include renderer_internal.hpp directly. These
// hooks are declared against nothing but api.h's opaque mpviz::
// VisualRenderer and theme.hpp's Filament-free mpviz::detail::Float3 —
// defined where the state they read actually lives (renderer.cpp: both the
// ego-following ground/grid patch and the lane MaterialInstance's theming
// are wired up there, per the plan's Task 2 Files list).
#pragma once

#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// Current world-space translation of the ego-following ground/grid patch
// (Task 2 Step 1/2) — {0,0,0} if `r` is null or the patch has never been
// given a TransformManager component (should not happen post-Step 2).
mpviz::Vec3 ground_patch_centre(mpviz::VisualRenderer* r);

// The color value last actually passed to the lane MaterialInstance's
// "baseColor" setParameter call (Step 8a) — NOT a read-back of live GPU
// state (Filament's MaterialInstance exposes no getter for a set
// parameter); mirrors whatever push_theme_to_scene() last pushed, updated
// at the exact call site so a bug that skips theming the lane material
// specifically (as opposed to merely tracking a theme that was decided)
// shows up here as clay.mat's compiled-in zero default, not a
// false-positive pass. {0,0,0} if `r` is null.
mpviz::detail::Float3 lane_material_base_color(mpviz::VisualRenderer* r);

// Epic 3 Task 1 (VM-036) decision #6: the per-kind MaterialInstance
// dispatch's base color, read the same way lane_material_base_color() reads
// laneMaterial's -- returns whichever *MaterialBaseColor mirror field
// push_theme_to_scene() last pushed for `kind` (renderer.cpp's
// material_for_kind() switch, mirrored here field-for-field so a bug that
// swaps or collapses the dispatch shows up as a wrong color, not just a
// wrong MaterialInstance pointer no test can see). {0,0,0} if `r` is null.
mpviz::detail::Float3 map_kind_base_color(mpviz::VisualRenderer* r, mpviz::MapKind kind);

// Epic 3 Task 1 (VM-036) Step 7: incremented once per adopt_or_build()
// cache-MISS branch in update_map_elements() (map_elements.cpp) -- Epic 2's
// "cached, no per-frame rebuild" AC, finally checked (a rebuild-once-and-
// never-rebuild-again bug and a rebuild-every-frame bug both show up here,
// as a nonzero delta across two identical-content set_scene() calls). 0 if
// `r` is null.
uint64_t map_element_rebuild_count(mpviz::VisualRenderer* r);

// Epic 3 Task 1 (VM-036) Step 4: how many live meshes update_map_elements()
// currently holds -- same "mesh COUNT through a Filament-free hook" shape
// as ribbon.cpp's ribbon_mesh_count (test_ribbon.cpp's
// LongPathSplitsAcrossMeshesWithoutTruncation is the precedent this
// mirrors). Used to distinguish "one solid CENTERLINE mesh chunk" from
// "N dash-run mesh chunks for a BOUNDARY of the same geometry" without a
// full-frame SSIM (which would also carry road-fill/per-kind color, a
// separate concern per decision #3's AC). 0 if `r` is null.
size_t map_element_mesh_count(mpviz::VisualRenderer* r);

// User directive 2026-09-08 (post Task 1 candidate review): total vertex
// count summed across every mesh update_map_elements() currently holds --
// used to prove a CENTERLINE element built dot DISCS (many small triangle
// fans) rather than the old solid ribbon STRIP (few triangles), without a
// full-frame SSIM (CenterlineRendersAsDotDiscsNotAStrip, test_map_
// elements.cpp). 0 if `r` is null.
size_t map_element_total_vertex_count(mpviz::VisualRenderer* r);

}  // namespace mpviz::testing
