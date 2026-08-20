// ego_test_hooks.hpp — internal-only, `-I src` visibility, not installed,
// not POD (Epic 1 Task 4 / VM-012). Deliberately a separate header from
// renderer_internal.hpp/ego.hpp: `tests/test_ego.cpp` needs to call this
// hook but its test binary links only against `visual_renderer` and is
// never granted that target's PRIVATE Filament include dir (see
// renderer_internal.hpp's own header comment) — so it can't include a
// header that pulls in <filament/...>. This declares the hook against
// nothing but api.h's already-forward-declared opaque `mpviz::
// VisualRenderer` instead, which is all test_ego.cpp needs: it only ever
// holds the pointer, never dereferences the class.
#pragma once

#include "theme.hpp"
#include "visual_renderer/api.h"

namespace mpviz::testing {

// Diagonal (meters) of the currently-rendered ego's bounding box: computed
// from the loaded gltfio asset's own bounding box if set_ego_model()
// loaded one, else from the clay-box fallback's RenderableManager AABB, 0.0
// if neither has been built yet. >0 is this test suite's proxy for
// "something real got built and is renderable" — ego.cpp defines this,
// where the full VisualRenderer/Mesh/FilamentAsset types it needs to
// compute anything are visible.
double rendered_bounding_box_diagonal(mpviz::VisualRenderer* r);

// The color value last actually passed to the ego MaterialInstance's
// "baseColor" setParameter call (user directive 2026-08-20) — same
// mirrored-CPU-copy pattern as map_elements_test_hooks.hpp's
// lane_material_base_color() (Filament's MaterialInstance has no getter);
// defined in renderer.cpp, where egoMaterial is created and themed
// (push_theme_to_scene()), not ego.cpp. {0,0,0} if `r` is null.
mpviz::detail::Float3 ego_material_base_color(mpviz::VisualRenderer* r);

}  // namespace mpviz::testing
