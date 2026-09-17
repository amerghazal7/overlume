// ego_test_hooks.hpp — internal-only, not installed, not POD. Separate
// from renderer_internal.hpp/ego.hpp: tests/test_ego.cpp links only
// against `overlume` and has no access to its PRIVATE Filament
// include dir, so it can't include a header that pulls in <filament/...>.
// Declared against api.h's opaque VisualRenderer instead — all
// test_ego.cpp needs, since it only holds the pointer.
#pragma once

#include "theme.hpp"
#include "overlume/api.h"

namespace overlume::testing {

// Diagonal (meters) of the currently-rendered ego's bounding box: from the
// loaded gltfio asset if set_ego_model() loaded one, else from the
// clay-box fallback's dims, 0.0 if neither has been built yet. >0 is this
// suite's proxy for "something real got built and is renderable."
double rendered_bounding_box_diagonal(overlume::VisualRenderer* r);

// The color last passed to the ego MaterialInstance's "baseColor" param
// (mirrored CPU copy — Filament has no getter); defined in renderer.cpp,
// where egoMaterial is created and themed, not ego.cpp. {0,0,0} if `r` is
// null.
overlume::detail::Float3 ego_material_base_color(overlume::VisualRenderer* r);

}  // namespace overlume::testing
