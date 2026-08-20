// ground_grid.hpp — internal-only, `-I src` visibility, not installed, not
// POD (Epic 2 Task 6 / VM-025). Declares the per-frame counterpart to the
// two per-kind ground_grid.mat instances renderer.cpp creates/themes: the
// same create-once-in-renderer.cpp / update-every-frame-elsewhere split
// apply_current_theme()/update_ego_transform()/update_map_elements()/
// update_ribbons() already established.
//
// Pulls in renderer_internal.hpp (and therefore <filament/...>), so this is
// never included by tests/*.cpp — see ground_grid_test_hooks.hpp
// (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.grids`/`grid_count` against `r.groundGridSlots` (keyed by
// SLOT INDEX, same shape as ribbonSlots — see that member's own comment in
// renderer_internal.hpp): a slot whose origin/dims/resolution changed since
// the last call gets its quad rebuilt; a slot whose cell dims changed gets
// its texture destroyed and recreated; same-dims cell data is uploaded IN
// PLACE via Texture::setImage -- but ONLY on an actual content change
// (a dims/geometry rebuild, or the grid's own last_update_sec advancing
// past what this slot last uploaded, i.e. a genuinely new
// ingest()/ingest_update() arrived). A render with no new message since
// the last one re-touches nothing: no upload, no allocation, no PCIe
// traffic for bytes the texture already holds. Slots `>= grid_count` are
// torn down (mesh + texture). Called from render_frame() on the one
// thread that owns the Engine, mirroring
// update_map_elements()/update_ribbons()'s existing split.
void update_ground_grids(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
