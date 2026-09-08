// ground_grid.hpp — internal-only (`-I src`), not installed, not POD.
// Declares the per-frame update for the two per-kind ground_grid.mat
// instances renderer.cpp creates/themes.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see ground_grid_test_hooks.hpp (Filament-free)
// for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.grids`/`grid_count` against `r.groundGridSlots` (keyed by
// slot index, same shape as ribbonSlots -- see renderer_internal.hpp): a
// slot whose origin/dims/resolution changed gets its quad rebuilt; a slot
// whose cell dims changed gets its texture destroyed and recreated;
// same-dims cell data is uploaded in place via Texture::setImage -- but
// only on an actual content change (a dims/geometry rebuild, or the
// grid's own last_update_sec advancing past what this slot last
// uploaded). A render with no new message re-touches nothing: no upload,
// no allocation, no PCIe traffic. Slots `>= grid_count` are torn down.
// Called from render_frame() on the thread that owns the Engine.
void update_ground_grids(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
