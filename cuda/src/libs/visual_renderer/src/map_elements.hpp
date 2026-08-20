// map_elements.hpp — internal-only, `-I src` visibility, not installed, not
// POD (Epic 2 Task 2 / VM-024). Declares the per-frame counterpart to the
// lane MaterialInstance renderer.cpp creates/themes: render_frame() calls
// this every tick with the last-published active() scene, the same
// create-once-in-renderer.cpp / update-every-frame-elsewhere split Task 3
// (apply_current_theme) and Task 4 (update_ego_transform) already
// established.
//
// Pulls in renderer_internal.hpp (and therefore <filament/...>), so — like
// that header — this is never included by tests/*.cpp; see
// map_elements_test_hooks.hpp (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.map_elements`/`map_element_count` against
// `r.mapElementMeshes` (keyed by content signature, not array position —
// see that member's own comment in renderer_internal.hpp) and rebuilds only
// what changed: new signatures get a Mesh built (polyline -> extruded
// ribbon on r.laneMaterial; polygon -> triangulated fan + hatch, same
// material instance), matching signatures are kept untouched, and
// signatures no longer present are destroyed. Called from render_frame()
// on the one thread that owns the Engine, mirroring update_ego_transform()/
// apply_current_theme()'s existing split.
void update_map_elements(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
