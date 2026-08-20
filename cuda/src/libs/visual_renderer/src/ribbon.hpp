// ribbon.hpp — internal-only, `-I src` visibility, not installed, not POD
// (Epic 2 Task 5 / VM-023). Declares the per-frame counterpart to the three
// ribbon-role MaterialInstances renderer.cpp creates/themes: render_frame()
// calls this every tick with the last-published active() scene, the same
// create-once-in-renderer.cpp / update-every-frame-elsewhere split
// map_elements.cpp/objects.cpp already established.
//
// Pulls in renderer_internal.hpp (and therefore <filament/...>), so — like
// that header — this is never included by tests/*.cpp; see
// ribbon_test_hooks.hpp (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.paths`/`path_count` against `r.ribbonSlots`, KEYED BY SLOT
// INDEX (not role — see renderer_internal.hpp's RibbonSlot comment and the
// plan's Task 5 Step 4 for why: both shipped profiles put two rows on role
// LOCAL, so SceneAssembly::paths routinely holds four ribbons over three
// roles in one frame). Each slot's own content signature (role + point
// data) decides whether it rebuilds this frame; a role change on a slot
// re-homes it to the right material. Slots >= path_count are released.
// Called from render_frame() on the one thread that owns the Engine,
// mirroring update_map_elements()/update_objects()'s existing split.
void update_ribbons(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
