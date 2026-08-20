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

}  // namespace mpviz::testing
