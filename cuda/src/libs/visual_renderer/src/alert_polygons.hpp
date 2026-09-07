// alert_polygons.hpp — internal-only, `-I src` visibility, not installed,
// not POD (Epic 2 Task 7 / VM-026). Declares the per-frame counterpart to
// the three per-severity MaterialInstances renderer.cpp creates/themes:
// render_frame() calls this every tick with the last-published active()
// scene, the same create-once-in-renderer.cpp / update-every-frame-
// elsewhere split map_elements.cpp/objects.cpp/ribbon.cpp/ground_grid.cpp
// already established.
//
// Pulls in renderer_internal.hpp (and therefore <filament/...>), so — like
// those files — this is never included by tests/*.cpp; see
// alert_polygons_test_hooks.hpp (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.alerts`/`alert_count` against `r.alertSlots`, KEYED BY SLOT
// INDEX (not a content hash, not a role — AlertPolygon is frozen with no
// id, same reasoning as ribbon.cpp's RibbonSlot keying). Each slot's own
// content signature (severity + point data) decides whether it rebuilds
// this frame. Slots >= alert_count are released. Called from render_frame()
// on the one thread that owns the Engine, mirroring update_map_elements()/
// update_objects()/update_ribbons()/update_ground_grids()'s existing split.
void update_alert_polygons(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
