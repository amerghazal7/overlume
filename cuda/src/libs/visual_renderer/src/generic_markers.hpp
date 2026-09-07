// generic_markers.hpp — internal-only, `-I src` visibility, not installed,
// not POD (Epic 2 Task 8 / VM-027). Declares the per-frame counterpart to
// the eager theme-neutral MaterialInstance renderer.cpp creates/themes:
// render_frame() calls this every tick with the last-published active()
// scene, the same create-once-in-renderer.cpp / update-every-frame-
// elsewhere split map_elements.cpp/objects.cpp/ribbon.cpp/alert_polygons.cpp
// already established.
//
// Pulls in renderer_internal.hpp (and therefore <filament/...>), so — like
// those files — this is never included by tests/*.cpp; see
// generic_markers_test_hooks.hpp (Filament-free) for what tests use
// instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.markers`/`marker_count` against `r.genericMarkerSlots`,
// KEYED BY SLOT INDEX (not a content hash, not an id — GenericMarker is
// frozen with no id, same reasoning as RibbonSlot/AlertSlot's own slot
// keying). Each slot's own primitive + content signature decides whether
// its geometry rebuilds this frame; a primitive change on a slot tears
// down and re-acquires it fresh. Slots >= marker_count are released.
// Called from render_frame() on the one thread that owns the Engine,
// mirroring update_map_elements()/update_objects()/update_ribbons()/
// update_alert_polygons()'s existing split. Must run AFTER
// update_alert_polygons() (the plan's own ordering: alerts are the
// topmost non-fallback category, generic markers are the last thing
// drawn — a debug/parity layer, not meant to hide under anything).
void update_generic_markers(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
