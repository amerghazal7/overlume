// point_cloud.hpp — internal-only (`-I src`), not installed, not POD.
// Declares the per-frame update for the ONE point_cloud.mat instance the
// whole PointCloudLayer shares (Epic 3 Task 6 / VM-035).
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see point_cloud_test_hooks.hpp (Filament-free)
// for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.point_clouds`/`point_cloud_count` against `r.pointCloudSlots`
// (keyed by slot index, same shape as ribbonSlots/groundGridSlots): a slot
// whose content signature changed gets its (possibly chunked, past
// polyline.hpp's kMaxPointsPerMesh) meshes rebuilt from scratch; an
// unchanged cloud rebuilds nothing. Slots >= point_cloud_count are torn
// down. The single shared point_cloud.mat instance's "alpha" parameter is
// set once per call from the freshest live cloud's staleness_alpha() (see
// this file's .cpp for why "freshest of several" is the stated choice for
// the N>1 case). Called from render_frame() on the thread that owns the
// Engine.
void update_point_clouds(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
