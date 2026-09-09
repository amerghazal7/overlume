// trajectory_carpet.hpp — internal-only (`-I src`), not installed, not POD.
// Declares the per-frame update for the ONE trajectory_carpet.mat instance
// the whole TrajectoryCarpetLayer shares (VM-077), mirroring point_cloud.hpp
// exactly.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see trajectory_carpet_test_hooks.hpp
// (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.trajectory_carpets`/`_count` against `r.trajectoryCarpetSlots`
// (keyed by slot index, same shape as pointCloudSlots): a slot whose content
// signature changed gets its (possibly chunked, past polyline.hpp's
// kMaxPointsPerMesh) meshes rebuilt from scratch, as a TRIANGLES primitive
// (the wire data is already a flat triangle list -- sequential indices, no
// fan/strip reinterpretation). Slots >= trajectory_carpet_count are torn
// down. The single shared trajectory_carpet.mat instance's "alpha" parameter
// is set once per call from the freshest live carpet's staleness_alpha()
// (same "freshest of several" stated choice as point_cloud.cpp -- today's
// shipped profile carries exactly one row, so this never fires in
// practice). Called from render_frame() on the thread that owns the Engine.
void update_trajectory_carpets(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
