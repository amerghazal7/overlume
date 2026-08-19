// ego.hpp — internal-only, `-I src` visibility, not installed, not POD
// (Epic 1 Task 4 / VM-012). Declares the per-frame counterpart to
// set_ego_model() (mpviz::set_ego_model, declared publicly in scene.h,
// defined in ego.cpp): render_frame() (renderer.cpp) calls
// update_ego_transform() every frame with the just-published SceneGraph::
// ego, the same split Task 3 already uses for apply_current_theme() —
// set_ego_model() builds/loads the entity ONCE, this updates its
// TransformManager transform EVERY frame.
//
// Pulls in renderer_internal.hpp (and therefore <filament/...>), so — like
// that header — this is never included by tests/*.cpp; see
// ego_test_hooks.hpp (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Drives whichever entity set_ego_model() populated (r.egoTransformEntity)
// from `ego.position`/`ego.heading_rad` every render_frame() call, on the
// one thread that owns the Engine — mirrors set_scene()/render_frame()'s
// existing split (Task 1/Task 3): set_ego_model() never touches
// TransformManager itself, only this does. `ego.valid == 0` (no TF yet,
// spec §9's non-fatal "no data" path) zero-scales the transform instead of
// touching scene membership — simpler than tracking add/remove across a
// glTF asset's whole entity list (root + N node entities), and a
// zero-scale renderable has no visible extent and casts no shadow either.
// No-op if set_ego_model() was never called (r.egoTransformEntity is null).
void update_ego_transform(VisualRenderer& r, const EgoState& ego);

}  // namespace mpviz
