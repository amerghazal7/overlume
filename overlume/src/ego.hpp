// ego.hpp — internal-only (`-I src`), not installed, not POD. Declares the
// per-frame counterpart to set_ego_model() (scene.h/ego.cpp):
// set_ego_model() builds/loads the entity once, update_ego_transform()
// updates its TransformManager transform every render_frame().
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see ego_test_hooks.hpp (Filament-free) for
// what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

// Drives whichever entity set_ego_model() populated (r.egoTransformEntity)
// from `ego.position`/`ego.heading_rad` every render_frame() call, on the
// thread that owns the Engine. `ego.valid == 0` zero-scales the transform
// instead of touching scene membership — see ego.cpp for why. No-op if
// set_ego_model() was never called.
void update_ego_transform(VisualRenderer& r, const EgoState& ego);

}  // namespace overlume
