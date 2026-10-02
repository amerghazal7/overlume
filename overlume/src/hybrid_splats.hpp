// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

// Per frame, after update_bowl: show/hide the splat meshes, anchor them with the ego transform,
// push size + exposure, and toggle the stencil state of the backdrop.
void update_hybrid_splats(VisualRenderer& r, const EgoState& ego);

// View stencil on/off and the compare function of the bowl, ground and grid instances (NE 1 while
// the layer is active, A otherwise). Records what it applied for the test hook.
void apply_backdrop_stencil(VisualRenderer& r, bool active);

void create_hybrid_splat_material(VisualRenderer& r);
void destroy_hybrid_splats(VisualRenderer& r);

}
