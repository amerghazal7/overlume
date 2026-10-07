// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

void update_height_grids(VisualRenderer& r, const SceneGraph& scene);

// Sets the ramp texture, ramp span, unknown colour and roughness on a height_grid /
// height_grid_faded instance. `g` is passed explicitly because push_theme_to_scene runs before
// r.active_theme is updated during a theme transition.
void apply_height_grid_params(filament::MaterialInstance& inst, const VisualRenderer& r,
                              const detail::Theme::HeightGrid& g);

void destroy_height_grid_slots(VisualRenderer& r);

}
