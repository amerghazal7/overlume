// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

struct BowlState {
    filament::Material* material = nullptr;
    filament::MaterialInstance* instance = nullptr;
    Mesh mesh;
    bool inScene = false;
    bool stencilNe = false;  // compare last applied to THIS instance (a re-bake resets it)
};

// Rig -> world transform of the bowl for this ego (identity when the ego is invalid).
filament::math::mat4f ego_anchor_transform(const EgoState& ego);

bool build_bowl(VisualRenderer& r, const BowlConfig& cfg);

void update_bowl(VisualRenderer& r, const EgoState& ego);

}
