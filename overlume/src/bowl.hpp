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
};

bool build_bowl(VisualRenderer& r, const BowlConfig& cfg);

void update_bowl(VisualRenderer& r, const EgoState& ego);

}
