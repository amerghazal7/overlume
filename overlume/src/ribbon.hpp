// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume::detail {

float ribbon_fade_alpha(double station_m, double total_length_m, float fade_start);

}

namespace overlume {

void update_ribbons(VisualRenderer& r, const SceneGraph& scene);

}
