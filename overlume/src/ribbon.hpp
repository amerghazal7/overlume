// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume::detail {

float ribbon_fade_alpha(double station_m, double origin_station_m, float fade_start_m,
                        float fade_end_m);

}

namespace overlume {

void update_ribbons(VisualRenderer& r, const SceneGraph& scene);

}
