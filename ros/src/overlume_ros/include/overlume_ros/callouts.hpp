// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>

#include "overlume_ros/hud_overlay.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

struct Callout {
    char text[16];
    float anchor_x, anchor_y;
};

bool BuildNearestCallout(overlume::VisualRenderer* renderer, const overlume::AlertPolygon* alerts,
                         uint32_t alert_count, overlume::Vec3 ego_pos, Callout& out);

void DrawCallout(uint8_t* rgb, uint32_t width, uint32_t height, const Callout& callout,
                 HudRgb chip_rgb, float scale, const char* font_path);

}  // namespace overlume::ros
