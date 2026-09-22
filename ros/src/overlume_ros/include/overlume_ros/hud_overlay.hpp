// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>

#include "overlume/scene.h"

namespace overlume::ros {

void PopulateHud(overlume::SceneGraph& scene, int render_mode);

struct HudRgb {
    float r, g, b;
};

struct HudSnapshot {
    double speed_mps;
    uint8_t active_mode;
};

bool CompositeHud(uint8_t* rgb, uint32_t width, uint32_t height, const HudSnapshot& hud,
                  HudRgb text_rgb, HudRgb accent_rgb, float scale, const char* font_path);

bool DrawText(uint8_t* rgb, uint32_t width, uint32_t height, const char* text, float x, float y,
              HudRgb rgb_color, float scale, const char* font_path);

void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color);

}  // namespace overlume::ros
