// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

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

// `text` is UTF-8. U+0020..U+007E and U+00A0..U+00FF have glyphs; any other code point advances
// the pen by half the line height, and bytes that start no one- or two-byte sequence are skipped.
bool DrawText(uint8_t* rgb, uint32_t width, uint32_t height, const char* text, float x, float y,
              HudRgb rgb_color, float scale, const char* font_path);

// How far DrawText advances the pen for `text`, in pixels; 0 when the font cannot be used.
float MeasureText(const char* text, float scale, const char* font_path);

// Alpha-blends an RGBA8 image (row 0 at the top, no row padding) into the RGB8 frame with its
// top-left pixel at (x, y), clipped at the frame edges. A null image draws nothing.
void DrawImageRgba(uint8_t* rgb, uint32_t width, uint32_t height, const uint8_t* rgba,
                   uint32_t image_width, uint32_t image_height, int x, int y);

struct RgbaImage {
    std::vector<uint8_t> rgba;  // row 0 at the top, 4 bytes per pixel, no row padding
    uint32_t width = 0;
    uint32_t height = 0;
};

// Decodes a PNG file to RGBA8. False, with `out` left empty, when the file is missing or is not
// a decodable PNG.
bool LoadPngRgba(const char* path, RgbaImage& out);

// A provider attribution: `logo` at native size in the bottom-left corner and `text` right-aligned
// in the bottom-right, the ink of both 8 px from the frame edges. Text that would come within 8 px
// of the logo moves up a line, to 8 px above the logo's top edge. Text wider than the frame between
// the margins starts at the left margin and is cut at the right edge. An empty logo is skipped;
// false (the logo still drawn) when the font cannot be used.
bool CompositeAttribution(uint8_t* rgb, uint32_t width, uint32_t height, const RgbaImage& logo,
                          const char* text, HudRgb text_rgb, float scale, const char* font_path);

void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color);

}
