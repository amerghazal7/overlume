// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/hud_overlay.hpp"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace overlume::ros {

void PopulateHud(overlume::SceneGraph& scene, int render_mode) {
    scene.hud.speed_mps = scene.ego.speed_mps;
    scene.hud.active_mode = static_cast<uint8_t>(render_mode);
}

namespace {

constexpr int kAtlasDim = 512;
constexpr int kFirstChar = 32;
constexpr int kNumChars = 95;
constexpr float kBasePixelHeight = 32.0f;

struct FontAtlas {
    std::string path;
    float scale = 0.0f;
    bool ok = false;
    float pixel_height = 0.0f;
    std::vector<unsigned char> bitmap;
    std::vector<stbtt_bakedchar> chars;
};

bool bake(const std::string& path, float pixel_height, FontAtlas& atlas) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    const std::vector<unsigned char> ttf((std::istreambuf_iterator<char>(in)),
                                         std::istreambuf_iterator<char>());
    if (ttf.empty()) return false;

    atlas.bitmap.assign(static_cast<size_t>(kAtlasDim) * kAtlasDim, 0);
    atlas.chars.assign(static_cast<size_t>(kNumChars), stbtt_bakedchar{});
    const int rows_used =
        stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, atlas.bitmap.data(), kAtlasDim, kAtlasDim,
                             kFirstChar, kNumChars, atlas.chars.data());
    return rows_used > 0;
}

const FontAtlas* get_atlas(const std::string& font_path, float scale) {
    const float qscale = std::round(scale * 10.0f) / 10.0f;
    static FontAtlas atlas;
    if (atlas.path != font_path || atlas.scale != qscale) {
        FontAtlas next;
        next.path = font_path;
        next.scale = qscale;
        next.pixel_height = kBasePixelHeight * qscale;
        next.ok = bake(font_path, next.pixel_height, next);
        atlas = std::move(next);
    }
    return atlas.ok ? &atlas : nullptr;
}

void blend_pixel(uint8_t* rgb, uint32_t width, uint32_t height, int x, int y, float r, float g,
                 float b, float coverage) {
    if (x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height)) return;
    if (coverage <= 0.0f) return;
    uint8_t* px = rgb + (static_cast<size_t>(y) * width + x) * 3;
    px[0] = static_cast<uint8_t>(px[0] * (1.0f - coverage) + r * 255.0f * coverage);
    px[1] = static_cast<uint8_t>(px[1] * (1.0f - coverage) + g * 255.0f * coverage);
    px[2] = static_cast<uint8_t>(px[2] * (1.0f - coverage) + b * 255.0f * coverage);
}

float draw_line(uint8_t* rgb, uint32_t width, uint32_t height, const FontAtlas& atlas,
                const std::string& text, float x, float y, float r, float g, float b) {
    for (unsigned char c : text) {
        if (c < kFirstChar || c >= kFirstChar + kNumChars) {
            x += atlas.pixel_height * 0.5f;
            continue;
        }
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(atlas.chars.data(), kAtlasDim, kAtlasDim, c - kFirstChar, &x, &y, &q, 1);
        const int gx0 = static_cast<int>(std::floor(q.x0));
        const int gx1 = static_cast<int>(std::ceil(q.x1));
        const int gy0 = static_cast<int>(std::floor(q.y0));
        const int gy1 = static_cast<int>(std::ceil(q.y1));
        if (gx1 <= gx0 || gy1 <= gy0) continue;
        const float u_span = q.s1 - q.s0, v_span = q.t1 - q.t0;
        const float gw = static_cast<float>(gx1 - gx0), gh = static_cast<float>(gy1 - gy0);
        for (int py = gy0; py < gy1; ++py) {
            const float v = q.t0 + ((py - gy0) + 0.5f) / gh * v_span;
            const int ay = static_cast<int>(v * kAtlasDim);
            if (ay < 0 || ay >= kAtlasDim) continue;
            for (int px = gx0; px < gx1; ++px) {
                const float u = q.s0 + ((px - gx0) + 0.5f) / gw * u_span;
                const int ax = static_cast<int>(u * kAtlasDim);
                if (ax < 0 || ax >= kAtlasDim) continue;
                const float coverage =
                    atlas.bitmap[static_cast<size_t>(ay) * kAtlasDim + ax] / 255.0f;
                blend_pixel(rgb, width, height, px, py, r, g, b, coverage);
            }
        }
    }
    return x;
}

}  // namespace

bool CompositeHud(uint8_t* rgb, uint32_t width, uint32_t height, const HudSnapshot& hud,
                  HudRgb text_rgb, HudRgb accent_rgb, float scale, const char* font_path) {
    if (rgb == nullptr || width == 0 || height == 0 || font_path == nullptr ||
        font_path[0] == '\0') {
        return false;
    }
    const float safe_scale = scale > 0.0f ? scale : 1.0f;
    const FontAtlas* atlas = get_atlas(font_path, safe_scale);
    if (atlas == nullptr) return false;

    char speed_buf[32];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.1f m/s", hud.speed_mps);
    char mode_buf[16];
    switch (hud.active_mode) {
        case 1:
            std::snprintf(mode_buf, sizeof(mode_buf), "BOWL");
            break;
        case 2:
            std::snprintf(mode_buf, sizeof(mode_buf), "POINTCLOUD");
            break;
        case 3:
            std::snprintf(mode_buf, sizeof(mode_buf), "VISUAL");
            break;
        default:
            std::snprintf(mode_buf, sizeof(mode_buf), "MODE %d", static_cast<int>(hud.active_mode));
            break;
    }

    constexpr float kMarginX = 24.0f, kMarginY = 8.0f, kLineGap = 8.0f;
    float x = kMarginX, y = kMarginY + atlas->pixel_height;
    draw_line(rgb, width, height, *atlas, speed_buf, x, y, text_rgb.r, text_rgb.g, text_rgb.b);

    float x2 = kMarginX, y2 = y + atlas->pixel_height + kLineGap;
    draw_line(rgb, width, height, *atlas, mode_buf, x2, y2, accent_rgb.r, accent_rgb.g,
              accent_rgb.b);
    return true;
}

bool DrawText(uint8_t* rgb, uint32_t width, uint32_t height, const char* text, float x, float y,
              HudRgb rgb_color, float scale, const char* font_path) {
    if (rgb == nullptr || width == 0 || height == 0 || text == nullptr || font_path == nullptr ||
        font_path[0] == '\0') {
        return false;
    }
    const float safe_scale = scale > 0.0f ? scale : 1.0f;
    const FontAtlas* atlas = get_atlas(font_path, safe_scale);
    if (atlas == nullptr) return false;
    draw_line(rgb, width, height, *atlas, std::string(text), x, y, rgb_color.r, rgb_color.g,
              rgb_color.b);
    return true;
}

void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color) {
    if (rgb == nullptr || width == 0 || height == 0) return;
    int x = static_cast<int>(std::lround(x0));
    int y = static_cast<int>(std::lround(y0));
    const int ix1 = static_cast<int>(std::lround(x1));
    const int iy1 = static_cast<int>(std::lround(y1));
    const int dx = std::abs(ix1 - x), sx = x < ix1 ? 1 : -1;
    const int dy = -std::abs(iy1 - y), sy = y < iy1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        blend_pixel(rgb, width, height, x, y, rgb_color.r, rgb_color.g, rgb_color.b, 1.0f);
        if (x == ix1 && y == iy1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
}

}  // namespace overlume::ros
