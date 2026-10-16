// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/hud_overlay.hpp"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

// The node's only stb_image implementation: static, so test executables that compile this file
// beside their own STB_IMAGE_IMPLEMENTATION still link; PNG only, for the attribution logo.
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <algorithm>
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
// U+0020..U+00FF in code-point order: ASCII packs into the same atlas spots as the old ASCII-only
// bake, so its glyphs rasterise exactly as before. DEL and the C1 controls are baked too (as the
// font's .notdef box) but never drawn.
constexpr int kFirstChar = 32;
constexpr int kNumChars = 224;
constexpr float kBasePixelHeight = 32.0f;

struct FontAtlas {
    std::string path;
    float scale = 0.0f;
    bool ok = false;
    float pixel_height = 0.0f;
    int dim = kAtlasDim;
    std::vector<unsigned char> bitmap;
    std::vector<stbtt_bakedchar> chars;
};

bool bake(const std::string& path, float pixel_height, FontAtlas& atlas) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    const std::vector<unsigned char> ttf((std::istreambuf_iterator<char>(in)),
                                         std::istreambuf_iterator<char>());
    if (ttf.empty()) return false;

    atlas.chars.assign(static_cast<size_t>(kNumChars), stbtt_bakedchar{});
    // NotoSans fits 512 x 512 up to HUD scale 2.0; a larger scale gets a larger atlas instead of
    // losing the HUD (the ASCII-only atlas fitted up to 3.3).
    for (atlas.dim = kAtlasDim; atlas.dim <= 4 * kAtlasDim; atlas.dim *= 2) {
        atlas.bitmap.assign(static_cast<size_t>(atlas.dim) * atlas.dim, 0);
        if (stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, atlas.bitmap.data(), atlas.dim,
                                 atlas.dim, kFirstChar, kNumChars, atlas.chars.data()) > 0) {
            return true;
        }
    }
    return false;
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

// The code point starting at text[i], with i moved past it; -1 for a byte that starts no
// well-formed one- or two-byte UTF-8 sequence (that byte alone is skipped).
int next_code_point(const std::string& text, size_t& i) {
    const unsigned char lead = static_cast<unsigned char>(text[i++]);
    if (lead < 0x80) return lead;
    if (lead >= 0xC2 && lead <= 0xDF && i < text.size()) {
        const unsigned char cont = static_cast<unsigned char>(text[i]);
        if ((cont & 0xC0) == 0x80) {
            ++i;
            return ((lead & 0x1F) << 6) | (cont & 0x3F);
        }
    }
    return -1;
}

bool has_glyph(int cp) { return (cp >= 0x20 && cp <= 0x7E) || (cp >= 0xA0 && cp <= 0xFF); }

// Lays `text` out from the pen at (x, y) and returns the pen x after it. Draws only when `rgb` is
// not null, so measuring and drawing advance the pen identically. `ink_bottom`, when given, is
// raised to the lowest glyph edge below y.
float draw_line(uint8_t* rgb, uint32_t width, uint32_t height, const FontAtlas& atlas,
                const std::string& text, float x, float y, float r, float g, float b,
                float* ink_bottom = nullptr) {
    for (size_t i = 0; i < text.size();) {
        const int cp = next_code_point(text, i);
        if (cp < 0) continue;
        if (!has_glyph(cp)) {
            x += atlas.pixel_height * 0.5f;
            continue;
        }
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(atlas.chars.data(), atlas.dim, atlas.dim, cp - kFirstChar, &x, &y, &q,
                           1);
        if (ink_bottom != nullptr) *ink_bottom = std::max(*ink_bottom, q.y1 - y);
        if (rgb == nullptr) continue;
        const int gx0 = static_cast<int>(std::floor(q.x0));
        const int gx1 = static_cast<int>(std::ceil(q.x1));
        const int gy0 = static_cast<int>(std::floor(q.y0));
        const int gy1 = static_cast<int>(std::ceil(q.y1));
        if (gx1 <= gx0 || gy1 <= gy0) continue;
        const float u_span = q.s1 - q.s0, v_span = q.t1 - q.t0;
        const float gw = static_cast<float>(gx1 - gx0), gh = static_cast<float>(gy1 - gy0);
        for (int py = gy0; py < gy1; ++py) {
            const float v = q.t0 + ((py - gy0) + 0.5f) / gh * v_span;
            const int ay = static_cast<int>(v * atlas.dim);
            if (ay < 0 || ay >= atlas.dim) continue;
            for (int px = gx0; px < gx1; ++px) {
                const float u = q.s0 + ((px - gx0) + 0.5f) / gw * u_span;
                const int ax = static_cast<int>(u * atlas.dim);
                if (ax < 0 || ax >= atlas.dim) continue;
                const float coverage =
                    atlas.bitmap[static_cast<size_t>(ay) * atlas.dim + ax] / 255.0f;
                blend_pixel(rgb, width, height, px, py, r, g, b, coverage);
            }
        }
    }
    return x;
}

}

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

float MeasureText(const char* text, float scale, const char* font_path) {
    if (text == nullptr || font_path == nullptr || font_path[0] == '\0') return 0.0f;
    const FontAtlas* atlas = get_atlas(font_path, scale > 0.0f ? scale : 1.0f);
    if (atlas == nullptr) return 0.0f;
    return draw_line(nullptr, 0, 0, *atlas, std::string(text), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void DrawImageRgba(uint8_t* rgb, uint32_t width, uint32_t height, const uint8_t* rgba,
                   uint32_t image_width, uint32_t image_height, int x, int y) {
    if (rgb == nullptr || rgba == nullptr) return;
    for (uint32_t iy = 0; iy < image_height; ++iy) {
        const int64_t fy = static_cast<int64_t>(y) + iy;
        if (fy < 0 || fy >= static_cast<int64_t>(height)) continue;
        for (uint32_t ix = 0; ix < image_width; ++ix) {
            const int64_t fx = static_cast<int64_t>(x) + ix;
            if (fx < 0 || fx >= static_cast<int64_t>(width)) continue;
            const uint8_t* src = rgba + (static_cast<size_t>(iy) * image_width + ix) * 4;
            uint8_t* dst = rgb + (static_cast<size_t>(fy) * width + static_cast<size_t>(fx)) * 3;
            const uint32_t a = src[3];
            for (int c = 0; c < 3; ++c) {
                dst[c] = static_cast<uint8_t>((dst[c] * (255u - a) + src[c] * a + 127u) / 255u);
            }
        }
    }
}

bool LoadPngRgba(const char* path, RgbaImage& out) {
    out = RgbaImage{};
    if (path == nullptr || path[0] == '\0') return false;
    int w = 0, h = 0, channels = 0;
    stbi_uc* pixels = stbi_load(path, &w, &h, &channels, 4);
    if (pixels == nullptr) return false;
    out.rgba.assign(pixels, pixels + static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
    out.width = static_cast<uint32_t>(w);
    out.height = static_cast<uint32_t>(h);
    stbi_image_free(pixels);
    return true;
}

bool CompositeAttribution(uint8_t* rgb, uint32_t width, uint32_t height, const RgbaImage& logo,
                          const char* text, HudRgb text_rgb, float scale, const char* font_path) {
    if (rgb == nullptr || width == 0 || height == 0) return false;
    constexpr int kMargin = 8;
    const int logo_top = static_cast<int>(height) - kMargin - static_cast<int>(logo.height);
    if (!logo.rgba.empty()) {
        DrawImageRgba(rgb, width, height, logo.rgba.data(), logo.width, logo.height, kMargin,
                      logo_top);
    }
    if (text == nullptr || font_path == nullptr || font_path[0] == '\0') return false;
    const FontAtlas* atlas = get_atlas(font_path, scale > 0.0f ? scale : 1.0f);
    if (atlas == nullptr) return false;
    const std::string line(text);
    float ink_bottom = 0.0f;
    const float text_width =
        draw_line(nullptr, 0, 0, *atlas, line, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &ink_bottom);
    const float x = static_cast<float>(width) - kMargin - text_width;
    float y = static_cast<float>(height) - kMargin - ink_bottom;
    if (!logo.rgba.empty() && x < static_cast<float>(kMargin + logo.width + kMargin)) {
        y = static_cast<float>(logo_top - kMargin) - ink_bottom;
    }
    // ponytail: two narrow-frame limits. Text wider than the frame between the margins (output
    // narrower than 16 px plus the text's advance: 541 px at HUD scale 1.0) keeps its start and
    // loses its end, "Improve this map"; and the lifted line leaves through the top of a frame
    // shorter than 86 px at HUD scale 1.0 (8 + 46 + 8 + 24 with the shipped logo). Stack two
    // right-aligned lines above the logo if outputs that small ship.
    draw_line(rgb, width, height, *atlas, line, std::max(x, static_cast<float>(kMargin)), y,
              text_rgb.r, text_rgb.g, text_rgb.b);
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

}
