/** @file hud_overlay.cpp
 *  @brief See hud_overlay.hpp. The ONE .cpp defining
 *  STB_TRUETYPE_IMPLEMENTATION (Step 1) -- stb_truetype.h itself is
 *  header-only and otherwise declaration-only.
 */
#include "micropilot_visualization_node/hud_overlay.hpp"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace mpviz_node
{

void PopulateHud(mpviz::SceneGraph& scene, int active_mode)
{
    scene.hud.speed_mps = scene.ego.speed_mps;
    scene.hud.active_mode = static_cast<uint8_t>(active_mode);
}

namespace
{

// Single-channel (coverage) atlas, ASCII ' '(32) through '~'(126).
constexpr int kAtlasDim = 512;
constexpr int kFirstChar = 32;
constexpr int kNumChars = 95;
// Baked once per distinct (font_path, scale) pair (see get_atlas() below),
// at this pixel height times `scale` -- baking bigger text for a bigger
// hud.scale rather than upscaling a fixed-size bake, so glyphs stay crisp
// at any theme's scale.
constexpr float kBasePixelHeight = 32.0f;

struct FontAtlas
{
    std::string path;
    float scale = 0.0f;
    bool ok = false;
    float pixel_height = 0.0f;
    std::vector<unsigned char> bitmap;  // kAtlasDim*kAtlasDim
    std::vector<stbtt_bakedchar> chars;
};

bool bake(const std::string& path, float pixel_height, FontAtlas& atlas)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    const std::vector<unsigned char> ttf((std::istreambuf_iterator<char>(in)),
                                          std::istreambuf_iterator<char>());
    if (ttf.empty()) return false;

    atlas.bitmap.assign(static_cast<size_t>(kAtlasDim) * kAtlasDim, 0);
    atlas.chars.assign(static_cast<size_t>(kNumChars), stbtt_bakedchar{});
    // Returns the number of rows actually used (>0 on success), or a
    // negative count if the atlas was too small to fit every glyph --
    // either way, <=0 is the failure case (stb_truetype.h's own contract).
    const int rows_used =
        stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, atlas.bitmap.data(), kAtlasDim,
                              kAtlasDim, kFirstChar, kNumChars, atlas.chars.data());
    return rows_used > 0;
}

// Cached across calls, keyed by (font_path, scale) -- bakes once per
// distinct pair, not per-frame (Step 3's "at on_configure()-adjacent load
// time" requirement), without a second node-lifecycle hook: in the running
// node both are effectively constant (hud_font_path_ param, active theme's
// hud.scale, which only changes on a set_theme() switch, not every tick).
// ponytail: one-entry cache, not an LRU keyed on every path ever seen --
// this node loads exactly one font for its whole lifetime; revisit if a
// future feature swaps fonts at runtime.
const FontAtlas* get_atlas(const std::string& font_path, float scale)
{
    // Scale is quantized to 0.1 steps for the cache test: a theme_transition
    // between two themes with DIFFERENT hud.scale values blends the scale
    // every tick, and an exact-float key would re-bake the whole atlas every
    // frame for the length of the transition (review 2026-09-09). Bounded to
    // at most one re-bake per 0.1 of scale instead; glyphs still bake at the
    // quantized height, so text stays crisp.
    const float qscale = std::round(scale * 10.0f) / 10.0f;
    static FontAtlas atlas;
    if (atlas.path != font_path || atlas.scale != qscale)
    {
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
                  float b, float coverage)
{
    if (x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height)) return;
    if (coverage <= 0.0f) return;
    uint8_t* px = rgb + (static_cast<size_t>(y) * width + x) * 3;
    px[0] = static_cast<uint8_t>(px[0] * (1.0f - coverage) + r * 255.0f * coverage);
    px[1] = static_cast<uint8_t>(px[1] * (1.0f - coverage) + g * 255.0f * coverage);
    px[2] = static_cast<uint8_t>(px[2] * (1.0f - coverage) + b * 255.0f * coverage);
}

// Draws one baked-atlas line at pen position (x, y) (y = baseline, stb's
// own convention), alpha-blending each glyph's coverage over whatever's
// already in `rgb`. Advances and returns the pen's final x -- unused by
// CompositeHud() today (each line starts at a fresh x), kept because
// stbtt_GetBakedQuad's pen-advance is otherwise silently discarded.
float draw_line(uint8_t* rgb, uint32_t width, uint32_t height, const FontAtlas& atlas,
                 const std::string& text, float x, float y, float r, float g, float b)
{
    for (unsigned char c : text)
    {
        if (c < kFirstChar || c >= kFirstChar + kNumChars)
        {
            x += atlas.pixel_height * 0.5f;  // unbaked char (e.g. control byte) -- blank advance
            continue;
        }
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(atlas.chars.data(), kAtlasDim, kAtlasDim, c - kFirstChar, &x, &y, &q,
                            1 /*opengl_fillrule -- top-left origin, matches rgb's own row-major*/);
        const int gx0 = static_cast<int>(std::floor(q.x0));
        const int gx1 = static_cast<int>(std::ceil(q.x1));
        const int gy0 = static_cast<int>(std::floor(q.y0));
        const int gy1 = static_cast<int>(std::ceil(q.y1));
        if (gx1 <= gx0 || gy1 <= gy0) continue;  // whitespace glyph -- nothing to blit
        const float u_span = q.s1 - q.s0, v_span = q.t1 - q.t0;
        const float gw = static_cast<float>(gx1 - gx0), gh = static_cast<float>(gy1 - gy0);
        for (int py = gy0; py < gy1; ++py)
        {
            const float v = q.t0 + ((py - gy0) + 0.5f) / gh * v_span;
            const int ay = static_cast<int>(v * kAtlasDim);
            if (ay < 0 || ay >= kAtlasDim) continue;
            for (int px = gx0; px < gx1; ++px)
            {
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
                   HudRgb text_rgb, HudRgb accent_rgb, float scale, const char* font_path)
{
    if (rgb == nullptr || width == 0 || height == 0 || font_path == nullptr ||
        font_path[0] == '\0')
    {
        return false;
    }
    const float safe_scale = scale > 0.0f ? scale : 1.0f;
    const FontAtlas* atlas = get_atlas(font_path, safe_scale);
    if (atlas == nullptr) return false;  // unreadable/unbakeable font -- non-fatal, rgb untouched

    char speed_buf[32];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.1f m/s", hud.speed_mps);
    // Mode NAMES, not numbers (user directive 2026-09-09: "Mode is shown as
    // number not a string!") -- same labels tools/vcam_gui.py's own mode
    // toggle uses ({1: bowl, 2: pointcloud, 3: visual}); an id neither knows
    // falls back to the numeric form rather than rendering nothing.
    char mode_buf[16];
    switch (hud.active_mode)
    {
        case 1: std::snprintf(mode_buf, sizeof(mode_buf), "BOWL"); break;
        case 2: std::snprintf(mode_buf, sizeof(mode_buf), "POINTCLOUD"); break;
        case 3: std::snprintf(mode_buf, sizeof(mode_buf), "VISUAL"); break;
        default:
            std::snprintf(mode_buf, sizeof(mode_buf), "MODE %d", static_cast<int>(hud.active_mode));
            break;
    }

    // Top-left chip stack: speed (text_rgb) above the active-mode indicator
    // (accent_rgb), each line's baseline one pixel_height + a fixed gap
    // below the previous.
    constexpr float kMarginX = 24.0f, kMarginY = 8.0f, kLineGap = 8.0f;
    float x = kMarginX, y = kMarginY + atlas->pixel_height;
    draw_line(rgb, width, height, *atlas, speed_buf, x, y, text_rgb.r, text_rgb.g, text_rgb.b);

    float x2 = kMarginX, y2 = y + atlas->pixel_height + kLineGap;
    draw_line(rgb, width, height, *atlas, mode_buf, x2, y2, accent_rgb.r, accent_rgb.g,
              accent_rgb.b);
    return true;
}

}  // namespace mpviz_node
