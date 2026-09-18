// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file hud_overlay.hpp
 *  @brief Epic 3 Task 3 (VM-030): the node-side CPU HUD compositor -- the
 *  accepted P2 alternative to an SDF atlas + Filament overlay pass inside
 *  overlume (no font pipeline in the clang/libc++ archive; see the
 *  plan's own P2 citation, Epic 3 plan Task 3 section).
 *
 *  Two pieces:
 *   - `PopulateHud()`: copies `scene.ego.speed_mps`/the node's own
 *     the node's `render_mode_` into `scene.hud` (Step 0). A free function, not
 *     inlined at the `timer_callback()` call site, purely so it has a
 *     unit-testable seam without standing up a full OverlumeNode/
 *     rclcpp/tf2 harness -- the plan's own escape valve for Step 0's test
 *     ("a focused unit test against a hand-built SceneAssembly/Ego pair").
 *     `Hud::chips`/`chip_count` are deliberately left untouched (zero-init
 *     default) -- Task 4 (VM-031) scope.
 *   - `CompositeHud()`: bakes a stb_truetype font atlas ONCE per distinct
 *     (font_path, scale) pair (cached in a function-local static, keyed so
 *     re-baking never happens per-frame in the running node, where both are
 *     effectively constant) and blits the speed + active-mode text directly
 *     into an RGB8 frame buffer, in place. Colors come from
 *     `overlume::get_hud_colors()` (scene.h) -- the live, possibly
 *     mid-`theme_transition` theme HUD colors -- scaled by that same
 *     theme's `hud.scale`.
 *
 *  Non-fatal on a missing/unloadable font (spec §9, same philosophy as
 *  `set_ego_model()`'s clay-box fallback): `CompositeHud()` returns false
 *  and leaves `rgb` byte-for-byte unchanged. It never logs itself (pure,
 *  same "no ROS node, no clock" shape as diagnostics.hpp) -- the caller
 *  (overlume_node.cpp) WARNs once on a false return.
 */

#include <cstdint>

#include "overlume/scene.h"

namespace overlume::ros {

// Step 0: scene.hud.speed_mps + the render mode only -- chips are Task 4 scope.
void PopulateHud(overlume::SceneGraph& scene, int render_mode);

// A plain RGB triplet, 0..1 float -- not overlume::Vec3 (that's doubles, a
// world-space point type); not std::array (this header is node-only, no
// POD-boundary constraint, but there's no reason to reach for more than an
// aggregate here).
struct HudRgb {
    float r, g, b;
};

// Node-side mirror of overlume::Hud's two Task-3 fields (speed_mps/
// active_mode) -- deliberately NOT overlume::Hud itself: that struct also
// carries chips/chip_count (Task 4), which CompositeHud() has no use for
// yet, and a caller building one of these for a test needn't zero-init
// fields it doesn't care about.
struct HudSnapshot {
    double speed_mps;
    uint8_t active_mode;
};

// Composites a two-line HUD ("<speed> m/s" in `text_rgb`, "MODE <n>" in
// `accent_rgb`) onto `rgb` (width*height*3, RGB8, row-major) at the given
// `scale` (theme hud.scale). Returns false -- `rgb` left byte-for-byte
// unchanged -- if `font_path` can't be read or baked; true otherwise.
// `width`/`height` must match `rgb`'s actual allocation; a null `rgb` or
// `font_path`, or zero `width`/`height`, is also a (harmless) false return.
bool CompositeHud(uint8_t* rgb, uint32_t width, uint32_t height, const HudSnapshot& hud,
                  HudRgb text_rgb, HudRgb accent_rgb, float scale, const char* font_path);

// Epic 3 Task 4 (VM-031): exposed so callouts.cpp can draw its chip/leader
// line through the SAME font-atlas cache and glyph-blit path CompositeHud()
// already uses -- extension, not a second compositor (plan's own
// instruction). Draws one line of `text` with its baseline at (x, y) (stb's
// own pen-position convention -- same as CompositeHud()'s two lines),
// alpha-blended over whatever's already in `rgb`. False (rgb untouched) on
// the same missing/unloadable-font/null/zero-size conditions as
// CompositeHud(); true otherwise.
bool DrawText(uint8_t* rgb, uint32_t width, uint32_t height, const char* text, float x, float y,
              HudRgb rgb_color, float scale, const char* font_path);

// A callout's leader line: solid, alpha-blended, 1px-wide, Bresenham-drawn
// from (x0, y0) to (x1, y1). Pure raster, no font/atlas involved -- unlike
// DrawText() this can't fail (out-of-bounds endpoints are simply clipped
// pixel-by-pixel by the same bounds check blend_pixel() already applies).
void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color);

}  // namespace overlume::ros
