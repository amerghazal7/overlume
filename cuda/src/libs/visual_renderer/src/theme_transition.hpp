// theme_transition.hpp — Epic 1 Task 3 (VM-014): animated theme toggle.
// Internal-only (not installed, not POD) — same rules as theme.hpp: ordinary
// std:: usage is fine here, nothing here crosses the api.h/scene.h POD
// boundary.
#pragma once

#include "theme.hpp"

namespace mpviz::detail {

// sRGB<->Oklab conversions (Björn Ottosson's published reference formulas —
// https://bottosson.github.io/posts/oklab/, public domain). Theme colors are
// already authored in LINEAR space (see theme.hpp's header comment), which
// is exactly the input domain these matrices expect — no gamma decode step
// needed before/after.
struct Oklab {
    float L = 0.0f, a = 0.0f, b = 0.0f;
};

Oklab linear_srgb_to_oklab(const Float3& c);
Float3 oklab_to_linear_srgb(const Oklab& c);

// Cubic smoothstep, clamped to [0,1] outside that range: 3t^2 - 2t^3.
float smoothstep01(float t);

// Perceptual color blend: lerps `a`/`b` in Oklab space by `t` (already the
// EASED weight — callers pass smoothstep01(raw_t), not raw_t; see blend()
// below) and converts back to linear sRGB.
Float3 blend_color(const Float3& a, const Float3& b, float t);

// Blends every themed token from `a` toward `b`: `w = smoothstep01(t)`;
// every float3 PALETTE/grid/hud color -> blend_color(w); every scalar
// (roughness, metallic, ribbon_strength, hud scale, sun/ibl intensity, fog
// density) -> linear lerp by w. One function, no per-field branch beyond
// "is this a color or a scalar" (Task 3 Step 3).
//
// Exception: grid.fade_start_m/fade_end_m are carried through from `b`
// unchanged, NOT lerped -- they're baked into the grid vertex buffer once at
// create_renderer() time (renderer.cpp's build_grid_lines()) and are not
// re-read from the blended Theme by push_theme_to_scene(), so a lerp here
// would be dead code. See blend()'s definition for the full comment.
//
// Deliberate exception, not covered by that color/scalar split:
// `sun.direction` is a float3 but is NOT a color — it's a world-space
// direction vector. Running it through the Oklab color matrices (calibrated
// for physically-plausible ~[0,1] linear-sRGB tristimulus values, not
// arbitrary signed direction components) would produce a numerically well-
// defined but photometrically meaningless result. It gets a plain
// component-wise linear lerp instead, un-normalized — matching the existing
// static-theme convention already in renderer.cpp, where
// theme.sun.direction (e.g. dark_adas's {-0.5,-0.3,-1.0}, magnitude ~1.157)
// is fed to LightManager::Builder::direction()/setDirection() as-is, never
// renormalized, and that already works.
Theme blend(const Theme& a, const Theme& b, float t);

// Per-VisualRenderer transition state (Task 3 Step 6): `from` is a snapshot
// of the currently-blended theme at the moment set_theme() was called (so a
// mid-flight retarget starts from the current blend, not either endpoint —
// see set_theme()'s frozen contract in scene.h), `to` is the freshly loaded
// target theme, and `start_sec`/`duration_sec` key the render_frame()-side
// `t = clamp((sim_time_sec - start_sec) / duration_sec, 0, 1)` clock (the
// caller's sim clock, never wall-clock — scene.h's SceneGraph::sim_time_sec
// contract).
struct ThemeTransition {
    Theme from;
    Theme to;
    double start_sec = 0.0;
    double duration_sec = 0.8;
};

}  // namespace mpviz::detail
