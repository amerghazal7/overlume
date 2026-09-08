// theme_transition.hpp — animated theme toggle. Internal-only (not
// installed, not POD) — same rules as theme.hpp: ordinary std:: usage is
// fine here, nothing here crosses the api.h/scene.h POD boundary.
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
// every float3 palette/grid/hud color -> blend_color(w); every scalar
// (roughness, metallic, ribbon_strength, hud scale, fog density) -> linear
// lerp by w. One function, no per-field branch beyond "is this a color or a
// scalar" -- except sun.intensity/ibl.intensity, which use a geometric
// (log-space) lerp instead of linear (see lerpf_geometric() in the .cpp for
// why: those two are physical lux values far apart between the shipped
// themes, and a linear lerp of illumination against simultaneously
// brightening albedo overshoots both endpoints).
//
// Exception: grid.fade_start_m/fade_end_m are carried through from `b`
// unchanged, not lerped -- see blend()'s definition for why.
//
// Deliberate exception, not covered by that color/scalar split:
// `sun.direction` is a float3 but is not a color -- it's a world-space
// direction vector. Running it through the Oklab color matrices (which
// assume ~[0,1] linear-sRGB tristimulus values) would be photometrically
// meaningless. It gets a plain component-wise linear lerp instead,
// un-normalized -- matching the existing static-theme convention in
// renderer.cpp, which feeds theme.sun.direction to
// LightManager::Builder::direction() as-is, never renormalized.
Theme blend(const Theme& a, const Theme& b, float t);

// Per-VisualRenderer transition state: `from` is a snapshot of the
// currently-blended theme at the moment set_theme() was called (so a
// mid-flight retarget starts from the current blend, not either endpoint —
// see set_theme()'s contract in scene.h), `to` is the freshly loaded target
// theme, and `start_sec`/`duration_sec` key the render_frame()-side
// `t = clamp((sim_time_sec - start_sec) / duration_sec, 0, 1)` clock (the
// caller's sim clock, never wall-clock).
struct ThemeTransition {
    Theme from;
    Theme to;
    double start_sec = 0.0;
    double duration_sec = 0.8;
};

}  // namespace mpviz::detail
