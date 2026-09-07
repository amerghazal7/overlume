// theme_transition.cpp — Epic 1 Task 3 (VM-014). See theme_transition.hpp.
#include "theme_transition.hpp"

#include <algorithm>
#include <cmath>

namespace mpviz::detail {
namespace {

float lerpf(float a, float b, float w) { return a + (b - a) * w; }

Float3 lerp_vec3(const Float3& a, const Float3& b, float w) {
    return Float3{lerpf(a.r, b.r, w), lerpf(a.g, b.g, w), lerpf(a.b, b.b, w)};
}

// Geometric (log-space) lerp for PHOTOMETRIC INTENSITY SCALARS ONLY
// (sun.intensity, ibl.intensity) -- review finding (Epic1 Task2/3 gate,
// MAJOR 1). Illumination x albedo is a product: a plain linear lerp of two
// values ~19-29x apart (dark_adas 480000/256000 lux -> light_clay
// 25000/8750 lux) blended against palette albedo brightening at the same
// time overshoots BOTH endpoints around w~0.5-0.65 (measured: frame-mean
// luminance 43 -> peak ~205 -> settles ~175 under the old linear lerp). A
// geometric lerp (out = a * pow(b/a, w)) is monotonic between the two
// endpoints by construction -- it can't overshoot either one. Falls back to
// a linear lerp if either endpoint is <= 0 (log/pow undefined there); every
// shipped theme's intensities are positive physical lux, so this is a
// defensive guard, not a code path either theme actually takes.
float lerpf_geometric(float a, float b, float w) {
    if (a <= 0.0f || b <= 0.0f) return lerpf(a, b, w);
    return a * std::pow(b / a, w);
}

}  // namespace

Oklab linear_srgb_to_oklab(const Float3& c) {
    const float l = 0.4122214708f * c.r + 0.5363325363f * c.g + 0.0514459929f * c.b;
    const float m = 0.2119034982f * c.r + 0.6806995451f * c.g + 0.1073969566f * c.b;
    const float s = 0.0883024619f * c.r + 0.2817188376f * c.g + 0.6299787005f * c.b;

    // cbrtf is the real (signed) cube root -- correct for the occasional
    // out-of-[0,1] or negative component an authored theme color can have
    // transiently during LMS projection, unlike a pow(x, 1/3) that would
    // NaN on negative input.
    const float l_ = std::cbrt(l);
    const float m_ = std::cbrt(m);
    const float s_ = std::cbrt(s);

    return Oklab{
        0.2104542553f * l_ + 0.7936177850f * m_ - 0.0040720468f * s_,
        1.9779984951f * l_ - 2.4285922050f * m_ + 0.4505937099f * s_,
        0.0259040371f * l_ + 0.7827717662f * m_ - 0.8086757660f * s_,
    };
}

Float3 oklab_to_linear_srgb(const Oklab& c) {
    const float l_ = c.L + 0.3963377774f * c.a + 0.2158037573f * c.b;
    const float m_ = c.L - 0.1055613458f * c.a - 0.0638541728f * c.b;
    const float s_ = c.L - 0.0894841775f * c.a - 1.2914855480f * c.b;

    const float l = l_ * l_ * l_;
    const float m = m_ * m_ * m_;
    const float s = s_ * s_ * s_;

    return Float3{
        +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
        -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
        -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s,
    };
}

float smoothstep01(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

Float3 blend_color(const Float3& a, const Float3& b, float t) {
    const Oklab la = linear_srgb_to_oklab(a);
    const Oklab lb = linear_srgb_to_oklab(b);
    const Oklab mid{lerpf(la.L, lb.L, t), lerpf(la.a, lb.a, t), lerpf(la.b, lb.b, t)};
    return oklab_to_linear_srgb(mid);
}

Theme blend(const Theme& a, const Theme& b, float t) {
    const float w = smoothstep01(t);
    Theme out;
    out.name = w >= 1.0f ? b.name : a.name;

    out.palette.ground = blend_color(a.palette.ground, b.palette.ground, w);
    out.palette.sky = blend_color(a.palette.sky, b.palette.sky, w);
    out.palette.fog = blend_color(a.palette.fog, b.palette.fog, w);
    out.palette.lane_paint = blend_color(a.palette.lane_paint, b.palette.lane_paint, w);
    out.palette.ribbon_core = blend_color(a.palette.ribbon_core, b.palette.ribbon_core, w);
    out.palette.ribbon_glow = blend_color(a.palette.ribbon_glow, b.palette.ribbon_glow, w);
    out.palette.ego = blend_color(a.palette.ego, b.palette.ego, w);
    out.palette.ribbon_global =
        blend_color(a.palette.ribbon_global, b.palette.ribbon_global, w);
    out.palette.ribbon_local = blend_color(a.palette.ribbon_local, b.palette.ribbon_local, w);
    out.palette.object_tints.car =
        blend_color(a.palette.object_tints.car, b.palette.object_tints.car, w);
    out.palette.object_tints.truck_van =
        blend_color(a.palette.object_tints.truck_van, b.palette.object_tints.truck_van, w);
    out.palette.object_tints.bus =
        blend_color(a.palette.object_tints.bus, b.palette.object_tints.bus, w);
    out.palette.object_tints.pedestrian =
        blend_color(a.palette.object_tints.pedestrian, b.palette.object_tints.pedestrian, w);
    out.palette.object_tints.cyclist =
        blend_color(a.palette.object_tints.cyclist, b.palette.object_tints.cyclist, w);
    out.palette.object_tints.unknown =
        blend_color(a.palette.object_tints.unknown, b.palette.object_tints.unknown, w);
    out.palette.alert.info = blend_color(a.palette.alert.info, b.palette.alert.info, w);
    out.palette.alert.warning = blend_color(a.palette.alert.warning, b.palette.alert.warning, w);
    out.palette.alert.critical =
        blend_color(a.palette.alert.critical, b.palette.alert.critical, w);

    out.material.roughness = lerpf(a.material.roughness, b.material.roughness, w);
    out.material.metallic = lerpf(a.material.metallic, b.material.metallic, w);

    out.emissive.ribbon_strength =
        lerpf(a.emissive.ribbon_strength, b.emissive.ribbon_strength, w);

    out.grid.line_color = blend_color(a.grid.line_color, b.grid.line_color, w);
    // grid.fade_start_m/fade_end_m are NOT animated here: they're baked into
    // the grid vertex buffer's per-vertex alpha once, at create_renderer()
    // time (renderer.cpp's build_grid_lines()), and push_theme_to_scene()
    // never re-reads them from the blended Theme. Carry `b`'s (the "to"
    // theme's) values through unchanged rather than lerping toward a value
    // set_theme() can't actually apply mid-transition.
    out.grid.fade_start_m = b.grid.fade_start_m;
    out.grid.fade_end_m = b.grid.fade_end_m;

    out.hud.text_color = blend_color(a.hud.text_color, b.hud.text_color, w);
    out.hud.accent_color = blend_color(a.hud.accent_color, b.hud.accent_color, w);
    out.hud.scale = lerpf(a.hud.scale, b.hud.scale, w);

    // sun.direction: NOT a color -- see this header's own comment above
    // blend()'s declaration. Plain vector lerp, un-normalized (matches the
    // existing static-theme convention already in renderer.cpp).
    out.sun.direction = lerp_vec3(a.sun.direction, b.sun.direction, w);
    out.sun.color = blend_color(a.sun.color, b.sun.color, w);
    out.sun.intensity = lerpf_geometric(a.sun.intensity, b.sun.intensity, w);

    out.ibl.sky_color = blend_color(a.ibl.sky_color, b.ibl.sky_color, w);
    out.ibl.ground_color = blend_color(a.ibl.ground_color, b.ibl.ground_color, w);
    out.ibl.intensity = lerpf_geometric(a.ibl.intensity, b.ibl.intensity, w);

    out.fog.density = lerpf(a.fog.density, b.fog.density, w);

    // ribbon.width_m (user directive 2026-08-20, ITEM 1): a plain scalar
    // lerp, same as roughness/metallic/hud.scale above -- not a color, no
    // Oklab involved.
    out.ribbon.width_m = lerpf(a.ribbon.width_m, b.ribbon.width_m, w);

    return out;
}

}  // namespace mpviz::detail
