// theme.hpp — internal-only (not installed, not POD). YAML -> Theme token
// set consumed generically by renderer.cpp; no per-theme branch anywhere in
// the renderer. std:: usage is fine here — it's a `.hpp` under `src/`,
// never crosses the api.h/scene.h POD boundary.
//
// Color-space note: every color field below is authored in linear space in
// the YAML files and consumed as-is by materialParams/setFogOptions/
// IndirectLight::Builder — no sRGB decode happens anywhere in this loader.
// This matches Filament's own convention that baseColor/light/fog colors
// are linear, and is consistent with sun/ibl `intensity` already being
// physical units (lux), not colors.
#pragma once

#include <optional>
#include <string>

namespace mpviz::detail {

struct Float3 {
    float r = 0.0f, g = 0.0f, b = 0.0f;
};

// Mirrors assets/themes/*.yaml 1:1 — every field here is consumed
// generically by renderer.cpp, so both shipped themes (and any future one)
// must populate the same key set; see the YAML files' own header comments.
struct Theme {
    std::string name;

    struct Palette {
        Float3 ground;
        Float3 sky;
        Float3 fog;
        Float3 lane_paint;
        Float3 ribbon_core;
        Float3 ribbon_glow;
        // Ego contrast color: not reused from `ground` -- rendering the
        // ego with groundMaterial directly would blend it into the ground
        // plane it stands on. Deliberately a cross-theme swap
        // (dark_adas.yaml's `ego` is light_clay's `ground` and vice versa,
        // see those files' own comments), not a new color invented from
        // scratch, so the ego always pops against whichever ground it's
        // standing on. Optional key (see theme.cpp's parse()): absence
        // doesn't invalidate an otherwise-valid theme file, unlike every
        // other palette.* field above.
        Float3 ego;
        // ribbon_global/ribbon_local: dedicated tokens for the GLOBAL/LOCAL
        // path-ribbon roles, which used to reuse ribbon_core/ribbon_glow
        // (the BEHAVIOR/hero ribbon's own tokens). Soft-defaulted exactly
        // like `ego` above (theme.cpp's parse()): a theme file missing
        // either key still parses, falling back to the reused-token look
        // (ribbon_core for global, ribbon_glow for local).
        Float3 ribbon_global;
        Float3 ribbon_local;
        // road/lane_centerline/lane_boundary/crosswalk: soft-defaulted
        // exactly like `ego`/`ribbon_global`/`ribbon_local` above
        // (theme.cpp's parse()), so a theme file predating these still
        // parses -- lane_centerline/lane_boundary/crosswalk fall back to
        // `lane_paint`, `road` falls back to `ground`. Both shipped themes
        // author `crosswalk` explicitly (2026-09-09, color-distinct from
        // boundaries); the soft-defaults exist for a third-party theme file.
        Float3 road;
        Float3 lane_centerline;
        Float3 lane_boundary;
        Float3 crosswalk;
        // road_edge: the road's outer boundary (MapKind::ROAD_EDGE) renders
        // solid and yellow-family, distinct from the dashed-white interior
        // `lane_boundary` tone above. Soft-defaulted like every other
        // token in this block (theme.cpp's parse()) -- falls back to
        // `lane_paint`, so a theme file predating this key still parses.
        // Neither shipped theme relies on the default.
        Float3 road_edge;
        // building: baked-environment clay buildings (VM-052, STANDING
        // directive 2026-09-09: every rendered element gets a style token +
        // a disable knob -- environment_enabled, the node-side disable
        // half). Soft-defaulted like every other token in this block
        // (theme.cpp's parse()) -- falls back to `road` -- but both shipped
        // themes author an explicit value (Decision 10) so the reference
        // images' value separation actually shows.
        Float3 building;
        struct ObjectTints {
            Float3 car, truck_van, bus, pedestrian, cyclist, unknown;
        } object_tints;
        struct Alert {
            Float3 info, warning, critical;
        } alert;
    } palette;

    struct Material {
        float roughness = 0.0f;
        float metallic = 0.0f;
    } material;

    struct Emissive {
        float ribbon_strength = 0.0f;
    } emissive;

    struct Grid {
        Float3 line_color;
        float fade_start_m = 0.0f;
        float fade_end_m = 0.0f;
    } grid;

    struct Hud {
        Float3 text_color;
        Float3 accent_color;
        float scale = 1.0f;
    } hud;

    // point_cloud.point_size_px (2026-09-09, "I can't see any point cloud!"):
    // gl_PointSize for the POINTS primitive -- 1px default was invisible at
    // 720p. Soft-defaulted (palette.ego convention); STANDING-directive style
    // token for the point-cloud layer.
    struct PointCloudStyle {
        float point_size_px = 2.0f;  // user-tuned 2026-09-09 ("Points are large, make default is 2px")
    } point_cloud;

    struct Sun {
        Float3 direction;
        Float3 color;
        float intensity = 0.0f;
    } sun;

    struct Ibl {
        Float3 sky_color;
        Float3 ground_color;
        float intensity = 0.0f;
    } ibl;

    struct Fog {
        float density = 0.0f;
    } fog;

    // Ribbon geometry config. width_m is the extruded strip's full width
    // (not half-width) -- soft-defaulted (theme.cpp's parse()) to 0.24.
    // It's no longer read by ribbon.cpp's geometry directly; it now serves
    // only as the soft-default seed for the three margin fields below
    // (theme.cpp's parse() states the exact arithmetic), so a theme YAML
    // that only sets width_m reproduces an identical rendered strip.
    //
    // lane_width_m/margin_{behavior,global,local}_m: each role extrudes at
    // (lane_width_m - 2*margin_role)/2 half-width (clamped to the 0.12m
    // floor, ribbon.cpp's kRibbonMinHalfWidthM), so a lower ribbon peeks
    // out as a colored rim around a narrower one stacked above it (the
    // per-role z-stagger).
    // margin_velocity_m: the velocity-profile ribbon (VM-077 carpet-as-ribbon
    // redirect, 2026-09-10, user directive -- "I still prefer to treat it as
    // [a] ribbon that can be stacked on top of local ribbon with margin").
    // Soft-defaulted to 1.05 -- deliberately BETWEEN margin_local_m (0.8,
    // dark_adas.yaml) and margin_behavior_m (1.3): the user asked for a
    // strip narrower than LOCAL (so LOCAL's own color rim stays visible
    // under it) but wider than BEHAVIOR (so the hero ribbon's rim shows
    // through THIS one, not the other way around) -- see ribbon.cpp's
    // kRibbonZLiftByRoleM-style stagger doc and trajectory_carpet.cpp's own
    // z-slot constant for the matching z placement.
    struct Ribbon {
        float width_m = 0.24f;
        float lane_width_m = 3.5f;
        float margin_behavior_m = 1.63f;  // (lane_width_m - width_m) / 2 default -- see theme.cpp
        float margin_global_m = 1.63f;
        float margin_local_m = 1.63f;
        float margin_velocity_m = 1.05f;
    } ribbon;
};

// Loads `dir/<name>.yaml`. Returns std::nullopt (never throws) on any
// failure: dir null/missing/unreadable, the file missing, or malformed
// YAML/missing keys — the caller (create_renderer, Step 7b) decides the
// non-fatal fallback; this function itself has no fallback behavior.
std::optional<Theme> load_theme(const std::string& dir, const std::string& name);

// Compiled-in fallback (Step 7b) — same values as dark_adas.yaml, never read
// from disk. Used by create_renderer() whenever load_theme() fails, so a
// broken/missing theme-asset install never takes rendering down (spec §9).
const Theme& kFallbackTheme();

}  // namespace mpviz::detail
