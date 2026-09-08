// theme.hpp — internal-only (not installed, not POD). YAML -> Theme token
// set consumed generically by renderer.cpp; no per-theme branch anywhere in
// the renderer (Epic 1 Task 2 / VM-011). std:: usage is fine here — it's a
// `.hpp` under `src/`, never crosses the api.h/scene.h POD boundary.
//
// Color-space note (stated once, here, per
// docs/superpowers/plans/2026-08-18-visual-mode-epic1.md Task 2 Step 7a):
// every color field below is authored in LINEAR space in the YAML files and
// consumed as-is by materialParams/setFogOptions/IndirectLight::Builder —
// no sRGB decode happens anywhere in this loader. This matches Filament's
// own convention that baseColor/light/fog colors are linear, and is
// consistent with sun/ibl `intensity` already being physical units (lux),
// not colors.
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
        // Ego contrast color (user directive 2026-08-20): NOT reused from
        // `ground` — the ego used to bind groundMaterial directly and so
        // rendered as palette.ground, blending into the ground plane it
        // stands on in both shipped themes. This is deliberately a
        // CROSS-theme swap (dark_adas.yaml's `ego` is light_clay's `ground`
        // and vice versa, see those files' own comments), not a new color
        // invented from scratch, so the ego always pops against whichever
        // ground it's standing on. Optional key (see theme.cpp's parse()):
        // absence doesn't invalidate an otherwise-valid theme file, unlike
        // every other palette.* field above.
        Float3 ego;
        // ribbon_global/ribbon_local (user directive 2026-08-20, ITEM 1):
        // GLOBAL/LOCAL path-ribbon roles used to reuse ribbon_core/
        // ribbon_glow (the BEHAVIOR/hero ribbon's own tokens) -- an
        // authoring gap, not a code gap, per renderer.cpp's old comment.
        // These are the two dedicated tokens that close it. SOFT-DEFAULTED
        // exactly like `ego` just above (theme.cpp's parse()): a theme file
        // missing either key still parses, falling back to the value that
        // reproduces today's reused-token look (ribbon_core for global,
        // ribbon_glow for local) rather than invalidating the whole theme.
        Float3 ribbon_global;
        Float3 ribbon_local;
        // road/lane_centerline/lane_boundary/crosswalk (Epic 3 Task 1 /
        // VM-036, decision #6): SOFT-DEFAULTED exactly like `ego`/
        // `ribbon_global`/`ribbon_local` above (theme.cpp's parse()), so a
        // theme file predating these still parses -- lane_centerline/
        // lane_boundary/crosswalk fall back to `lane_paint`, `road` falls
        // back to `ground`. Neither shipped theme relies on the default
        // (both get explicit values, per decision #6's concrete hue
        // targets); the soft-default exists for a third-party theme file.
        Float3 road;
        Float3 lane_centerline;
        Float3 lane_boundary;
        Float3 crosswalk;
        // road_edge (user directive 2026-09-08, post Task 1 candidate
        // review): the road's outer boundary (MapKind::ROAD_EDGE -- reserved
        // since Task 1, first producer this directive adds in hd_map.cpp)
        // renders SOLID and yellow-family, distinct from the dashed-white
        // interior `lane_boundary` tone above. SOFT-DEFAULTED exactly like
        // every other token in this block (theme.cpp's parse()) -- falls
        // back to `lane_paint`, the same `palette.ego` precedent -- so a
        // theme file predating this key still parses. Neither shipped theme
        // relies on the default; both author an explicit yellow.
        Float3 road_edge;
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

    // Ribbon geometry config (user directive 2026-08-20, ITEM 1): width_m is
    // the extruded strip's FULL width (not half-width) -- SOFT-DEFAULTED
    // (theme.cpp's parse(), same convention as palette.ego/ribbon_global/
    // ribbon_local above) to 0.24, i.e. 2*kRibbonHalfWidthM, the constant
    // ribbon.cpp used to hard-code before this field existed -- a theme file
    // missing the whole `ribbon:` section (or just `width_m` in it) parses
    // unchanged, reproducing today's look exactly.
    struct Ribbon {
        float width_m = 0.24f;
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
