#include "theme.hpp"

#include <algorithm>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "visual_renderer/scene.h"

namespace mpviz::detail {
namespace {

Float3 to_float3(const YAML::Node& node) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error("theme: expected a 3-element sequence");
    }
    return Float3{node[0].as<float>(), node[1].as<float>(), node[2].as<float>()};
}

// Throws (caught by load_theme) on any missing/malformed key — most fields
// in the schema are required; a handful of newer palette/ribbon keys are
// soft-defaulted instead (see each field's own inline comment below).
Theme parse(const YAML::Node& root) {
    Theme t;
    t.name = root["name"].as<std::string>();

    const YAML::Node palette = root["palette"];
    t.palette.ground = to_float3(palette["ground"]);
    t.palette.sky = to_float3(palette["sky"]);
    t.palette.fog = to_float3(palette["fog"]);
    t.palette.lane_paint = to_float3(palette["lane_paint"]);
    t.palette.ribbon_core = to_float3(palette["ribbon_core"]);
    t.palette.ribbon_glow = to_float3(palette["ribbon_glow"]);
    // ego: soft-defaulted rather than thrown on absence, so theme YAMLs
    // written before this field existed keep parsing instead of falling
    // back to the whole compiled-in kFallbackTheme() over one missing key.
    // Default matches kFallbackTheme()'s own palette.ego below.
    t.palette.ego = palette["ego"] ? to_float3(palette["ego"]) : Float3{0.82f, 0.80f, 0.76f};

    // ribbon_global/ribbon_local: same soft-default convention as `ego`
    // above -- missing either reproduces the reused-token look (GLOBAL
    // falls back to ribbon_core, LOCAL to ribbon_glow) instead of falling
    // back to the whole compiled-in kFallbackTheme().
    t.palette.ribbon_global =
        palette["ribbon_global"] ? to_float3(palette["ribbon_global"]) : t.palette.ribbon_core;
    t.palette.ribbon_local =
        palette["ribbon_local"] ? to_float3(palette["ribbon_local"]) : t.palette.ribbon_glow;

    // road/lane_centerline/lane_boundary/crosswalk: same soft-default
    // convention as ribbon_global/ribbon_local above. `road` falls back to
    // `ground`; the other three fall back to `lane_paint`.
    t.palette.road = palette["road"] ? to_float3(palette["road"]) : t.palette.ground;
    t.palette.lane_centerline =
        palette["lane_centerline"] ? to_float3(palette["lane_centerline"]) : t.palette.lane_paint;
    t.palette.lane_boundary =
        palette["lane_boundary"] ? to_float3(palette["lane_boundary"]) : t.palette.lane_paint;
    t.palette.crosswalk =
        palette["crosswalk"] ? to_float3(palette["crosswalk"]) : t.palette.lane_paint;
    // road_edge: same soft-default convention, falls back to lane_paint --
    // neither shipped theme relies on it, both author an explicit yellow.
    t.palette.road_edge =
        palette["road_edge"] ? to_float3(palette["road_edge"]) : t.palette.lane_paint;

    // building: same soft-default convention as road_edge above -- falls
    // back to `road`; both shipped themes author an explicit value (Decision
    // 10, VM-052).
    t.palette.building = palette["building"] ? to_float3(palette["building"]) : t.palette.road;

    const YAML::Node tints = palette["object_tints"];
    t.palette.object_tints.car = to_float3(tints["car"]);
    t.palette.object_tints.truck_van = to_float3(tints["truck_van"]);
    t.palette.object_tints.bus = to_float3(tints["bus"]);
    t.palette.object_tints.pedestrian = to_float3(tints["pedestrian"]);
    t.palette.object_tints.cyclist = to_float3(tints["cyclist"]);
    t.palette.object_tints.unknown = to_float3(tints["unknown"]);

    const YAML::Node alert = palette["alert"];
    t.palette.alert.info = to_float3(alert["info"]);
    t.palette.alert.warning = to_float3(alert["warning"]);
    t.palette.alert.critical = to_float3(alert["critical"]);

    t.material.roughness = root["material"]["roughness"].as<float>();
    t.material.metallic = root["material"]["metallic"].as<float>();

    t.emissive.ribbon_strength = root["emissive"]["ribbon_strength"].as<float>();

    const YAML::Node grid = root["grid"];
    t.grid.line_color = to_float3(grid["line_color"]);
    t.grid.fade_start_m = grid["fade_start_m"].as<float>();
    t.grid.fade_end_m = grid["fade_end_m"].as<float>();

    const YAML::Node hud = root["hud"];
    t.hud.text_color = to_float3(hud["text_color"]);
    t.hud.accent_color = to_float3(hud["accent_color"]);
    t.hud.scale = hud["scale"].as<float>();

    // point_cloud: soft-defaulted (a YAML predating the token parses fine).
    const YAML::Node pc = root["point_cloud"];
    t.point_cloud.point_size_px = (pc && pc["point_size_px"])
                                       ? pc["point_size_px"].as<float>()
                                       : 2.0f;

    const YAML::Node sun = root["sun"];
    t.sun.direction = to_float3(sun["direction"]);
    t.sun.color = to_float3(sun["color"]);
    t.sun.intensity = sun["intensity"].as<float>();

    const YAML::Node ibl = root["ibl"];
    t.ibl.sky_color = to_float3(ibl["sky_color"]);
    t.ibl.ground_color = to_float3(ibl["ground_color"]);
    t.ibl.intensity = ibl["intensity"].as<float>();

    t.fog.density = root["fog"]["density"].as<float>();

    // ribbon.width_m: the whole `ribbon:` section, and width_m within it,
    // are optional -- soft-defaulted to Theme::Ribbon's own 0.24 default
    // (theme.hpp). No longer read directly by ribbon.cpp's geometry (see
    // below).
    const YAML::Node ribbon = root["ribbon"];
    t.ribbon.width_m = (ribbon && ribbon["width_m"]) ? ribbon["width_m"].as<float>() : 0.24f;

    // lane_width_m/margin_{behavior,global,local}_m: same soft-default
    // convention. lane_width_m defaults to 3.5m; each margin defaults to
    // (lane_width_m - width_m) / 2, using whatever lane_width_m/width_m
    // this theme actually parsed to above -- that formula reproduces a
    // pre-existing YAML's strip width to within one float ULP, not
    // byte-for-byte (the round-trip computes 0x3df5c290 vs the old
    // width_m * 0.5f's 0x3df5c28f, a 7.5e-9 m difference) -- do not assert
    // exact equality here, tests use EXPECT_NEAR. Neither shipped theme
    // relies on this default; it exists for a third-party theme file
    // predating this directive.
    t.ribbon.lane_width_m = (ribbon && ribbon["lane_width_m"]) ? ribbon["lane_width_m"].as<float>() : 3.5f;
    const float marginDefault = (t.ribbon.lane_width_m - t.ribbon.width_m) / 2.0f;
    t.ribbon.margin_behavior_m =
        (ribbon && ribbon["margin_behavior_m"]) ? ribbon["margin_behavior_m"].as<float>() : marginDefault;
    t.ribbon.margin_global_m =
        (ribbon && ribbon["margin_global_m"]) ? ribbon["margin_global_m"].as<float>() : marginDefault;
    t.ribbon.margin_local_m =
        (ribbon && ribbon["margin_local_m"]) ? ribbon["margin_local_m"].as<float>() : marginDefault;

    // margin_velocity_m: NOT derived from marginDefault (unlike the three
    // role margins above) -- it's a fixed 1.05 soft default, deliberately
    // between margin_local_m (0.8) and margin_behavior_m (1.3), independent
    // of whatever width_m/lane_width_m this theme authors (see theme.hpp's
    // own comment on why 1.05).
    t.ribbon.margin_velocity_m = (ribbon && ribbon["margin_velocity_m"])
                                     ? ribbon["margin_velocity_m"].as<float>()
                                     : 1.05f;

    // objects.opacity: soft-defaulted (VM-078), same convention as
    // point_cloud.point_size_px above -- a theme YAML predating this key
    // still parses, at the fully-opaque 1.0 default.
    const YAML::Node objects = root["objects"];
    t.objects.opacity =
        std::clamp(
        (objects && objects["opacity"]) ? objects["opacity"].as<float>() : 1.0f, 0.0f, 1.0f);
    // Clamped: >1 would keep alpha >= 1 and silently SUPPRESS the staleness
    // fade for most of its window; <0 would bind a negative baseColor alpha.

    return t;
}

}  // namespace

std::optional<Theme> load_theme(const std::string& dir, const std::string& name) {
    if (dir.empty() || name.empty()) return std::nullopt;
    const std::string path = dir + "/" + name + ".yaml";
    try {
        const YAML::Node root = YAML::LoadFile(path);
        return parse(root);
    } catch (const std::exception&) {
        // Missing dir/file, unreadable, malformed YAML, or a missing/
        // mistyped key inside parse() above -- all non-fatal; caller
        // (create_renderer) falls back to kFallbackTheme().
        return std::nullopt;
    }
}

const Theme& kFallbackTheme() {
    static const Theme theme = [] {
        // Same values as dark_adas.yaml, kept in code so a broken/missing
        // theme-asset install can never take rendering down.
        Theme t;
        t.name = "dark_adas";
        t.palette.ground = {0.05f, 0.06f, 0.08f};
        t.palette.sky = {0.02f, 0.02f, 0.05f};
        t.palette.fog = {0.02f, 0.02f, 0.05f};
        // Must match dark_adas.yaml exactly (ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml).
        t.palette.lane_paint = {0.85f, 0.85f, 0.88f};
        // Cold green; glow intentionally killed (no neon/high ribbon_strength).
        t.palette.ribbon_core = {0.12f, 0.55f, 0.42f};
        t.palette.ribbon_glow = {0.12f, 0.55f, 0.42f};
        // Cross-theme swap: light_clay's ground color, same value
        // dark_adas.yaml's `ego` key authors on disk -- see theme.hpp's
        // Palette::ego comment.
        t.palette.ego = {0.82f, 0.80f, 0.76f};
        // dark_adas.yaml's own authored values -- not a reuse of
        // ribbon_core/glow.
        t.palette.ribbon_global = {0.25f, 0.55f, 0.95f};
        t.palette.ribbon_local = {0.95f, 0.70f, 0.15f};
        // Must match dark_adas.yaml exactly (ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml).
        // road is darker than palette.ground; lane_boundary is the same
        // near-white as lane_paint; crosswalk is authored warm ivory
        // (distinct from boundaries by color, not just hatch geometry);
        // lane_centerline is a
        // low-contrast fade of lane_paint toward road (25%/75% -- faint dot
        // guidance, not a bold stroke); road_edge is a clear, saturated
        // road-paint yellow, solid and readable on the dark road.
        t.palette.road = {0.03f, 0.035f, 0.045f};
        t.palette.lane_centerline = {0.235f, 0.239f, 0.254f};
        t.palette.lane_boundary = {0.85f, 0.85f, 0.88f};
        t.palette.crosswalk = {0.95f, 0.90f, 0.70f};  // authored warm ivory, matches dark_adas.yaml
        t.palette.road_edge = {0.95f, 0.75f, 0.05f};
        // Must match dark_adas.yaml exactly, same convention as road/
        // lane_boundary/etc above (VM-052, Decision 10).
        t.palette.building = {0.06f, 0.07f, 0.11f};
        t.palette.object_tints.car = {0.25f, 0.35f, 0.9f};
        t.palette.object_tints.truck_van = {0.30f, 0.35f, 0.85f};
        t.palette.object_tints.bus = {0.85f, 0.6f, 0.15f};
        t.palette.object_tints.pedestrian = {0.9f, 0.2f, 0.2f};
        t.palette.object_tints.cyclist = {0.9f, 0.55f, 0.1f};
        t.palette.object_tints.unknown = {0.5f, 0.5f, 0.5f};
        t.palette.alert.info = {0.2f, 0.6f, 1.0f};
        t.palette.alert.warning = {1.0f, 0.7f, 0.1f};
        t.palette.alert.critical = {1.0f, 0.15f, 0.1f};
        t.material.roughness = 0.85f;
        t.material.metallic = 0.0f;
        t.emissive.ribbon_strength = 0.0f;  // glow killed, matches dark_adas.yaml
        t.grid.line_color = {0.12f, 0.14f, 0.18f};
        t.grid.fade_start_m = 15.0f;
        t.grid.fade_end_m = 40.0f;
        t.hud.text_color = {0.9f, 0.95f, 1.0f};
        t.hud.accent_color = {0.10f, 1.0f, 0.4f};
        t.hud.scale = 1.0f;
        t.point_cloud.point_size_px = 2.0f;
        t.sun.direction = {-0.5f, -0.3f, -1.0f};
        t.sun.color = {0.55f, 0.6f, 0.75f};
        t.sun.intensity = 480000.0f;
        t.ibl.sky_color = {0.05f, 0.06f, 0.12f};
        t.ibl.ground_color = {0.02f, 0.02f, 0.03f};
        t.ibl.intensity = 256000.0f;
        t.fog.density = 0.015f;
        // width_m is no longer authored on disk (dark_adas.yaml drops the
        // key) so it parses via the soft-default seed (0.24);
        // lane_width_m/margins are explicit, matching dark_adas.yaml
        // exactly (ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml).
        t.ribbon.width_m = 0.24f;
        t.ribbon.lane_width_m = 3.5f;
        // 0.5m rim per side so the 3 stacked ribbons stay visually distinct.
        t.ribbon.margin_behavior_m = 1.3f;  // narrowest -- top of the z-stagger, the hero ribbon
        t.ribbon.margin_global_m = 0.3f;    // widest -- bottom of the z-stagger
        t.ribbon.margin_local_m = 0.8f;
        // Between margin_local_m and margin_behavior_m -- see theme.hpp's
        // own comment (VM-077 carpet-as-ribbon redirect, 2026-09-10).
        t.ribbon.margin_velocity_m = 1.05f;
        // Must match dark_adas.yaml exactly (VM-078).
        t.objects.opacity = 1.0f;
        return t;
    }();
    return theme;
}

}  // namespace mpviz::detail

namespace mpviz {

// See scene.h for the full rationale. Two lines over the existing
// GPU-free detail::load_theme(): guard against null (std::string's ctor is
// UB on nullptr; detail::load_theme already treats an empty dir/name as a
// non-fatal std::nullopt) and report success.
bool theme_parses(const char* dir, const char* theme_name) {
    if (dir == nullptr || theme_name == nullptr) return false;
    return detail::load_theme(dir, theme_name).has_value();
}

}  // namespace mpviz
