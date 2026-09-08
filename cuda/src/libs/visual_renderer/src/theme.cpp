#include "theme.hpp"

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

// Throws (caught by load_theme) on any missing/malformed key — every field
// in the schema is required EXCEPT palette.ego, which is soft-defaulted
// (see its inline comment below). No other per-theme optional keys (see the
// YAML files' own "no per-theme code branches" comment).
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
    // ego (user directive 2026-08-20): the one OPTIONAL palette key, unlike
    // every required field above/below -- soft-defaulted rather than thrown
    // on absence (profile.cpp's `node[key] ? node[key].as<T>() : def`
    // convention, ros_apps/.../profile.cpp), so shipped/fixture theme YAMLs
    // written before this field existed (tests/fixtures/themes/*.yaml) keep
    // parsing instead of falling back to the whole compiled-in
    // kFallbackTheme() over one missing key. Default matches kFallbackTheme
    // ()'s own palette.ego below.
    t.palette.ego = palette["ego"] ? to_float3(palette["ego"]) : Float3{0.82f, 0.80f, 0.76f};

    // ribbon_global/ribbon_local (user directive 2026-08-20, ITEM 1): the
    // other two OPTIONAL palette keys, same soft-default convention as
    // `ego` just above -- missing either reproduces today's look (GLOBAL
    // reused ribbon_core, LOCAL reused ribbon_glow; see renderer.cpp's old
    // push_theme_to_scene() comment), not a fallback to the whole compiled-
    // in kFallbackTheme().
    t.palette.ribbon_global =
        palette["ribbon_global"] ? to_float3(palette["ribbon_global"]) : t.palette.ribbon_core;
    t.palette.ribbon_local =
        palette["ribbon_local"] ? to_float3(palette["ribbon_local"]) : t.palette.ribbon_glow;

    // road/lane_centerline/lane_boundary/crosswalk (Epic 3 Task 1 / VM-036,
    // decision #6): same soft-default convention as ribbon_global/
    // ribbon_local above. `road` falls back to `ground` (today's "ground
    // carries the road tone" look); the other three fall back to
    // `lane_paint` (today's "every map element is one stroke color" look).
    t.palette.road = palette["road"] ? to_float3(palette["road"]) : t.palette.ground;
    t.palette.lane_centerline =
        palette["lane_centerline"] ? to_float3(palette["lane_centerline"]) : t.palette.lane_paint;
    t.palette.lane_boundary =
        palette["lane_boundary"] ? to_float3(palette["lane_boundary"]) : t.palette.lane_paint;
    t.palette.crosswalk =
        palette["crosswalk"] ? to_float3(palette["crosswalk"]) : t.palette.lane_paint;
    // road_edge (user directive 2026-09-08): same soft-default convention,
    // falls back to lane_paint (the palette.ego precedent) -- neither
    // shipped theme relies on it, both author an explicit yellow.
    t.palette.road_edge =
        palette["road_edge"] ? to_float3(palette["road_edge"]) : t.palette.lane_paint;

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

    const YAML::Node sun = root["sun"];
    t.sun.direction = to_float3(sun["direction"]);
    t.sun.color = to_float3(sun["color"]);
    t.sun.intensity = sun["intensity"].as<float>();

    const YAML::Node ibl = root["ibl"];
    t.ibl.sky_color = to_float3(ibl["sky_color"]);
    t.ibl.ground_color = to_float3(ibl["ground_color"]);
    t.ibl.intensity = ibl["intensity"].as<float>();

    t.fog.density = root["fog"]["density"].as<float>();

    // ribbon.width_m (user directive 2026-08-20, ITEM 1): the whole `ribbon:`
    // section, and width_m within it, are OPTIONAL -- soft-defaulted to
    // Theme::Ribbon's own 0.24 default (theme.hpp), same convention as
    // palette.ego/ribbon_global/ribbon_local above.
    const YAML::Node ribbon = root["ribbon"];
    t.ribbon.width_m = (ribbon && ribbon["width_m"]) ? ribbon["width_m"].as<float>() : 0.24f;

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
        // mistyped key inside parse() above -- all non-fatal per spec §9;
        // caller (create_renderer, Step 7b) falls back to kFallbackTheme().
        return std::nullopt;
    }
}

const Theme& kFallbackTheme() {
    static const Theme theme = [] {
        // Same values as dark_adas.yaml, kept in code so a broken/missing
        // theme-asset install can never take rendering down (spec §9).
        Theme t;
        t.name = "dark_adas";
        t.palette.ground = {0.05f, 0.06f, 0.08f};
        t.palette.sky = {0.02f, 0.02f, 0.05f};
        t.palette.fog = {0.02f, 0.02f, 0.05f};
        // Re-authored toward near-white (Epic 3 Task 1 / VM-036 debt item
        // a, review-verified: was mid-gray [0.45,0.5,0.55]) -- must match
        // dark_adas.yaml exactly (ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml).
        t.palette.lane_paint = {0.85f, 0.85f, 0.88f};
        t.palette.ribbon_core = {0.10f, 1.00f, 0.40f};
        t.palette.ribbon_glow = {0.10f, 1.00f, 0.40f};
        // Cross-theme swap (user directive 2026-08-20): light_clay's ground
        // color, same value dark_adas.yaml's `ego` key authors on disk --
        // see this struct's own header comment in theme.hpp.
        t.palette.ego = {0.82f, 0.80f, 0.76f};
        // dark_adas.yaml's own authored values (user directive 2026-08-20,
        // ITEM 1) -- not a reuse of ribbon_core/glow, unlike the pre-ITEM-1
        // code this fallback mirrors.
        t.palette.ribbon_global = {0.25f, 0.55f, 0.95f};
        t.palette.ribbon_local = {0.95f, 0.70f, 0.15f};
        // road/lane_centerline/lane_boundary/crosswalk/road_edge (Epic 3
        // Task 1 / VM-036, decision #6; lane_centerline + road_edge
        // re-authored by user directive 2026-09-08) -- must match
        // dark_adas.yaml exactly (ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml).
        // road is darker than palette.ground (0.05,0.06,0.08);
        // lane_boundary/crosswalk are the same near-white lane_paint was
        // re-authored toward; lane_centerline is now a LOW-CONTRAST fade of
        // lane_paint toward road (25% lane_paint / 75% road -- faint dot
        // guidance, not a bold stroke); road_edge is a clear, fully-
        // saturated road-paint yellow, solid and readable on the dark road.
        t.palette.road = {0.03f, 0.035f, 0.045f};
        t.palette.lane_centerline = {0.235f, 0.239f, 0.254f};
        t.palette.lane_boundary = {0.85f, 0.85f, 0.88f};
        t.palette.crosswalk = {0.85f, 0.85f, 0.88f};
        t.palette.road_edge = {0.95f, 0.75f, 0.05f};
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
        t.emissive.ribbon_strength = 4.0f;
        t.grid.line_color = {0.12f, 0.14f, 0.18f};
        t.grid.fade_start_m = 15.0f;
        t.grid.fade_end_m = 40.0f;
        t.hud.text_color = {0.9f, 0.95f, 1.0f};
        t.hud.accent_color = {0.10f, 1.0f, 0.4f};
        t.hud.scale = 1.0f;
        t.sun.direction = {-0.5f, -0.3f, -1.0f};
        t.sun.color = {0.55f, 0.6f, 0.75f};
        t.sun.intensity = 480000.0f;
        t.ibl.sky_color = {0.05f, 0.06f, 0.12f};
        t.ibl.ground_color = {0.02f, 0.02f, 0.03f};
        t.ibl.intensity = 256000.0f;
        t.fog.density = 0.015f;
        t.ribbon.width_m = 0.24f;
        return t;
    }();
    return theme;
}

}  // namespace mpviz::detail

namespace mpviz {

// Epic 2 Task 1 (VM-020) Step 0.3 -- see scene.h for the full rationale.
// Two lines over the existing GPU-free detail::load_theme(): guard against
// null (std::string's ctor is UB on nullptr; detail::load_theme already
// treats an EMPTY dir/name as a non-fatal std::nullopt) and report success.
bool theme_parses(const char* dir, const char* theme_name) {
    if (dir == nullptr || theme_name == nullptr) return false;
    return detail::load_theme(dir, theme_name).has_value();
}

}  // namespace mpviz
