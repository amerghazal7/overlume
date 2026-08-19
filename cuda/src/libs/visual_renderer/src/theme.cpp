#include "theme.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace mpviz::detail {
namespace {

Float3 to_float3(const YAML::Node& node) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error("theme: expected a 3-element sequence");
    }
    return Float3{node[0].as<float>(), node[1].as<float>(), node[2].as<float>()};
}

// Throws (caught by load_theme) on any missing/malformed key — every field
// in the schema is required, no per-theme optional keys (see the YAML
// files' own "no per-theme code branches" comment).
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
        t.palette.lane_paint = {0.45f, 0.5f, 0.55f};
        t.palette.ribbon_core = {0.10f, 1.00f, 0.40f};
        t.palette.ribbon_glow = {0.10f, 1.00f, 0.40f};
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
        return t;
    }();
    return theme;
}

}  // namespace mpviz::detail
