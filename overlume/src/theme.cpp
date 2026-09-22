// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "theme.hpp"

#include <algorithm>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "overlume/scene.h"

namespace overlume::detail {
namespace {

Float3 to_float3(const YAML::Node& node) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error("theme: expected a 3-element sequence");
    }
    return Float3{node[0].as<float>(), node[1].as<float>(), node[2].as<float>()};
}

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
    t.palette.ego = palette["ego"] ? to_float3(palette["ego"]) : Float3{0.82f, 0.80f, 0.76f};

    t.palette.ribbon_global =
        palette["ribbon_global"] ? to_float3(palette["ribbon_global"]) : t.palette.ribbon_core;
    t.palette.ribbon_local =
        palette["ribbon_local"] ? to_float3(palette["ribbon_local"]) : t.palette.ribbon_glow;

    t.palette.road = palette["road"] ? to_float3(palette["road"]) : t.palette.ground;
    t.palette.lane_centerline =
        palette["lane_centerline"] ? to_float3(palette["lane_centerline"]) : t.palette.lane_paint;
    t.palette.lane_boundary =
        palette["lane_boundary"] ? to_float3(palette["lane_boundary"]) : t.palette.lane_paint;
    t.palette.crosswalk =
        palette["crosswalk"] ? to_float3(palette["crosswalk"]) : t.palette.lane_paint;
    t.palette.road_edge =
        palette["road_edge"] ? to_float3(palette["road_edge"]) : t.palette.lane_paint;

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

    const YAML::Node pc = root["point_cloud"];
    t.point_cloud.point_size_px =
        (pc && pc["point_size_px"]) ? pc["point_size_px"].as<float>() : 2.0f;

    const YAML::Node sun = root["sun"];
    t.sun.direction = to_float3(sun["direction"]);
    t.sun.color = to_float3(sun["color"]);
    t.sun.intensity = sun["intensity"].as<float>();

    const YAML::Node ibl = root["ibl"];
    t.ibl.sky_color = to_float3(ibl["sky_color"]);
    t.ibl.ground_color = to_float3(ibl["ground_color"]);
    t.ibl.intensity = ibl["intensity"].as<float>();

    t.fog.density = root["fog"]["density"].as<float>();

    const YAML::Node ribbon = root["ribbon"];
    t.ribbon.width_m = (ribbon && ribbon["width_m"]) ? ribbon["width_m"].as<float>() : 0.24f;

    t.ribbon.lane_width_m =
        (ribbon && ribbon["lane_width_m"]) ? ribbon["lane_width_m"].as<float>() : 3.5f;
    const float marginDefault = (t.ribbon.lane_width_m - t.ribbon.width_m) / 2.0f;
    t.ribbon.margin_behavior_m = (ribbon && ribbon["margin_behavior_m"])
                                     ? ribbon["margin_behavior_m"].as<float>()
                                     : marginDefault;
    t.ribbon.margin_global_m = (ribbon && ribbon["margin_global_m"])
                                   ? ribbon["margin_global_m"].as<float>()
                                   : marginDefault;
    t.ribbon.margin_local_m =
        (ribbon && ribbon["margin_local_m"]) ? ribbon["margin_local_m"].as<float>() : marginDefault;

    t.ribbon.margin_velocity_m =
        (ribbon && ribbon["margin_velocity_m"]) ? ribbon["margin_velocity_m"].as<float>() : 1.05f;

    const YAML::Node objects = root["objects"];
    t.objects.opacity = std::clamp(
        (objects && objects["opacity"]) ? objects["opacity"].as<float>() : 1.0f, 0.0f, 1.0f);

    return t;
}

}

std::optional<Theme> load_theme(const std::string& dir, const std::string& name) {
    if (dir.empty() || name.empty()) return std::nullopt;
    const std::string path = dir + "/" + name + ".yaml";
    try {
        const YAML::Node root = YAML::LoadFile(path);
        return parse(root);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

const Theme& kFallbackTheme() {
    static const Theme theme = [] {
        Theme t;
        t.name = "dark_adas";
        t.palette.ground = {0.055f, 0.055f, 0.078f};
        t.palette.sky = {0.028f, 0.036f, 0.085f};
        t.palette.fog = {0.028f, 0.036f, 0.085f};
        t.palette.lane_paint = {0.85f, 0.85f, 0.88f};
        t.palette.ribbon_core = {0.12f, 0.55f, 0.42f};
        t.palette.ribbon_glow = {0.12f, 0.55f, 0.42f};
        t.palette.ego = {0.82f, 0.80f, 0.76f};
        t.palette.ribbon_global = {0.22f, 0.30f, 0.42f};
        t.palette.ribbon_local = {0.55f, 0.42f, 0.22f};
        t.palette.road = {0.034f, 0.042f, 0.078f};
        t.palette.lane_centerline = {0.238f, 0.244f, 0.279f};
        t.palette.lane_boundary = {0.85f, 0.85f, 0.88f};
        t.palette.crosswalk = {0.42f, 0.36f, 0.22f};
        t.palette.road_edge = {0.720f, 0.520f, 0.090f};
        t.palette.building = {0.130f, 0.135f, 0.180f};
        t.palette.object_tints.car = {0.180f, 0.210f, 0.320f};
        t.palette.object_tints.truck_van = {0.28f, 0.32f, 0.55f};
        t.palette.object_tints.bus = {0.75f, 0.55f, 0.20f};
        t.palette.object_tints.pedestrian = {0.85f, 0.25f, 0.25f};
        t.palette.object_tints.cyclist = {0.80f, 0.50f, 0.15f};
        t.palette.object_tints.unknown = {0.45f, 0.45f, 0.50f};
        t.palette.alert.info = {0.20f, 0.55f, 0.85f};
        t.palette.alert.warning = {0.85f, 0.60f, 0.15f};
        t.palette.alert.critical = {0.90f, 0.20f, 0.15f};
        t.material.roughness = 0.85f;
        t.material.metallic = 0.0f;
        t.emissive.ribbon_strength = 0.0f;
        t.grid.line_color = {0.130f, 0.130f, 0.180f};
        t.grid.fade_start_m = 15.0f;
        t.grid.fade_end_m = 40.0f;
        t.hud.text_color = {0.90f, 0.92f, 0.95f};
        t.hud.accent_color = {0.12f, 0.60f, 0.45f};
        t.hud.scale = 1.0f;
        t.point_cloud.point_size_px = 2.0f;
        t.sun.direction = {-0.6f, -0.2f, -0.5f};
        t.sun.color = {0.85f, 0.65f, 0.55f};
        t.sun.intensity = 350000.0f;
        t.ibl.sky_color = {0.028f, 0.036f, 0.085f};
        t.ibl.ground_color = {0.030f, 0.030f, 0.045f};
        t.ibl.intensity = 350000.0f;
        t.fog.density = 0.010f;
        t.ribbon.width_m = 0.24f;
        t.ribbon.lane_width_m = 3.5f;
        t.ribbon.margin_behavior_m = 1.3f;
        t.ribbon.margin_global_m = 0.3f;
        t.ribbon.margin_local_m = 0.8f;
        t.ribbon.margin_velocity_m = 1.05f;
        t.objects.opacity = 1.0f;
        return t;
    }();
    return theme;
}

}

namespace overlume {

bool theme_parses(const char* dir, const char* theme_name) {
    if (dir == nullptr || theme_name == nullptr) return false;
    return detail::load_theme(dir, theme_name).has_value();
}

}
