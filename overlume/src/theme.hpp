// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace overlume::detail {

inline constexpr float kThemeDefaultTileRadiusM = 700.0f;

inline bool ribbon_length_fade_enabled(float fade_start_m, float fade_end_m) {
    return fade_start_m >= 0.0f && fade_end_m > fade_start_m;
}

struct Float3 {
    float r = 0.0f, g = 0.0f, b = 0.0f;
};

struct Theme {
    std::string name;

    struct Palette {
        Float3 ground;
        Float3 sky;
        Float3 fog;
        Float3 lane_paint;
        Float3 ribbon_core;
        Float3 ribbon_glow;
        Float3 ego;
        Float3 ribbon_global;
        Float3 ribbon_local;
        Float3 road;
        Float3 lane_centerline;
        Float3 lane_boundary;
        Float3 crosswalk;
        Float3 road_edge;
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

    struct PointCloudStyle {
        float point_size_px = 2.0f;
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

    struct Ribbon {
        float width_m = 0.24f;
        float lane_width_m = 3.5f;
        float margin_behavior_m = 1.63f;
        float margin_global_m = 1.63f;
        float margin_local_m = 1.63f;
        float margin_velocity_m = 1.05f;
        float opacity = 1.0f;
        float fade_start_m = 0.0f;
        float fade_end_m = 0.0f;
    } ribbon;

    struct Objects {
        float opacity = 1.0f;
    } objects;

    struct OgmRamp {
        static constexpr size_t kEntries = 101;
        std::array<Float3, kEntries> color{};
        std::array<float, kEntries> alpha{};
    };
    struct Ogm {
        OgmRamp dynamic;
        OgmRamp geometric;
    } ogm;

    struct Environment {
        float tile_radius_m = kThemeDefaultTileRadiusM;
    } environment;
};

struct OgmRampStop {
    float value = 0.0f;
    Float3 color;
    float alpha = 1.0f;
};

Theme::OgmRamp bake_ogm_ramp(std::vector<OgmRampStop> stops);

std::optional<Theme> load_theme(const std::string& dir, const std::string& name);

const Theme& kFallbackTheme();

}
