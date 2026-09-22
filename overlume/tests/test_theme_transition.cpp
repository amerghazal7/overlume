// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "theme.hpp"
#include "theme_transition.hpp"

#include "golden.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace {

double MaxAbsDiff(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    double maxDiff = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        maxDiff =
            std::max(maxDiff, std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i])));
    }
    return maxDiff;
}

}

TEST(OklabHelpers, RoundTripIsIdentity) {
    const overlume::detail::Float3 colors[] = {
        {0.05f, 0.06f, 0.08f},
        {0.82f, 0.80f, 0.76f},
        {1.0f, 1.0f, 1.0f},
        {0.0f, 0.0f, 0.0f},
    };
    for (const auto& c : colors) {
        const overlume::detail::Oklab lab = overlume::detail::linear_srgb_to_oklab(c);
        const overlume::detail::Float3 back = overlume::detail::oklab_to_linear_srgb(lab);
        EXPECT_NEAR(back.r, c.r, 1e-4f);
        EXPECT_NEAR(back.g, c.g, 1e-4f);
        EXPECT_NEAR(back.b, c.b, 1e-4f);
    }
}

TEST(ThemeTransition, MidpointBlend_IsBetweenEndpointsInOklab) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const overlume::detail::Float3& a = dark->palette.ground;
    const overlume::detail::Float3& b = light->palette.ground;
    const overlume::detail::Float3 oklabBlend = overlume::detail::blend_color(a, b, 0.5f);
    const overlume::detail::Float3 naiveLerp{(a.r + b.r) / 2.0f, (a.g + b.g) / 2.0f,
                                             (a.b + b.b) / 2.0f};

    const double delta = std::abs(oklabBlend.r - naiveLerp.r) +
                         std::abs(oklabBlend.g - naiveLerp.g) +
                         std::abs(oklabBlend.b - naiveLerp.b);
    EXPECT_GT(delta, 0.01) << "Oklab-space blend and naive sRGB lerp landed on ~the same color -- "
                              "blend_color() isn't actually blending in Oklab space.";

    const float La = overlume::detail::linear_srgb_to_oklab(a).L;
    const float Lb = overlume::detail::linear_srgb_to_oklab(b).L;
    const float Lmid = overlume::detail::linear_srgb_to_oklab(oklabBlend).L;
    EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
    EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);
}

TEST(ThemeTransition, Smoothstep_EasesInAndOut) {
    EXPECT_LT(overlume::detail::smoothstep01(0.1f), 0.1f);
    EXPECT_GT(overlume::detail::smoothstep01(0.9f), 0.9f);
    EXPECT_FLOAT_EQ(overlume::detail::smoothstep01(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(overlume::detail::smoothstep01(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(overlume::detail::smoothstep01(0.5f), 0.5f);
}

TEST(ThemeTransition, DeterministicClock_MatchesTargetAtDuration) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);
    overlume::set_theme(r, "light_clay", 0.0, 0.8);

    overlume::CameraPose kFixedPose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};

    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);
    EXPECT_GT(overlume::testing::render_and_compare(
                  r, kFixedPose, OVERLUME_TEST_DATA_DIR "/tests/goldens/transition_t0.png",
                  "/tmp/transition_t0_actual.png"),
              0.98);

    scene.sim_time_sec = 0.4;
    overlume::set_scene(r, scene);
    EXPECT_GT(overlume::testing::render_and_compare(
                  r, kFixedPose, OVERLUME_TEST_DATA_DIR "/tests/goldens/transition_t0_4.png",
                  "/tmp/transition_t0_4_actual.png"),
              0.98);

    scene.sim_time_sec = 0.8;
    overlume::set_scene(r, scene);
    EXPECT_GT(overlume::testing::render_and_compare(
                  r, kFixedPose, OVERLUME_TEST_DATA_DIR "/tests/goldens/transition_t0_8.png",
                  "/tmp/transition_t0_8_actual.png"),
              0.98);

    overlume::destroy_renderer(r);
}

TEST(ThemeTransition, RetargetMidFlight_StartsFromCurrentBlendNotEndpoint) {
    constexpr uint32_t kWidth = 320, kHeight = 240;
    overlume::RenderConfig cfg{kWidth, kHeight, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }

    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);
    overlume::set_theme(r, "light_clay", 0.0, 0.8);

    scene.sim_time_sec = 0.4;
    overlume::set_scene(r, scene);
    std::vector<uint8_t> beforeRetarget(static_cast<size_t>(kWidth) * kHeight * 3);
    overlume::FrameView beforeView{beforeRetarget.data(), kWidth, kHeight};
    ASSERT_TRUE(overlume::render_frame(r, pose, beforeView));

    ASSERT_TRUE(overlume::set_theme(r, "dark_adas", 0.4, 0.8));

    std::vector<uint8_t> afterRetarget(static_cast<size_t>(kWidth) * kHeight * 3);
    overlume::FrameView afterView{afterRetarget.data(), kWidth, kHeight};
    ASSERT_TRUE(overlume::render_frame(r, pose, afterView));

    EXPECT_LT(MaxAbsDiff(beforeRetarget, afterRetarget), 24.0)
        << "retargeting set_theme() mid-flight visibly snapped the frame -- "
           "the new transition's `from` isn't the current blend.";

    overlume::destroy_renderer(r);
}

TEST(ThemeTransition, MidTransition_LuminanceDoesNotOvershootEndpoints) {
    constexpr uint32_t kWidth = 320, kHeight = 240;
    overlume::RenderConfig cfg{kWidth, kHeight, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};

    const overlume::testing::FrameStats endpointA =
        overlume::testing::analyze_png(OVERLUME_TEST_DATA_DIR "/tests/goldens/transition_t0.png");
    const overlume::testing::FrameStats endpointB =
        overlume::testing::analyze_png(OVERLUME_TEST_DATA_DIR "/tests/goldens/transition_t0_8.png");
    ASSERT_GT(endpointA.mean, 0.0) << "couldn't load transition_t0.png golden";
    ASSERT_GT(endpointB.mean, 0.0) << "couldn't load transition_t0_8.png golden";
    const double loBound = std::min(endpointA.mean, endpointB.mean) - 3.0;
    const double hiBound = std::max(endpointA.mean, endpointB.mean) + 3.0;

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);
    overlume::set_theme(r, "light_clay", 0.0, 0.8);

    const double sampleTsOfDuration[] = {0.5, 0.6};
    for (double tOfDuration : sampleTsOfDuration) {
        scene.sim_time_sec = tOfDuration * 0.8;
        overlume::set_scene(r, scene);
        const std::string outPath =
            "/tmp/transition_mid_t" + std::to_string(tOfDuration) + "_actual.png";
        overlume::testing::render_and_compare(r, pose, "/nonexistent/no_such_golden.png",
                                              outPath.c_str());
        const overlume::testing::FrameStats stats = overlume::testing::analyze_png(outPath.c_str());
        EXPECT_GE(stats.mean, loBound)
            << "t=" << tOfDuration << " mean " << stats.mean << " undershot endpoints ["
            << endpointA.mean << ", " << endpointB.mean << "]";
        EXPECT_LE(stats.mean, hiBound)
            << "t=" << tOfDuration << " mean " << stats.mean << " overshot endpoints ["
            << endpointA.mean << ", " << endpointB.mean
            << "] -- illumination x albedo blew out mid-transition (linear intensity lerp?)";
    }

    overlume::destroy_renderer(r);
}

namespace {

overlume::detail::Theme MakeSentinelTheme(float scalar, const std::string& name) {
    using overlume::detail::Float3;
    const Float3 c{scalar, scalar, scalar};
    overlume::detail::Theme t;
    t.name = name;
    t.palette.ground = c;
    t.palette.sky = c;
    t.palette.fog = c;
    t.palette.lane_paint = c;
    t.palette.ribbon_core = c;
    t.palette.ribbon_glow = c;
    t.palette.ego = c;
    t.palette.ribbon_global = c;
    t.palette.ribbon_local = c;
    t.palette.road = c;
    t.palette.lane_centerline = c;
    t.palette.lane_boundary = c;
    t.palette.crosswalk = c;
    t.palette.road_edge = c;
    t.palette.building = c;
    t.palette.object_tints.car = c;
    t.palette.object_tints.truck_van = c;
    t.palette.object_tints.bus = c;
    t.palette.object_tints.pedestrian = c;
    t.palette.object_tints.cyclist = c;
    t.palette.object_tints.unknown = c;
    t.palette.alert.info = c;
    t.palette.alert.warning = c;
    t.palette.alert.critical = c;
    t.material.roughness = scalar;
    t.material.metallic = scalar;
    t.emissive.ribbon_strength = scalar;
    t.grid.line_color = c;
    t.grid.fade_start_m = scalar;
    t.grid.fade_end_m = scalar;
    t.hud.text_color = c;
    t.hud.accent_color = c;
    t.hud.scale = scalar;
    t.point_cloud.point_size_px = scalar;
    t.sun.direction = c;
    t.sun.color = c;
    t.sun.intensity = scalar;
    t.ibl.sky_color = c;
    t.ibl.ground_color = c;
    t.ibl.intensity = scalar;
    t.fog.density = scalar;
    t.ribbon.width_m = scalar;
    t.ribbon.lane_width_m = scalar;
    t.ribbon.margin_behavior_m = scalar;
    t.ribbon.margin_global_m = scalar;
    t.ribbon.margin_local_m = scalar;
    t.ribbon.margin_velocity_m = scalar;
    t.objects.opacity = scalar;
    return t;
}

void ExpectBetweenSentinels(float v, const char* label) {
    EXPECT_GT(v, 100.0f) << label << " looks unblended (silently defaulted?)";
    EXPECT_LT(v, 230.0f) << label << " looks unblended (silently defaulted?)";
}

void ExpectBetweenSentinels(const overlume::detail::Float3& v, const char* label) {
    ExpectBetweenSentinels(v.r, (std::string(label) + ".r").c_str());
    ExpectBetweenSentinels(v.g, (std::string(label) + ".g").c_str());
    ExpectBetweenSentinels(v.b, (std::string(label) + ".b").c_str());
}

}

TEST(ThemeTransition, SentinelThemesDetectAnyUnblendedField) {
    const overlume::detail::Theme a = MakeSentinelTheme(111.0f, "a");
    const overlume::detail::Theme b = MakeSentinelTheme(222.0f, "b");
    const overlume::detail::Theme mid = overlume::detail::blend(a, b, 0.5f);

    ExpectBetweenSentinels(mid.palette.ground, "palette.ground");
    ExpectBetweenSentinels(mid.palette.sky, "palette.sky");
    ExpectBetweenSentinels(mid.palette.fog, "palette.fog");
    ExpectBetweenSentinels(mid.palette.lane_paint, "palette.lane_paint");
    ExpectBetweenSentinels(mid.palette.ribbon_core, "palette.ribbon_core");
    ExpectBetweenSentinels(mid.palette.ribbon_glow, "palette.ribbon_glow");
    ExpectBetweenSentinels(mid.palette.ego, "palette.ego");
    ExpectBetweenSentinels(mid.palette.ribbon_global, "palette.ribbon_global");
    ExpectBetweenSentinels(mid.palette.ribbon_local, "palette.ribbon_local");
    ExpectBetweenSentinels(mid.palette.road, "palette.road");
    ExpectBetweenSentinels(mid.palette.lane_centerline, "palette.lane_centerline");
    ExpectBetweenSentinels(mid.palette.lane_boundary, "palette.lane_boundary");
    ExpectBetweenSentinels(mid.palette.crosswalk, "palette.crosswalk");
    ExpectBetweenSentinels(mid.palette.road_edge, "palette.road_edge");
    ExpectBetweenSentinels(mid.palette.building, "palette.building");
    ExpectBetweenSentinels(mid.palette.object_tints.car, "palette.object_tints.car");
    ExpectBetweenSentinels(mid.palette.object_tints.truck_van, "palette.object_tints.truck_van");
    ExpectBetweenSentinels(mid.palette.object_tints.bus, "palette.object_tints.bus");
    ExpectBetweenSentinels(mid.palette.object_tints.pedestrian, "palette.object_tints.pedestrian");
    ExpectBetweenSentinels(mid.palette.object_tints.cyclist, "palette.object_tints.cyclist");
    ExpectBetweenSentinels(mid.palette.object_tints.unknown, "palette.object_tints.unknown");
    ExpectBetweenSentinels(mid.palette.alert.info, "palette.alert.info");
    ExpectBetweenSentinels(mid.palette.alert.warning, "palette.alert.warning");
    ExpectBetweenSentinels(mid.palette.alert.critical, "palette.alert.critical");
    ExpectBetweenSentinels(mid.material.roughness, "material.roughness");
    ExpectBetweenSentinels(mid.material.metallic, "material.metallic");
    ExpectBetweenSentinels(mid.emissive.ribbon_strength, "emissive.ribbon_strength");
    ExpectBetweenSentinels(mid.grid.line_color, "grid.line_color");
    ExpectBetweenSentinels(mid.grid.fade_start_m, "grid.fade_start_m");
    ExpectBetweenSentinels(mid.grid.fade_end_m, "grid.fade_end_m");
    ExpectBetweenSentinels(mid.hud.text_color, "hud.text_color");
    ExpectBetweenSentinels(mid.hud.accent_color, "hud.accent_color");
    ExpectBetweenSentinels(mid.hud.scale, "hud.scale");
    ExpectBetweenSentinels(mid.point_cloud.point_size_px, "point_cloud.point_size_px");
    ExpectBetweenSentinels(mid.sun.direction, "sun.direction");
    ExpectBetweenSentinels(mid.sun.color, "sun.color");
    ExpectBetweenSentinels(mid.sun.intensity, "sun.intensity");
    ExpectBetweenSentinels(mid.ibl.sky_color, "ibl.sky_color");
    ExpectBetweenSentinels(mid.ibl.ground_color, "ibl.ground_color");
    ExpectBetweenSentinels(mid.ibl.intensity, "ibl.intensity");
    ExpectBetweenSentinels(mid.fog.density, "fog.density");
    ExpectBetweenSentinels(mid.ribbon.width_m, "ribbon.width_m");
    ExpectBetweenSentinels(mid.ribbon.lane_width_m, "ribbon.lane_width_m");
    ExpectBetweenSentinels(mid.ribbon.margin_behavior_m, "ribbon.margin_behavior_m");
    ExpectBetweenSentinels(mid.ribbon.margin_global_m, "ribbon.margin_global_m");
    ExpectBetweenSentinels(mid.ribbon.margin_local_m, "ribbon.margin_local_m");
    ExpectBetweenSentinels(mid.ribbon.margin_velocity_m, "ribbon.margin_velocity_m");
    ExpectBetweenSentinels(mid.objects.opacity, "objects.opacity");
}

TEST(ThemeTransition, UnknownThemeName_ReturnsFalseAndLeavesActiveThemeUnchanged) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);

    EXPECT_FALSE(overlume::set_theme(r, "no_such_theme", 0.0, 0.8));

    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    EXPECT_GT(overlume::testing::render_and_compare(
                  r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png",
                  "/tmp/set_theme_unknown_actual.png"),
              0.98);
    overlume::destroy_renderer(r);
}
