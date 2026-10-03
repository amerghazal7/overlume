// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "test_paths.hpp"
#include "theme.hpp"
#include "theme_transition.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace {

bool AnyDiffer(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) { return a != b; }

}

TEST(ThemePalette, EgoContrastsWithTheGroundInBothThemes) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const auto lightEgo = overlume::detail::linear_srgb_to_oklab(light->palette.ego);
    const auto lightGround = overlume::detail::linear_srgb_to_oklab(light->palette.ground);
    EXPECT_GT(lightGround.L - lightEgo.L, 0.2f)
        << "light_clay's ego must read as a darker grey against its light ground";
    EXPECT_LT(std::sqrt(lightEgo.a * lightEgo.a + lightEgo.b * lightEgo.b), 0.02f)
        << "light_clay's ego is a neutral grey";

    const float darkEgoLightness = overlume::detail::linear_srgb_to_oklab(dark->palette.ego).L;
    const float darkGroundLightness =
        overlume::detail::linear_srgb_to_oklab(dark->palette.ground).L;
    EXPECT_GT(darkEgoLightness, darkGroundLightness + 0.3f)
        << "dark_adas's ego color no longer contrasts against its own ground";
}

TEST(ThemePalette, EgoFallsBackToBuiltinDefaultWhenMissingFromYaml) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing only the optional `ego` key must still parse";
    EXPECT_NEAR(theme->palette.ego.r, 0.82f, 1e-4f);
    EXPECT_NEAR(theme->palette.ego.g, 0.80f, 1e-4f);
    EXPECT_NEAR(theme->palette.ego.b, 0.76f, 1e-4f);
}

TEST(ThemePalette, EgoBlendsInOklabAcrossTransition) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    overlume::detail::Theme other = *light;
    other.palette.ego = dark->palette.ground;
    const overlume::detail::Theme mid = overlume::detail::blend(*dark, other, 0.5f);

    const float La = overlume::detail::linear_srgb_to_oklab(dark->palette.ego).L;
    const float Lb = overlume::detail::linear_srgb_to_oklab(other.palette.ego).L;
    const float Lmid = overlume::detail::linear_srgb_to_oklab(mid.palette.ego).L;
    EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
    EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);

    EXPECT_GT(std::abs(Lmid - La), 1e-4f);
    EXPECT_GT(std::abs(Lmid - Lb), 1e-4f);
}

TEST(ThemePalette, RibbonGlobalLocalAndWidthFallBackToTodaysValuesWhenMissingFromYaml) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing only the optional ribbon_global/ribbon_local/ribbon keys "
           "must still parse";

    EXPECT_NEAR(theme->palette.ribbon_global.r, theme->palette.ribbon_core.r, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_global.g, theme->palette.ribbon_core.g, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_global.b, theme->palette.ribbon_core.b, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_local.r, theme->palette.ribbon_glow.r, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_local.g, theme->palette.ribbon_glow.g, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_local.b, theme->palette.ribbon_glow.b, 1e-4f);

    EXPECT_NEAR(theme->ribbon.width_m, 0.24f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.opacity, 1.0f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.fade_start_m, 0.0f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.fade_end_m, 0.0f, 1e-4f);
}

TEST(ThemePalette, RibbonGlobalLocalBlendInOklabAcrossTransition) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const overlume::detail::Theme mid = overlume::detail::blend(*dark, *light, 0.5f);

    for (const auto& [a, b, m] :
         {std::tuple{dark->palette.ribbon_global, light->palette.ribbon_global,
                     mid.palette.ribbon_global},
          std::tuple{dark->palette.ribbon_local, light->palette.ribbon_local,
                     mid.palette.ribbon_local}}) {
        const float La = overlume::detail::linear_srgb_to_oklab(a).L;
        const float Lb = overlume::detail::linear_srgb_to_oklab(b).L;
        const float Lmid = overlume::detail::linear_srgb_to_oklab(m).L;
        EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
        EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);
        EXPECT_GT(std::abs(Lmid - La), 1e-4f);
        EXPECT_GT(std::abs(Lmid - Lb), 1e-4f);
    }
}

TEST(ThemePalette, RibbonWidthLerpsLinearlyAcrossTransition) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const overlume::detail::Theme mid = overlume::detail::blend(*dark, *light, 0.5f);
    const float expectedMid = (dark->ribbon.width_m + light->ribbon.width_m) / 2.0f;
    EXPECT_NEAR(mid.ribbon.width_m, expectedMid, 1e-4f);
}

TEST(ThemePalette, RibbonLaneWidthAndMarginsFallBackToTheWidthMSeedWhenMissingFromYaml) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing the optional lane_width_m/margin_*_m keys must still parse";

    EXPECT_NEAR(theme->ribbon.lane_width_m, 3.5f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.margin_behavior_m, 1.63f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.margin_global_m, 1.63f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.margin_local_m, 1.63f, 1e-4f);
}

TEST(ThemePalette, RibbonLaneWidthAndMarginsLerpLinearlyAcrossTransition) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const overlume::detail::Theme mid = overlume::detail::blend(*dark, *light, 0.5f);
    EXPECT_NEAR(mid.ribbon.lane_width_m,
                (dark->ribbon.lane_width_m + light->ribbon.lane_width_m) / 2.0f, 1e-4f);
    EXPECT_NEAR(mid.ribbon.margin_behavior_m,
                (dark->ribbon.margin_behavior_m + light->ribbon.margin_behavior_m) / 2.0f, 1e-4f);
    EXPECT_NEAR(mid.ribbon.margin_global_m,
                (dark->ribbon.margin_global_m + light->ribbon.margin_global_m) / 2.0f, 1e-4f);
    EXPECT_NEAR(mid.ribbon.margin_local_m,
                (dark->ribbon.margin_local_m + light->ribbon.margin_local_m) / 2.0f, 1e-4f);
}

TEST(ThemePalette, RibbonMarginVelocityFallsBackToOnePointZeroFiveWhenMissingFromYaml) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing the optional margin_velocity_m key must still parse";
    EXPECT_NEAR(theme->ribbon.margin_velocity_m, 1.05f, 1e-4f);
}

TEST(ThemePalette, RibbonMarginVelocityLerpsLinearlyAcrossTransition) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const overlume::detail::Theme mid = overlume::detail::blend(*dark, *light, 0.5f);
    EXPECT_NEAR(mid.ribbon.margin_velocity_m,
                (dark->ribbon.margin_velocity_m + light->ribbon.margin_velocity_m) / 2.0f, 1e-4f);
}

TEST(ThemePalette, RibbonMarginVelocityParsesExplicitYamlValue) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "ribbon_margin_velocity");
    ASSERT_TRUE(theme.has_value());
    EXPECT_NEAR(theme->ribbon.margin_velocity_m, 0.9f, 1e-4f)
        << "an explicit ribbon.margin_velocity_m key must override the 1.05 soft default";
}

TEST(ThemePalette, ShippedThemesFallBackMarginVelocityBetweenLocalAndBehavior) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    for (const auto* t : {&*dark, &*light}) {
        EXPECT_NEAR(t->ribbon.margin_velocity_m, 1.05f, 1e-4f);
        EXPECT_LT(t->ribbon.margin_local_m, t->ribbon.margin_velocity_m);
        EXPECT_LT(t->ribbon.margin_velocity_m, t->ribbon.margin_behavior_m);
    }
}

TEST(ThemePalette, ShippedThemesAuthorTheFillLookExplicitly) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    for (const auto* t : {&*dark, &*light}) {
        EXPECT_NEAR(t->ribbon.lane_width_m, 3.5f, 1e-4f);
        EXPECT_NEAR(t->ribbon.margin_global_m, 0.3f, 1e-4f);
        EXPECT_NEAR(t->ribbon.margin_local_m, 0.8f, 1e-4f);
        EXPECT_NEAR(t->ribbon.margin_behavior_m, 1.3f, 1e-4f);
        EXPECT_LT(t->ribbon.margin_global_m, t->ribbon.margin_local_m);
        EXPECT_LT(t->ribbon.margin_local_m, t->ribbon.margin_behavior_m);
    }
}

TEST(ThemeRibbonFade, ShippedThemesEnableOpacityAndLengthFade) {
    for (const char* name : {"dark_adas", "light_clay"}) {
        const std::optional<overlume::detail::Theme> theme =
            overlume::detail::load_theme(kThemeDir, name);
        ASSERT_TRUE(theme.has_value()) << name;
        EXPECT_GT(theme->ribbon.opacity, 0.0f) << name;
        EXPECT_LE(theme->ribbon.opacity, 1.0f) << name;
        EXPECT_GT(theme->ribbon.fade_start_m, 0.0f) << name;
        EXPECT_GT(theme->ribbon.fade_end_m, theme->ribbon.fade_start_m)
            << name << ": shipped themes keep the metre fade enabled (end beyond start)";
    }
}

TEST(ThemeRibbonFade, FixtureParsesExplicitOpacityAndFadeStart) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "ribbon_fade");
    ASSERT_TRUE(theme.has_value());
    EXPECT_NEAR(theme->ribbon.opacity, 0.6f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.fade_start_m, 5.0f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.fade_end_m, 10.0f, 1e-4f);
}

TEST(ThemeObjects, OpacityFallsBackToOnePointZeroWhenMissingFromYaml) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing the whole optional objects: section must still parse";
    EXPECT_NEAR(theme->objects.opacity, 1.0f, 1e-4f);
}

TEST(ThemeObjects, OpacityParsesExplicitYamlValue) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "objects_half_opacity");
    ASSERT_TRUE(theme.has_value());
    EXPECT_NEAR(theme->objects.opacity, 0.5f, 1e-4f)
        << "an explicit objects.opacity key must override the 1.0 soft default";
}

TEST(ThemeObjects, ShippedThemesAuthorOpacityExplicitly) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());
    EXPECT_NEAR(dark->objects.opacity, 0.25f, 1e-4f);
    EXPECT_NEAR(light->objects.opacity, 0.25f, 1e-4f);
}

TEST(ThemeObjects, OpacityLerpsLinearlyAcrossTransition) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> half =
        overlume::detail::load_theme(fixtureDir, "objects_half_opacity");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(half.has_value());

    const overlume::detail::Theme mid = overlume::detail::blend(*dark, *half, 0.5f);
    EXPECT_NEAR(mid.objects.opacity, (dark->objects.opacity + half->objects.opacity) / 2.0f, 1e-4f);
}

TEST(ThemeObjects, OpacityOutOfRangeClampsToUnitInterval) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const auto theme = overlume::detail::load_theme(fixtureDir, "objects_overrange_opacity");
    ASSERT_TRUE(theme.has_value());
    EXPECT_FLOAT_EQ(theme->objects.opacity, 1.0f)
        << "objects.opacity 1.5 must clamp to 1.0, not suppress the staleness fade";
}

TEST(ThemePalette, RoadLaneCenterlineLaneBoundaryCrosswalkFallBackWhenMissingFromYaml) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> theme =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing only the optional road/lane_centerline/lane_boundary/"
           "crosswalk keys must still parse";

    EXPECT_NEAR(theme->palette.road.r, theme->palette.ground.r, 1e-4f);
    EXPECT_NEAR(theme->palette.road.g, theme->palette.ground.g, 1e-4f);
    EXPECT_NEAR(theme->palette.road.b, theme->palette.ground.b, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_centerline.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_centerline.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_centerline.b, theme->palette.lane_paint.b, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_boundary.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_boundary.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_boundary.b, theme->palette.lane_paint.b, 1e-4f);
    EXPECT_NEAR(theme->palette.crosswalk.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.crosswalk.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.crosswalk.b, theme->palette.lane_paint.b, 1e-4f);
    EXPECT_NEAR(theme->palette.road_edge.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.road_edge.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.road_edge.b, theme->palette.lane_paint.b, 1e-4f);
}

TEST(ThemePalette, RoadLaneCenterlineLaneBoundaryBlendInOklabAcrossTransition) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const overlume::detail::Theme mid = overlume::detail::blend(*dark, *light, 0.5f);

    for (const auto& [a, b, m] :
         {std::tuple{dark->palette.road, light->palette.road, mid.palette.road},
          std::tuple{dark->palette.lane_centerline, light->palette.lane_centerline,
                     mid.palette.lane_centerline},
          std::tuple{dark->palette.lane_boundary, light->palette.lane_boundary,
                     mid.palette.lane_boundary},
          std::tuple{dark->palette.road_edge, light->palette.road_edge, mid.palette.road_edge}}) {
        const float La = overlume::detail::linear_srgb_to_oklab(a).L;
        const float Lb = overlume::detail::linear_srgb_to_oklab(b).L;
        const float Lmid = overlume::detail::linear_srgb_to_oklab(m).L;
        EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
        EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);
        EXPECT_GT(std::abs(Lmid - La), 1e-4f);
        EXPECT_GT(std::abs(Lmid - Lb), 1e-4f);
    }
}

TEST(ThemeLoad, BuiltinFallbackMatchesDarkAdasYaml) {
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(dark.has_value());
    const overlume::detail::Theme& fb = overlume::detail::kFallbackTheme();

    const auto near3 = [](const overlume::detail::Float3& a, const overlume::detail::Float3& b) {
        EXPECT_NEAR(a.r, b.r, 1e-4f);
        EXPECT_NEAR(a.g, b.g, 1e-4f);
        EXPECT_NEAR(a.b, b.b, 1e-4f);
    };
    EXPECT_EQ(fb.name, dark->name);
    near3(fb.palette.ground, dark->palette.ground);
    near3(fb.palette.sky, dark->palette.sky);
    near3(fb.palette.fog, dark->palette.fog);
    near3(fb.palette.lane_paint, dark->palette.lane_paint);
    near3(fb.palette.ribbon_core, dark->palette.ribbon_core);
    near3(fb.palette.ribbon_glow, dark->palette.ribbon_glow);
    near3(fb.palette.ego, dark->palette.ego);
    near3(fb.palette.ribbon_global, dark->palette.ribbon_global);
    near3(fb.palette.ribbon_local, dark->palette.ribbon_local);
    near3(fb.palette.road, dark->palette.road);
    near3(fb.palette.lane_centerline, dark->palette.lane_centerline);
    near3(fb.palette.lane_boundary, dark->palette.lane_boundary);
    near3(fb.palette.crosswalk, dark->palette.crosswalk);
    near3(fb.palette.road_edge, dark->palette.road_edge);
    near3(fb.palette.object_tints.car, dark->palette.object_tints.car);
    near3(fb.palette.object_tints.truck_van, dark->palette.object_tints.truck_van);
    near3(fb.palette.object_tints.bus, dark->palette.object_tints.bus);
    near3(fb.palette.object_tints.pedestrian, dark->palette.object_tints.pedestrian);
    near3(fb.palette.object_tints.cyclist, dark->palette.object_tints.cyclist);
    near3(fb.palette.object_tints.unknown, dark->palette.object_tints.unknown);
    near3(fb.palette.alert.info, dark->palette.alert.info);
    near3(fb.palette.alert.warning, dark->palette.alert.warning);
    near3(fb.palette.alert.critical, dark->palette.alert.critical);
    EXPECT_NEAR(fb.material.roughness, dark->material.roughness, 1e-4f);
    EXPECT_NEAR(fb.material.metallic, dark->material.metallic, 1e-4f);
    EXPECT_NEAR(fb.emissive.ribbon_strength, dark->emissive.ribbon_strength, 1e-4f);
    near3(fb.grid.line_color, dark->grid.line_color);
    EXPECT_NEAR(fb.grid.fade_start_m, dark->grid.fade_start_m, 1e-4f);
    EXPECT_NEAR(fb.grid.fade_end_m, dark->grid.fade_end_m, 1e-4f);
    near3(fb.hud.text_color, dark->hud.text_color);
    near3(fb.hud.accent_color, dark->hud.accent_color);
    EXPECT_NEAR(fb.hud.scale, dark->hud.scale, 1e-4f);
    near3(fb.sun.direction, dark->sun.direction);
    near3(fb.sun.color, dark->sun.color);
    EXPECT_NEAR(fb.sun.intensity, dark->sun.intensity, 1e-4f);
    near3(fb.ibl.sky_color, dark->ibl.sky_color);
    near3(fb.ibl.ground_color, dark->ibl.ground_color);
    EXPECT_NEAR(fb.ibl.intensity, dark->ibl.intensity, 1e-4f);
    EXPECT_NEAR(fb.fog.density, dark->fog.density, 1e-4f);
    EXPECT_NEAR(fb.ribbon.width_m, dark->ribbon.width_m, 1e-4f);
    EXPECT_NEAR(fb.ribbon.lane_width_m, dark->ribbon.lane_width_m, 1e-4f);
    EXPECT_NEAR(fb.ribbon.margin_behavior_m, dark->ribbon.margin_behavior_m, 1e-4f);
    EXPECT_NEAR(fb.ribbon.margin_global_m, dark->ribbon.margin_global_m, 1e-4f);
    EXPECT_NEAR(fb.ribbon.margin_local_m, dark->ribbon.margin_local_m, 1e-4f);
}

TEST(ClayMaterial, RespondsToLightDirection) {
    constexpr uint32_t kWidth = 320, kHeight = 240;
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfgA{kWidth, kHeight, 1, fixtureDir.c_str(), "sun_dir_a"};
    overlume::RenderConfig cfgB{kWidth, kHeight, 1, fixtureDir.c_str(), "sun_dir_b"};

    overlume::VisualRenderer* rA = overlume::create_renderer(cfgA);
    if (rA == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::VisualRenderer* rB = overlume::create_renderer(cfgB);
    ASSERT_NE(rB, nullptr);

    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> pixelsA(static_cast<size_t>(kWidth) * kHeight * 3);
    std::vector<uint8_t> pixelsB(static_cast<size_t>(kWidth) * kHeight * 3);
    overlume::FrameView viewA{pixelsA.data(), kWidth, kHeight};
    overlume::FrameView viewB{pixelsB.data(), kWidth, kHeight};

    ASSERT_TRUE(overlume::render_frame(rA, pose, viewA));
    ASSERT_TRUE(overlume::render_frame(rB, pose, viewB));

    EXPECT_TRUE(AnyDiffer(pixelsA, pixelsB))
        << "sun_dir_a and sun_dir_b (identical themes except sun.direction) "
           "rendered identical pixels -- clay.mat isn't actually responding "
           "to the sun's direction.";

    overlume::destroy_renderer(rA);
    overlume::destroy_renderer(rB);
}

TEST(Fog, ColorAffectsRenderedOutput) {
    constexpr uint32_t kWidth = 320, kHeight = 240;
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfgA{kWidth, kHeight, 1, fixtureDir.c_str(), "fog_color_black"};
    overlume::RenderConfig cfgB{kWidth, kHeight, 1, fixtureDir.c_str(), "fog_color_white"};

    overlume::VisualRenderer* rA = overlume::create_renderer(cfgA);
    if (rA == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::VisualRenderer* rB = overlume::create_renderer(cfgB);
    ASSERT_NE(rB, nullptr);

    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    overlume::testing::render_and_compare(rA, pose, "/nonexistent/no_such_golden.png",
                                          OVERLUME_TMP_DIR "/fog_color_black_actual.png");
    overlume::testing::render_and_compare(rB, pose, "/nonexistent/no_such_golden.png",
                                          OVERLUME_TMP_DIR "/fog_color_white_actual.png");

    overlume::testing::FrameStats statsA =
        overlume::testing::analyze_png(OVERLUME_TMP_DIR "/fog_color_black_actual.png");
    overlume::testing::FrameStats statsB =
        overlume::testing::analyze_png(OVERLUME_TMP_DIR "/fog_color_white_actual.png");

    EXPECT_GT(statsB.mean - statsA.mean, 15.0)
        << "black-fog vs white-fog fixtures (identical otherwise) rendered "
           "near-identical mean brightness ("
        << statsA.mean << " vs " << statsB.mean
        << ") -- FogOptions::color isn't reaching the screen.";

    overlume::destroy_renderer(rA);
    overlume::destroy_renderer(rB);
}

TEST(Fog, ColorAffectsRenderedOutput_DarkAdas) {
    constexpr uint32_t kWidth = 320, kHeight = 240;
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfgA{kWidth, kHeight, 1, fixtureDir.c_str(), "fog_color_black_dark"};
    overlume::RenderConfig cfgB{kWidth, kHeight, 1, fixtureDir.c_str(), "fog_color_white_dark"};

    overlume::VisualRenderer* rA = overlume::create_renderer(cfgA);
    if (rA == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::VisualRenderer* rB = overlume::create_renderer(cfgB);
    ASSERT_NE(rB, nullptr);

    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    overlume::testing::render_and_compare(rA, pose, "/nonexistent/no_such_golden.png",
                                          OVERLUME_TMP_DIR "/fog_color_black_dark_actual.png");
    overlume::testing::render_and_compare(rB, pose, "/nonexistent/no_such_golden.png",
                                          OVERLUME_TMP_DIR "/fog_color_white_dark_actual.png");

    overlume::testing::FrameStats statsA =
        overlume::testing::analyze_png(OVERLUME_TMP_DIR "/fog_color_black_dark_actual.png");
    overlume::testing::FrameStats statsB =
        overlume::testing::analyze_png(OVERLUME_TMP_DIR "/fog_color_white_dark_actual.png");

    EXPECT_GT(statsB.mean - statsA.mean, 15.0)
        << "black-fog vs white-fog dark_adas-derived fixtures (identical otherwise) "
           "rendered near-identical mean brightness ("
        << statsA.mean << " vs " << statsB.mean
        << ") -- FogOptions::color isn't reaching the screen "
           "on dark_adas's branch of the color-scale formula.";

    overlume::destroy_renderer(rA);
    overlume::destroy_renderer(rB);
}

TEST(ThemeLoad, MissingThemeDir_FallsBackToBuiltinTheme) {
    overlume::RenderConfig cfg{320, 240, 0, "/nonexistent/theme/dir", "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::SceneGraph scene{};
    overlume::set_scene(r, scene);
    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    EXPECT_FALSE(overlume::theme_assets_loaded(r));
    overlume::destroy_renderer(r);
}

TEST(ThemeLoad, RealAssetsDir_ThemeAssetsLoadedIsTrue) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    EXPECT_TRUE(overlume::theme_assets_loaded(r));
    overlume::destroy_renderer(r);
}

TEST(ThemeGolden, EmptyWorld_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);
    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png",
        OVERLUME_TMP_DIR "/empty_world_dark_adas_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);

    overlume::testing::FrameStats stats =
        overlume::testing::analyze_png(OVERLUME_TMP_DIR "/empty_world_dark_adas_actual.png");
    EXPECT_GT(stats.mean, 20.0) << "frame reads as crushed black";
    EXPECT_LT(stats.mean, 200.0) << "frame reads as clipped white";
    EXPECT_GT(stats.distinct_levels, 15)
        << "too few distinct luminance levels -- grid-vs-ground contrast and "
           "distance fade aren't visible";
    EXPECT_GT(stats.bottom_third_mean, stats.top_third_mean)
        << "sky backdrop is brighter than the sunlit ground";
    EXPECT_LT(std::abs(stats.horizon_row_mean - stats.sky_row_mean), 45.0)
        << "far-field ground (" << stats.horizon_row_mean
        << ") doesn't fade "
           "into the sky ("
        << stats.sky_row_mean << ") -- fog is over/under-scaled";
    overlume::destroy_renderer(r);
}

TEST(ThemeGolden, EmptyWorld_LightClay) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "light_clay"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);
    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/empty_world_light_clay.png",
        OVERLUME_TMP_DIR "/empty_world_light_clay_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);

    overlume::testing::FrameStats stats =
        overlume::testing::analyze_png(OVERLUME_TMP_DIR "/empty_world_light_clay_actual.png");
    EXPECT_GT(stats.mean, 60.0) << "frame reads as crushed black";
    EXPECT_LT(stats.mean, 235.0) << "frame reads as clipped white";
    EXPECT_GT(stats.distinct_levels, 12)
        << "frame luminance has collapsed to near-uniform. NOTE: at this "
           "theme's exposure the grid and distance fade are ALREADY not "
           "visible (the frame sits in a ~176-199 band), so passing this "
           "does NOT prove grid contrast -- it only catches a total "
           "(~2-5 level) collapse";
    EXPECT_LT(std::abs(stats.horizon_row_mean - stats.sky_row_mean), 55.0)
        << "far-field ground (" << stats.horizon_row_mean
        << ") doesn't fade "
           "into the sky ("
        << stats.sky_row_mean << ") -- fog is over/under-scaled";
    overlume::destroy_renderer(r);
}

TEST(ThemeEnvironment, TileRadiusParsesDefaultsAndRejectsNonPositive) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> shipped =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(shipped.has_value());
    EXPECT_GT(shipped->environment.tile_radius_m, 0.0f)
        << "shipped value is a tunable; only positivity is contractual";
    const std::optional<overlume::detail::Theme> tiny =
        overlume::detail::load_theme(fixtureDir, "tile_radius_tiny");
    ASSERT_TRUE(tiny.has_value());
    EXPECT_NEAR(tiny->environment.tile_radius_m, 0.001f, 1e-6f);
    const std::optional<overlume::detail::Theme> absent =
        overlume::detail::load_theme(fixtureDir, "ribbon_margin_a");
    ASSERT_TRUE(absent.has_value());
    EXPECT_NEAR(absent->environment.tile_radius_m, 700.0f, 1e-3f)
        << "a theme without an environment: map keeps the 700 m default";
}

TEST(ThemeRibbonFade, TransitionToAFadeDisabledThemeHoldsTheMetresInsteadOfShrinkingThem) {
    overlume::detail::Theme on;
    on.ribbon.fade_start_m = 20.0f;
    on.ribbon.fade_end_m = 80.0f;
    overlume::detail::Theme off;
    off.ribbon.fade_start_m = 0.0f;
    off.ribbon.fade_end_m = 0.0f;
    const overlume::detail::Theme mid = overlume::detail::blend(on, off, 0.99f);
    EXPECT_NEAR(mid.ribbon.fade_start_m, 20.0f, 1e-4f)
        << "lerping the metres toward 0 would fade everything past 0.8 m at t=0.99";
    EXPECT_NEAR(mid.ribbon.fade_end_m, 80.0f, 1e-4f);
    const overlume::detail::Theme back = overlume::detail::blend(off, on, 0.01f);
    EXPECT_NEAR(back.ribbon.fade_end_m, 80.0f, 1e-4f);
}

TEST(ThemeOgmRamp, BakesStopsLinearlyAndLeavesOutOfRangeTransparent) {
    const auto ramp = overlume::detail::bake_ogm_ramp(
        {{100.0f, {1.0f, 0.0f, 0.0f}, 1.0f}, {50.0f, {0.0f, 0.0f, 1.0f}, 0.5f}});
    EXPECT_FLOAT_EQ(ramp.alpha[0], 0.0f) << "value 0 (free / no value) is always transparent";
    EXPECT_FLOAT_EQ(ramp.alpha[49], 0.0f) << "below the first stop is outside the range";
    EXPECT_FLOAT_EQ(ramp.alpha[50], 0.5f);
    EXPECT_FLOAT_EQ(ramp.color[50].b, 1.0f) << "stops are sorted by value";
    EXPECT_NEAR(ramp.color[75].r, 0.5f, 1e-6f);
    EXPECT_NEAR(ramp.color[75].b, 0.5f, 1e-6f);
    EXPECT_NEAR(ramp.alpha[75], 0.75f, 1e-6f);
    EXPECT_FLOAT_EQ(ramp.color[100].r, 1.0f);
    EXPECT_FLOAT_EQ(ramp.alpha[100], 1.0f);
}

TEST(ThemeOgmRamp, ThemesWithoutAnOgmBlockKeepTheGroundToWarningMix) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> t =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(t.has_value());
    for (uint32_t v : {1u, 37u, 100u}) {
        const float w = static_cast<float>(v) / 100.0f;
        const auto& g = t->palette.ground;
        const auto& o = t->palette.alert.warning;
        for (const auto* ramp : {&t->ogm.dynamic, &t->ogm.geometric}) {
            EXPECT_NEAR(ramp->color[v].r, g.r + (o.r - g.r) * w, 1e-5f) << v;
            EXPECT_NEAR(ramp->color[v].g, g.g + (o.g - g.g) * w, 1e-5f) << v;
            EXPECT_NEAR(ramp->color[v].b, g.b + (o.b - g.b) * w, 1e-5f) << v;
            EXPECT_FLOAT_EQ(ramp->alpha[v], 1.0f) << v;
        }
    }
}

TEST(ThemeOgmRamp, ShippedThemesGiveDynamicAndGeometricDistinctRamps) {
    for (const char* name : {"dark_adas", "light_clay"}) {
        const std::optional<overlume::detail::Theme> t =
            overlume::detail::load_theme(kThemeDir, name);
        ASSERT_TRUE(t.has_value()) << name;
        for (uint32_t v : {1u, 100u}) {
            const auto d = overlume::detail::linear_srgb_to_oklab(t->ogm.dynamic.color[v]);
            const auto g = overlume::detail::linear_srgb_to_oklab(t->ogm.geometric.color[v]);
            const float dist = std::sqrt((d.L - g.L) * (d.L - g.L) + (d.a - g.a) * (d.a - g.a) +
                                         (d.b - g.b) * (d.b - g.b));
            EXPECT_GT(dist, 0.08f) << name << ": dynamic and geometric must be perceptually "
                                   << "distinct at v " << v << " (Oklab distance " << dist << ")";
        }
        EXPECT_LT(t->ogm.dynamic.alpha[1], t->ogm.dynamic.alpha[100])
            << name << ": low values are shaded lighter than lethal ones";
        EXPECT_FLOAT_EQ(t->ogm.dynamic.alpha[0], 0.0f) << name;
    }
}

TEST(ThemeOgmRamp, RampsContrastWithTheirThemeGround) {
    const auto distance = [](const overlume::detail::Oklab& p, const overlume::detail::Oklab& q) {
        return std::sqrt((p.L - q.L) * (p.L - q.L) + (p.a - q.a) * (p.a - q.a) +
                         (p.b - q.b) * (p.b - q.b));
    };
    const std::optional<overlume::detail::Theme> dark =
        overlume::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<overlume::detail::Theme> light =
        overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());
    const auto darkGround = overlume::detail::linear_srgb_to_oklab(dark->palette.ground);
    const auto lightGround = overlume::detail::linear_srgb_to_oklab(light->palette.ground);
    for (uint32_t v : {1u, 50u, 100u}) {
        for (const auto* ramp : {&dark->ogm.dynamic, &dark->ogm.geometric}) {
            const auto c = overlume::detail::linear_srgb_to_oklab(ramp->color[v]);
            EXPECT_GT(c.L, darkGround.L + 0.15f)
                << "dark_adas OGM ramps must be lighter than its dark ground (v " << v << ")";
            EXPECT_GT(distance(c, darkGround), 0.25f)
                << "dark_adas OGM ramp too close to its ground (v " << v << ")";
        }
        for (const auto* ramp : {&light->ogm.dynamic, &light->ogm.geometric}) {
            const auto c = overlume::detail::linear_srgb_to_oklab(ramp->color[v]);
            EXPECT_LT(c.L, lightGround.L - 0.15f)
                << "light_clay OGM ramps must be darker than its light ground (v " << v << ")";
            EXPECT_GT(distance(c, lightGround), 0.25f)
                << "light_clay OGM ramp too close to its ground (v " << v << ")";
        }
    }
}
