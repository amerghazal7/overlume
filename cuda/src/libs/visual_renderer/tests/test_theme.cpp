// test_theme.cpp — Epic 1 Task 2 (VM-011): theme system on a real lit
// pipeline + golden-image harness
// (docs/superpowers/plans/2026-08-18-visual-mode-epic1.md).
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

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

bool AnyDiffer(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    return a != b;
}

}  // namespace

// ── palette.ego (user contrast directive 2026-08-20) ────────────────────
// Originally authored as a cross-theme swap in BOTH directions:
// dark_adas.yaml's `ego` key authored light_clay's THEN-current ground
// color and vice versa, so the ego always read against whichever ground it
// was standing on.
//
// ITEM 2 (user directive 2026-08-20, light_clay holistic re-authoring
// against ref-2) broke the dark_adas -> light_clay half of that swap ON
// PURPOSE: light_clay.palette.ground moved from the old near-white
// [0.82, 0.80, 0.76] to a road-toned gray (~[0.58, 0.58, 0.64]) to fix
// ref-2's value separation, but dark_adas.yaml's `ego` deliberately did NOT
// follow it -- dark_adas's own shipped goldens (MapGolden.
// LaneNetworkAtEgoOffset_DarkAdas among them) render dark_adas's ego
// fallback box in that exact color, and ITEM 2's OWN constraint is "dark
// goldens must stay green" (ribbons_three_roles_dark_adas is the one
// stated exception, for an unrelated reason -- ITEM 1's ribbon role
// split). dark_adas.ego is therefore now its own independently-authored
// contrast color, not a live mirror of light_clay.ground -- still miles
// away from dark_adas's own near-black ground, which is all the "pops
// against its own ground" contract ever required.
//
// The light_clay -> dark_adas half is untouched (dark_adas.palette.ground
// didn't change), so it still holds and is still asserted below.

TEST(ThemePalette, EgoParsesFromYamlAsCrossThemeSwap) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    // light_clay.ego == dark_adas.ground (both authored [0.05, 0.06, 0.08])
    // -- this half of the swap is untouched by ITEM 2.
    EXPECT_NEAR(light->palette.ego.r, dark->palette.ground.r, 1e-4f);
    EXPECT_NEAR(light->palette.ego.g, dark->palette.ground.g, 1e-4f);
    EXPECT_NEAR(light->palette.ego.b, dark->palette.ground.b, 1e-4f);

    // dark_adas.ego is no longer light_clay.ground (see this test's own
    // comment above) -- it must still be a bright, clearly-non-dark color
    // so it keeps popping against dark_adas's own near-black ground,
    // whatever light_clay does with its own palette.
    const float darkEgoLightness = mpviz::detail::linear_srgb_to_oklab(dark->palette.ego).L;
    const float darkGroundLightness = mpviz::detail::linear_srgb_to_oklab(dark->palette.ground).L;
    EXPECT_GT(darkEgoLightness, darkGroundLightness + 0.3f)
        << "dark_adas's ego color no longer contrasts against its own ground";
}

TEST(ThemePalette, EgoFallsBackToBuiltinDefaultWhenMissingFromYaml) {
    // tests/fixtures/themes/sun_dir_a.yaml predates palette.ego and was
    // deliberately NOT updated to add it (this task's scope) -- proving
    // `ego` is the one OPTIONAL palette key (theme.cpp's parse(), unlike
    // every required field which would instead throw and fall the whole
    // theme back to kFallbackTheme() -- see this test's own has_value()
    // assertion below, which would fail if it still threw).
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing only the optional `ego` key must still parse";
    EXPECT_NEAR(theme->palette.ego.r, 0.82f, 1e-4f);
    EXPECT_NEAR(theme->palette.ego.g, 0.80f, 1e-4f);
    EXPECT_NEAR(theme->palette.ego.b, 0.76f, 1e-4f);
}

TEST(ThemePalette, EgoBlendsInOklabAcrossTransition) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const mpviz::detail::Theme mid = mpviz::detail::blend(*dark, *light, 0.5f);

    // Self-consistency, same technique as ThemeTransition.MidpointBlend_
    // IsBetweenEndpointsInOklab (test_theme_transition.cpp): the blended
    // ego's Oklab lightness must sit between the two endpoints'.
    const float La = mpviz::detail::linear_srgb_to_oklab(dark->palette.ego).L;
    const float Lb = mpviz::detail::linear_srgb_to_oklab(light->palette.ego).L;
    const float Lmid = mpviz::detail::linear_srgb_to_oklab(mid.palette.ego).L;
    EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
    EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);

    // And it must actually have moved off both endpoints -- guards against
    // blend() silently skipping palette.ego (e.g. a copy-paste that left
    // `out.palette.ego` default-constructed / equal to `a`'s value).
    EXPECT_GT(std::abs(Lmid - La), 1e-4f);
    EXPECT_GT(std::abs(Lmid - Lb), 1e-4f);
}

// ── palette.ribbon_global/ribbon_local + ribbon.width_m (user directive
//    2026-08-20, ITEM 1): three new soft-defaulted tokens ─────────────────

TEST(ThemePalette, RibbonGlobalLocalAndWidthFallBackToTodaysValuesWhenMissingFromYaml) {
    // sun_dir_a.yaml predates ribbon_global/ribbon_local/the whole `ribbon:`
    // section and was deliberately NOT updated to add them (same "prove the
    // soft default, don't retrofit every old fixture" reasoning as palette.
    // ego's own EgoFallsBackToBuiltinDefaultWhenMissingFromYaml above).
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing only the optional ribbon_global/ribbon_local/ribbon keys "
           "must still parse";

    // Missing ribbon_global/ribbon_local fall back to ribbon_core/ribbon_glow
    // respectively -- today's reused-token look (renderer.cpp's old
    // push_theme_to_scene() comment), not a new invented color.
    EXPECT_NEAR(theme->palette.ribbon_global.r, theme->palette.ribbon_core.r, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_global.g, theme->palette.ribbon_core.g, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_global.b, theme->palette.ribbon_core.b, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_local.r, theme->palette.ribbon_glow.r, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_local.g, theme->palette.ribbon_glow.g, 1e-4f);
    EXPECT_NEAR(theme->palette.ribbon_local.b, theme->palette.ribbon_glow.b, 1e-4f);

    // Missing `ribbon:` (or just `width_m` within it) falls back to 0.24 --
    // 2*kRibbonHalfWidthM, the constant this field replaced.
    EXPECT_NEAR(theme->ribbon.width_m, 0.24f, 1e-4f);
}

TEST(ThemePalette, RibbonGlobalLocalBlendInOklabAcrossTransition) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const mpviz::detail::Theme mid = mpviz::detail::blend(*dark, *light, 0.5f);

    // Same self-consistency + "actually moved off both endpoints" technique
    // as palette.ego's own EgoBlendsInOklabAcrossTransition above, applied to
    // both new ribbon tokens.
    for (const auto& [a, b, m] :
         {std::tuple{dark->palette.ribbon_global, light->palette.ribbon_global,
                     mid.palette.ribbon_global},
          std::tuple{dark->palette.ribbon_local, light->palette.ribbon_local,
                     mid.palette.ribbon_local}}) {
        const float La = mpviz::detail::linear_srgb_to_oklab(a).L;
        const float Lb = mpviz::detail::linear_srgb_to_oklab(b).L;
        const float Lmid = mpviz::detail::linear_srgb_to_oklab(m).L;
        EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
        EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);
        EXPECT_GT(std::abs(Lmid - La), 1e-4f);
        EXPECT_GT(std::abs(Lmid - Lb), 1e-4f);
    }
}

TEST(ThemePalette, RibbonWidthLerpsLinearlyAcrossTransition) {
    // A plain scalar lerp (theme_transition.cpp's blend()), not Oklab -- so
    // the midpoint must land at EXACTLY the arithmetic mean, unlike the
    // color tokens above.
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const mpviz::detail::Theme mid = mpviz::detail::blend(*dark, *light, 0.5f);
    const float expectedMid = (dark->ribbon.width_m + light->ribbon.width_m) / 2.0f;
    EXPECT_NEAR(mid.ribbon.width_m, expectedMid, 1e-4f);
}

// ── ribbon.lane_width_m/margin_{behavior,global,local}_m (user directive
//    2026-09-08, ITEM 3: lane-fill margins) -- four new soft-defaulted,
//    linearly-blended scalars ────────────────────────────────────────────

TEST(ThemePalette, RibbonLaneWidthAndMarginsFallBackToTheWidthMSeedWhenMissingFromYaml) {
    // sun_dir_a.yaml predates lane_width_m/margin_*_m entirely (same "prove
    // the soft default, don't retrofit every old fixture" reasoning as
    // every other soft-defaulted token above) and has no `ribbon:` section
    // at all -- so width_m ALSO falls back to its own 0.24 default, and the
    // margin default is computed from THAT: (3.5 - 0.24) / 2 == 1.63.
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing the optional lane_width_m/margin_*_m keys must still parse";

    EXPECT_NEAR(theme->ribbon.lane_width_m, 3.5f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.margin_behavior_m, 1.63f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.margin_global_m, 1.63f, 1e-4f);
    EXPECT_NEAR(theme->ribbon.margin_local_m, 1.63f, 1e-4f);
}

TEST(ThemePalette, RibbonLaneWidthAndMarginsLerpLinearlyAcrossTransition) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const mpviz::detail::Theme mid = mpviz::detail::blend(*dark, *light, 0.5f);
    // Plain scalar lerps, same as ribbon.width_m above -- not colors.
    EXPECT_NEAR(mid.ribbon.lane_width_m, (dark->ribbon.lane_width_m + light->ribbon.lane_width_m) / 2.0f,
                1e-4f);
    EXPECT_NEAR(mid.ribbon.margin_behavior_m,
                (dark->ribbon.margin_behavior_m + light->ribbon.margin_behavior_m) / 2.0f, 1e-4f);
    EXPECT_NEAR(mid.ribbon.margin_global_m,
                (dark->ribbon.margin_global_m + light->ribbon.margin_global_m) / 2.0f, 1e-4f);
    EXPECT_NEAR(mid.ribbon.margin_local_m,
                (dark->ribbon.margin_local_m + light->ribbon.margin_local_m) / 2.0f, 1e-4f);
}

TEST(ThemePalette, ShippedThemesAuthorTheFillLookExplicitly) {
    // Both shipped themes drop the old width_m key and author the fill look
    // directly (user directive 2026-09-08): lane_width_m 3.5, margins
    // GLOBAL 0.2 (widest) < LOCAL 0.5 < BEHAVIOR 0.8 (narrowest, on top of
    // the existing z-stagger) -- proves the shipped YAMLs actually parsed
    // these explicit values, not silently falling back to the width_m-seed
    // default (which would instead read 1.63 for every role).
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    for (const auto* t : {&*dark, &*light}) {
        EXPECT_NEAR(t->ribbon.lane_width_m, 3.5f, 1e-4f);
        // Widened 2026-09-08 ("make the margins bit bigger by default I
        // can't clearly see the 3 ribbons stacked when I play the bag").
        EXPECT_NEAR(t->ribbon.margin_global_m, 0.3f, 1e-4f);
        EXPECT_NEAR(t->ribbon.margin_local_m, 0.8f, 1e-4f);
        EXPECT_NEAR(t->ribbon.margin_behavior_m, 1.3f, 1e-4f);
        EXPECT_LT(t->ribbon.margin_global_m, t->ribbon.margin_local_m);
        EXPECT_LT(t->ribbon.margin_local_m, t->ribbon.margin_behavior_m);
    }
}

// ── palette.road/lane_centerline/lane_boundary/crosswalk (Epic 3 Task 1 /
//    VM-036, decision #6): four new soft-defaulted tokens ─────────────────

TEST(ThemePalette, RoadLaneCenterlineLaneBoundaryCrosswalkFallBackWhenMissingFromYaml) {
    // sun_dir_a.yaml predates these four tokens and was deliberately NOT
    // updated to add them (same "prove the soft default, don't retrofit
    // every old fixture" reasoning as palette.ego's own
    // EgoFallsBackToBuiltinDefaultWhenMissingFromYaml above).
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing only the optional road/lane_centerline/lane_boundary/"
           "crosswalk keys must still parse";

    // road falls back to ground -- today's "ground carries the road tone".
    EXPECT_NEAR(theme->palette.road.r, theme->palette.ground.r, 1e-4f);
    EXPECT_NEAR(theme->palette.road.g, theme->palette.ground.g, 1e-4f);
    EXPECT_NEAR(theme->palette.road.b, theme->palette.ground.b, 1e-4f);
    // lane_centerline/lane_boundary/crosswalk fall back to lane_paint --
    // today's "every map element is one stroke color" look.
    EXPECT_NEAR(theme->palette.lane_centerline.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_centerline.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_centerline.b, theme->palette.lane_paint.b, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_boundary.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_boundary.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.lane_boundary.b, theme->palette.lane_paint.b, 1e-4f);
    EXPECT_NEAR(theme->palette.crosswalk.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.crosswalk.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.crosswalk.b, theme->palette.lane_paint.b, 1e-4f);
    // road_edge (user directive 2026-09-08): same soft-default convention,
    // falls back to lane_paint (the palette.ego precedent).
    EXPECT_NEAR(theme->palette.road_edge.r, theme->palette.lane_paint.r, 1e-4f);
    EXPECT_NEAR(theme->palette.road_edge.g, theme->palette.lane_paint.g, 1e-4f);
    EXPECT_NEAR(theme->palette.road_edge.b, theme->palette.lane_paint.b, 1e-4f);
}

TEST(ThemePalette, RoadLaneCenterlineLaneBoundaryBlendInOklabAcrossTransition) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const mpviz::detail::Theme mid = mpviz::detail::blend(*dark, *light, 0.5f);

    // Same self-consistency + "actually moved off both endpoints" technique
    // as palette.ego's own EgoBlendsInOklabAcrossTransition above.
    for (const auto& [a, b, m] :
         {std::tuple{dark->palette.road, light->palette.road, mid.palette.road},
          std::tuple{dark->palette.lane_centerline, light->palette.lane_centerline,
                     mid.palette.lane_centerline},
          std::tuple{dark->palette.lane_boundary, light->palette.lane_boundary,
                     mid.palette.lane_boundary},
          std::tuple{dark->palette.road_edge, light->palette.road_edge, mid.palette.road_edge}}) {
        const float La = mpviz::detail::linear_srgb_to_oklab(a).L;
        const float Lb = mpviz::detail::linear_srgb_to_oklab(b).L;
        const float Lmid = mpviz::detail::linear_srgb_to_oklab(m).L;
        EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
        EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);
        EXPECT_GT(std::abs(Lmid - La), 1e-4f);
        EXPECT_GT(std::abs(Lmid - Lb), 1e-4f);
    }
}

TEST(ThemeLoad, BuiltinFallbackMatchesDarkAdasYaml) {
    // kFallbackTheme() (theme.cpp) is a hand-kept C++ copy of dark_adas.yaml
    // -- the two are supposed to be byte-for-byte the same values, but
    // nothing enforced that before this test, so a future dark_adas.yaml
    // edit could silently drift from what create_renderer() falls back to
    // on a broken/missing theme-asset install (spec §9). Field-by-field,
    // not exhaustive-by-reflection (C++ has none here), but every field
    // this epic actually touches is covered, plus the pre-existing ones.
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(dark.has_value());
    const mpviz::detail::Theme& fb = mpviz::detail::kFallbackTheme();

    const auto near3 = [](const mpviz::detail::Float3& a, const mpviz::detail::Float3& b) {
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
    // Two renderers loaded from fixture themes that are byte-for-byte
    // identical except `sun.direction` (tests/fixtures/themes/sun_dir_{a,b}
    // .yaml), rendering the same static ground+grid scene from the same
    // pose, must NOT produce identical pixels -- proves clay.mat actually
    // responds to the sun's direction, unlike Engine::getDefaultMaterial()
    // and Epic 0's simple_color.mat (both confirmed lighting-independent,
    // "Known Epic 0 deviation" in the plan).
    //
    // This deliberately does NOT compare two different shipped themes
    // (dark_adas vs light_clay): those also differ in palette/fog/IBL, so a
    // completely unlit material rendering two different baseColors would
    // pass that comparison trivially and prove nothing about lighting (see
    // epic1 review). Isolating sun.direction as the only variable is what
    // the plan's Task 2 Step 3 AC actually specifies.
    constexpr uint32_t kWidth = 320, kHeight = 240;
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    mpviz::RenderConfig cfgA{kWidth, kHeight, /*quality=*/1, fixtureDir.c_str(), "sun_dir_a"};
    mpviz::RenderConfig cfgB{kWidth, kHeight, /*quality=*/1, fixtureDir.c_str(), "sun_dir_b"};

    mpviz::VisualRenderer* rA = mpviz::create_renderer(cfgA);
    if (rA == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::VisualRenderer* rB = mpviz::create_renderer(cfgB);
    ASSERT_NE(rB, nullptr);

    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> pixelsA(static_cast<size_t>(kWidth) * kHeight * 3);
    std::vector<uint8_t> pixelsB(static_cast<size_t>(kWidth) * kHeight * 3);
    mpviz::FrameView viewA{pixelsA.data(), kWidth, kHeight};
    mpviz::FrameView viewB{pixelsB.data(), kWidth, kHeight};

    ASSERT_TRUE(mpviz::render_frame(rA, pose, viewA));
    ASSERT_TRUE(mpviz::render_frame(rB, pose, viewB));

    EXPECT_TRUE(AnyDiffer(pixelsA, pixelsB))
        << "sun_dir_a and sun_dir_b (identical themes except sun.direction) "
           "rendered identical pixels -- clay.mat isn't actually responding "
           "to the sun's direction.";

    mpviz::destroy_renderer(rA);
    mpviz::destroy_renderer(rB);
}

TEST(Fog, ColorAffectsRenderedOutput) {
    // Regression test for the epic1 Task 2 review finding: setFogOptions()
    // fed the raw 0-1 authored `palette.fog` straight in as `FogOptions::
    // color`, but that field is scene radiance (Options.h: "a good value is
    // to use the average of the ambient light"), ~5-6 orders of magnitude
    // brighter than 0-1 in this scene's photometric units -- making the
    // token effectively inert (measured: forcing light_clay's fog to pure
    // red moved a golden's far-field row by <=2/255).
    //
    // Two fixtures identical to light_clay -- including its shipped
    // fog.density (0.008); an inflated density would swamp the domain-scale
    // bug with sheer extinction and mask a regression -- except `palette.
    // fog` (black vs. white) must render visibly different mean brightness
    // once the fog color actually reaches the screen. Measured: unfixed
    // code moves the mean by <1/255 here; fixed code moves it by ~90/255.
    // Same "byte-for-byte identical except one field" isolation technique
    // as ClayMaterial.RespondsToLightDirection above.
    constexpr uint32_t kWidth = 320, kHeight = 240;
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    mpviz::RenderConfig cfgA{kWidth, kHeight, /*quality=*/1, fixtureDir.c_str(), "fog_color_black"};
    mpviz::RenderConfig cfgB{kWidth, kHeight, /*quality=*/1, fixtureDir.c_str(), "fog_color_white"};

    mpviz::VisualRenderer* rA = mpviz::create_renderer(cfgA);
    if (rA == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::VisualRenderer* rB = mpviz::create_renderer(cfgB);
    ASSERT_NE(rB, nullptr);

    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    // golden_png_path deliberately doesn't exist -- render_and_compare
    // writes out_png_path unconditionally before checking it, and this test
    // only wants the render, not the (meaningless-here) SSIM return value.
    mpviz::testing::render_and_compare(rA, pose, "/nonexistent/no_such_golden.png",
                                        "/tmp/fog_color_black_actual.png");
    mpviz::testing::render_and_compare(rB, pose, "/nonexistent/no_such_golden.png",
                                        "/tmp/fog_color_white_actual.png");

    mpviz::testing::FrameStats statsA =
        mpviz::testing::analyze_png("/tmp/fog_color_black_actual.png");
    mpviz::testing::FrameStats statsB =
        mpviz::testing::analyze_png("/tmp/fog_color_white_actual.png");

    EXPECT_GT(statsB.mean - statsA.mean, 15.0)
        << "black-fog vs white-fog fixtures (identical otherwise) rendered "
           "near-identical mean brightness (" << statsA.mean
        << " vs " << statsB.mean << ") -- FogOptions::color isn't reaching the screen.";

    mpviz::destroy_renderer(rA);
    mpviz::destroy_renderer(rB);
}

TEST(Fog, ColorAffectsRenderedOutput_DarkAdas) {
    // Regression test for the epic1 Task 2 review round 5 finding: the
    // fixtures above are both derived from light_clay (ibl.intensity 8750)
    // and so only ever exercised ONE branch of setFogOptions()'s per-theme
    // color-scale formula. Round 4's kFogAmbientReferenceIntensitySq/
    // ibl.intensity^2 formula passed the light_clay-only test above while
    // being dead on dark_adas's own branch (ibl.intensity 256000): measured
    // directly, applying the exact same technique as the light_clay test to
    // dark_adas-derived fixtures, round 4's formula moved the mean by only
    // 5.33 (black=41.37, white=46.70) -- below this same test's own >15.0
    // liveness bar. This test closes that coverage gap so the shipped
    // default theme's fog token is actually guarded, not just light_clay's.
    constexpr uint32_t kWidth = 320, kHeight = 240;
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    mpviz::RenderConfig cfgA{kWidth, kHeight, /*quality=*/1, fixtureDir.c_str(),
                             "fog_color_black_dark"};
    mpviz::RenderConfig cfgB{kWidth, kHeight, /*quality=*/1, fixtureDir.c_str(),
                             "fog_color_white_dark"};

    mpviz::VisualRenderer* rA = mpviz::create_renderer(cfgA);
    if (rA == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::VisualRenderer* rB = mpviz::create_renderer(cfgB);
    ASSERT_NE(rB, nullptr);

    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    mpviz::testing::render_and_compare(rA, pose, "/nonexistent/no_such_golden.png",
                                        "/tmp/fog_color_black_dark_actual.png");
    mpviz::testing::render_and_compare(rB, pose, "/nonexistent/no_such_golden.png",
                                        "/tmp/fog_color_white_dark_actual.png");

    mpviz::testing::FrameStats statsA =
        mpviz::testing::analyze_png("/tmp/fog_color_black_dark_actual.png");
    mpviz::testing::FrameStats statsB =
        mpviz::testing::analyze_png("/tmp/fog_color_white_dark_actual.png");

    EXPECT_GT(statsB.mean - statsA.mean, 15.0)
        << "black-fog vs white-fog dark_adas-derived fixtures (identical otherwise) "
           "rendered near-identical mean brightness (" << statsA.mean
        << " vs " << statsB.mean << ") -- FogOptions::color isn't reaching the screen "
           "on dark_adas's branch of the color-scale formula.";

    mpviz::destroy_renderer(rA);
    mpviz::destroy_renderer(rB);
}

TEST(ThemeLoad, MissingThemeDir_FallsBackToBuiltinTheme) {
    mpviz::RenderConfig cfg{320, 240, 0, "/nonexistent/theme/dir", "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        // Only acceptable reason for null here is no GPU/EGL, same skip
        // convention as every other renderer test -- NOT a missing theme dir.
        GTEST_SKIP() << "no GPU/EGL";
    }
    // create_renderer must have succeeded despite the bad theme_assets_dir --
    // rendering one frame with the built-in fallback theme must not crash.
    mpviz::SceneGraph scene{};
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    // Gate-review addition (2026-08-20, spec §9 minor): false here is the
    // whole point of theme_assets_loaded() -- a caller-visible signal that
    // create_renderer() had to substitute the compiled-in fallback, so
    // callers (visualization_node.cpp's on_configure()) can WARN.
    EXPECT_FALSE(mpviz::theme_assets_loaded(r));
    mpviz::destroy_renderer(r);
}

TEST(ThemeLoad, RealAssetsDir_ThemeAssetsLoadedIsTrue) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    EXPECT_TRUE(mpviz::theme_assets_loaded(r));
    mpviz::destroy_renderer(r);
}

TEST(ThemeGolden, EmptyWorld_DarkAdas) {
    // quality=1 (medium: FXAA + SSAO half-res) -- the shipped default (see
    // epic plan's "Conservative perf assumptions"), so the committed golden
    // matches what Step 7a actually ships, not an arbitrary tier.
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::SceneGraph scene{};  // empty: ego.valid=0, every count=0
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);  // caller drives scene state...
    // initial_theme is already "dark_adas" from cfg, so no set_theme() call
    // needed here -- Task 3's transition tests are what exercise mid-blend.
    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = mpviz::testing::render_and_compare(
        r, pose,  // ...harness only renders + SSIMs `r` as-is
        MPVIZ_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png",
        "/tmp/empty_world_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);

    // Legibility ACs from Task 2 Step 7a ("clay surfaces read as mid-gray-
    // ish, not clipped white or crushed black") -- catches the exposure/lux
    // miscalibration that shipped this theme as an effectively-black frame
    // (see epic1 review). Bounds are deliberately loose (this is a
    // legibility floor, not a look-lock -- SSIM above already pins the
    // exact look).
    mpviz::testing::FrameStats stats =
        mpviz::testing::analyze_png("/tmp/empty_world_dark_adas_actual.png");
    EXPECT_GT(stats.mean, 20.0) << "frame reads as crushed black";
    EXPECT_LT(stats.mean, 200.0) << "frame reads as clipped white";
    // 15, re-measured 2026-08-20: the ground patch widened 20m -> 60m (map
    // horizon fix), so most of the frame is now uniform far-ground past the
    // 40m grid-fade end and the user-approved golden legitimately carries 22
    // distinct levels (was >40 with the 40m patch filling the frame with
    // fade gradient). A genuinely lost fade/grid collapses to ~2-5 levels,
    // so the tripwire still fires for the failure it was built to catch.
    EXPECT_GT(stats.distinct_levels, 15)
        << "too few distinct luminance levels -- grid-vs-ground contrast and "
           "distance fade aren't visible";
    // The sunlit ground must read brighter than the flat ambient sky
    // backdrop -- regression guard for "sky 10x brighter than ground".
    EXPECT_GT(stats.bottom_third_mean, stats.top_third_mean)
        << "sky backdrop is brighter than the sunlit ground";
    // dark_adas authors palette.fog == palette.sky (spec §4.3: one 'sky/fog'
    // token) -- the far-field ground just below the horizon should
    // therefore read close to the flat sky backdrop, i.e. the ground fades
    // toward the sky, not into a hard bright band against it (golden.hpp's
    // FrameStats comment has the "why not exact" caveat). The mean-band
    // check above (20 < mean < 200) can't express this: a fog scale that's
    // right for one theme and ~10-15x too hot for this one still lands
    // inside that band (epic1 Task 2 fog-scale review round).
    //
    // Guard loosened 30.0 -> 40.0 (epic1 Task 2 review round 7): round 6's
    // color-scale fix (renderer.cpp's setFogOptions comment) plus the
    // plan's authored density (0.015, restored in dark_adas.yaml -- rounds
    // 5/6 had raised it to 0.10 then 0.03 specifically to buy this guard
    // headroom while a *different* bug, ~1.71x too-bright fog color, was
    // still live) together measure a real, honestly-live gap of ~37.3 here,
    // not the ~31 a stale measurement against golden.cpp's now-corrected
    // [50,60) horizon-row band once suggested. That residual isn't a color/
    // density miscalibration to chase away: dark_adas's ground plane is
    // only 40m across (kGroundHalfExtent), so even the farthest on-plane
    // ray never reaches the near-total fog extinction a true infinite
    // ground would give -- full convergence to sky-row-exact isn't
    // reachable by either knob, and spending density to force it shut is
    // the exact mistake this round undoes. Loosened again 40.0 -> 45.0
    // (Epic 3 Task 1 / VM-036 debt item d): review verified there is no
    // shared bound to "split" between the two themes (dark's 40.0 and
    // light's 55.0 below are, and were before 3b3ce2c, two independent
    // EXPECT_LT calls) -- the debt item's only remaining, optional piece is
    // this one-line parity-of-headroom tweak. Still a real regression
    // guard (round 4's un-scaled fog measured a ~139-level gap; this would
    // still catch that) without demanding the physically unreachable.
    EXPECT_LT(std::abs(stats.horizon_row_mean - stats.sky_row_mean), 45.0)
        << "far-field ground (" << stats.horizon_row_mean << ") doesn't fade "
           "into the sky (" << stats.sky_row_mean << ") -- fog is over/under-scaled";
    mpviz::destroy_renderer(r);
}

TEST(ThemeGolden, EmptyWorld_LightClay) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "light_clay"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = mpviz::testing::render_and_compare(
        r, pose, MPVIZ_TEST_DATA_DIR "/tests/goldens/empty_world_light_clay.png",
        "/tmp/empty_world_light_clay_actual.png");
    EXPECT_GT(ssim, 0.98);

    // Same legibility floor as the dark_adas golden above.
    mpviz::testing::FrameStats stats =
        mpviz::testing::analyze_png("/tmp/empty_world_light_clay_actual.png");
    EXPECT_GT(stats.mean, 60.0) << "frame reads as crushed black";
    EXPECT_LT(stats.mean, 235.0) << "frame reads as clipped white";
    EXPECT_GT(stats.distinct_levels, 40)
        << "too few distinct luminance levels -- grid-vs-ground contrast and "
           "distance fade aren't visible";
    // Same "fog == sky" convergence guard as EmptyWorld_DarkAdas above --
    // light_clay also authors palette.fog == palette.sky. 55, re-measured
    // 2026-08-20 (same precedent as dark's own loosening, renderer.cpp fog
    // history): the ref-2-targeted light re-authoring deliberately runs
    // near-zero fog density (0.0015 -- ref-2 is crisp to the horizon, and
    // 0.008 drowned the whole scene), so the 60m patch's honest residual
    // horizon/sky gap measures ~44.6. Manufacturing fog mass to force it
    // under 30 is exactly the mistake the dark theme's history documents.
    // A genuinely over/under-scaled fog color still trips this at ~55+.
    EXPECT_LT(std::abs(stats.horizon_row_mean - stats.sky_row_mean), 55.0)
        << "far-field ground (" << stats.horizon_row_mean << ") doesn't fade "
           "into the sky (" << stats.sky_row_mean << ") -- fog is over/under-scaled";
    mpviz::destroy_renderer(r);
}
