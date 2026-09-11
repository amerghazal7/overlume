// test_theme.cpp — theme system on a real lit pipeline + golden-image
// harness.
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

// ── palette.ego ──────────────────────────────────────────────────────────
// light_clay.ego is still a live mirror of dark_adas.ground (cross-theme
// swap). dark_adas.ego is NOT a mirror of light_clay.ground -- it's now its
// own independently-authored contrast color, since light_clay's ground
// moved on and dark_adas's own shipped goldens require its ego fallback
// box to keep that exact color. Both are asserted below.

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
    // sun_dir_a.yaml predates palette.ego and was deliberately not updated
    // to add it -- proving `ego` is the one OPTIONAL palette key
    // (theme.cpp's parse()); a required field would instead throw and fall
    // back to kFallbackTheme(), failing the has_value() assertion below.
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

// ── palette.ribbon_global/ribbon_local + ribbon.width_m: soft-defaulted
//    tokens ───────────────────────────────────────────────────────────────

TEST(ThemePalette, RibbonGlobalLocalAndWidthFallBackToTodaysValuesWhenMissingFromYaml) {
    // sun_dir_a.yaml predates ribbon_global/ribbon_local/the whole `ribbon:`
    // section, same "prove the soft default, don't retrofit every old
    // fixture" reasoning as EgoFallsBackToBuiltinDefaultWhenMissingFromYaml.
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

// ── ribbon.lane_width_m/margin_{behavior,global,local}_m: soft-defaulted,
//    linearly-blended scalars ─────────────────────────────────────────────

TEST(ThemePalette, RibbonLaneWidthAndMarginsFallBackToTheWidthMSeedWhenMissingFromYaml) {
    // sun_dir_a.yaml has no `ribbon:` section at all, so width_m ALSO falls
    // back to its own 0.24 default, and the margin default is computed from
    // THAT: (3.5 - 0.24) / 2 == 1.63.
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

// ── ribbon.margin_velocity_m (VM-077 carpet-as-ribbon redirect,
//    2026-09-10): NOT derived from the width_m/marginDefault seed like the
//    three role margins above -- a fixed 1.05 soft default, independent of
//    whatever width_m/lane_width_m a theme authors ─────────────────────────

TEST(ThemePalette, RibbonMarginVelocityFallsBackToOnePointZeroFiveWhenMissingFromYaml) {
    // sun_dir_a.yaml has no `ribbon:` section at all -- unlike
    // margin_{behavior,global,local}_m (which fall back to the width_m-seed
    // formula), margin_velocity_m falls back to its own fixed 1.05,
    // regardless of what width_m/lane_width_m this theme parsed to.
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing the optional margin_velocity_m key must still parse";
    EXPECT_NEAR(theme->ribbon.margin_velocity_m, 1.05f, 1e-4f);
}

TEST(ThemePalette, RibbonMarginVelocityLerpsLinearlyAcrossTransition) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const mpviz::detail::Theme mid = mpviz::detail::blend(*dark, *light, 0.5f);
    EXPECT_NEAR(mid.ribbon.margin_velocity_m,
                (dark->ribbon.margin_velocity_m + light->ribbon.margin_velocity_m) / 2.0f, 1e-4f);
}

TEST(ThemePalette, RibbonMarginVelocityParsesExplicitYamlValue) {
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "ribbon_margin_velocity");
    ASSERT_TRUE(theme.has_value());
    EXPECT_NEAR(theme->ribbon.margin_velocity_m, 0.9f, 1e-4f)
        << "an explicit ribbon.margin_velocity_m key must override the 1.05 soft default";
}

TEST(ThemePalette, ShippedThemesFallBackMarginVelocityBetweenLocalAndBehavior) {
    // Neither shipped theme authors margin_velocity_m on disk -- both read
    // the 1.05 soft default, which sits strictly between margin_local_m
    // (0.8, wider ribbon) and margin_behavior_m (1.3, narrower ribbon) so
    // the velocity ribbon's own fill is narrower than LOCAL's (LOCAL's rim
    // stays visible under it) but wider than BEHAVIOR's (the hero's rim
    // shows through it in turn) -- see theme.hpp's own comment.
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    for (const auto* t : {&*dark, &*light}) {
        EXPECT_NEAR(t->ribbon.margin_velocity_m, 1.05f, 1e-4f);
        EXPECT_LT(t->ribbon.margin_local_m, t->ribbon.margin_velocity_m);
        EXPECT_LT(t->ribbon.margin_velocity_m, t->ribbon.margin_behavior_m);
    }
}

TEST(ThemePalette, ShippedThemesAuthorTheFillLookExplicitly) {
    // Both shipped themes drop the old width_m key and author the fill look
    // directly: lane_width_m 3.5, margins GLOBAL < LOCAL < BEHAVIOR
    // (narrowest, on top of the existing z-stagger) -- proves the shipped
    // YAMLs actually parsed these explicit values, not silently falling
    // back to the width_m-seed default (which would instead read 1.63 for
    // every role).
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
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

// ── objects.opacity: TrackedObject rendering opacity (VM-078) ────────────

TEST(ThemeObjects, OpacityFallsBackToOnePointZeroWhenMissingFromYaml) {
    // sun_dir_a.yaml predates the whole `objects:` section -- same
    // "prove the soft default, don't retrofit every old fixture" reasoning
    // as EgoFallsBackToBuiltinDefaultWhenMissingFromYaml above.
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(theme.has_value())
        << "a theme file missing the whole optional objects: section must still parse";
    EXPECT_NEAR(theme->objects.opacity, 1.0f, 1e-4f);
}

TEST(ThemeObjects, OpacityParsesExplicitYamlValue) {
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> theme =
        mpviz::detail::load_theme(fixtureDir, "objects_half_opacity");
    ASSERT_TRUE(theme.has_value());
    EXPECT_NEAR(theme->objects.opacity, 0.5f, 1e-4f)
        << "an explicit objects.opacity key must override the 1.0 soft default";
}

TEST(ThemeObjects, ShippedThemesAuthorOpacityExplicitly) {
    // AC: both theme YAMLs carry an explicit value (today's fully-opaque
    // look), not a silent fall-through to the soft default.
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());
    EXPECT_NEAR(dark->objects.opacity, 1.0f, 1e-4f);
    EXPECT_NEAR(light->objects.opacity, 1.0f, 1e-4f);
}

TEST(ThemeObjects, OpacityLerpsLinearlyAcrossTransition) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<mpviz::detail::Theme> half =
        mpviz::detail::load_theme(fixtureDir, "objects_half_opacity");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(half.has_value());

    const mpviz::detail::Theme mid = mpviz::detail::blend(*dark, *half, 0.5f);
    EXPECT_NEAR(mid.objects.opacity, (dark->objects.opacity + half->objects.opacity) / 2.0f,
                1e-4f);
}

// ── palette.road/lane_centerline/lane_boundary/crosswalk: soft-defaulted
//    tokens ────────────────────────────────────────────────────────────────

TEST(ThemePalette, RoadLaneCenterlineLaneBoundaryCrosswalkFallBackWhenMissingFromYaml) {
    // sun_dir_a.yaml predates these four tokens, same "prove the soft
    // default, don't retrofit every old fixture" reasoning as
    // EgoFallsBackToBuiltinDefaultWhenMissingFromYaml above.
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
    // road_edge: same soft-default convention, falls back to lane_paint.
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
    // Two renderers loaded from fixture themes byte-for-byte identical
    // except `sun.direction` (sun_dir_{a,b}.yaml), rendering the same
    // static ground+grid scene from the same pose, must NOT produce
    // identical pixels -- proves clay.mat actually responds to the sun's
    // direction. Deliberately does NOT compare two different shipped
    // themes (dark_adas vs light_clay): those also differ in palette/fog/
    // IBL, so a completely unlit material would pass that comparison
    // trivially and prove nothing about lighting.
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
    // Guards setFogOptions() actually feeding `palette.fog` into
    // `FogOptions::color` (scene radiance, Options.h) rather than leaving
    // it effectively inert. Two fixtures identical to light_clay (including
    // its shipped fog.density 0.008 -- an inflated density would mask the
    // regression with sheer extinction) except `palette.fog` (black vs.
    // white) must render visibly different mean brightness. Fixed code
    // moves the mean by ~90/255 here; unfixed by <1/255. Same isolation
    // technique as ClayMaterial.RespondsToLightDirection above.
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
    // Closes a coverage gap: the fixtures in Fog.ColorAffectsRenderedOutput
    // are both derived from light_clay (ibl.intensity 8750) and only
    // exercise one branch of setFogOptions()'s per-theme color-scale
    // formula. This guards dark_adas's own branch (ibl.intensity 256000)
    // the same way, against the same >15.0 liveness bar.
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
    // false here is the whole point of theme_assets_loaded() -- a
    // caller-visible signal that create_renderer() had to substitute the
    // compiled-in fallback, so callers (visualization_node.cpp's
    // on_configure()) can WARN.
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
    // quality=1 (medium: FXAA + SSAO half-res) -- the shipped default, so
    // the committed golden matches what actually ships, not an arbitrary
    // tier.
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

    // Legibility floor ("clay surfaces read as mid-gray-ish, not clipped
    // white or crushed black") -- catches an exposure/lux miscalibration.
    // Bounds are deliberately loose: this is a floor, not a look-lock --
    // SSIM above already pins the exact look.
    mpviz::testing::FrameStats stats =
        mpviz::testing::analyze_png("/tmp/empty_world_dark_adas_actual.png");
    EXPECT_GT(stats.mean, 20.0) << "frame reads as crushed black";
    EXPECT_LT(stats.mean, 200.0) << "frame reads as clipped white";
    // 15: the golden legitimately carries 22 distinct levels with the
    // current 60m ground patch; a genuinely lost fade/grid collapses to
    // ~2-5, so the tripwire still fires for the failure it was built to
    // catch.
    EXPECT_GT(stats.distinct_levels, 15)
        << "too few distinct luminance levels -- grid-vs-ground contrast and "
           "distance fade aren't visible";
    // The sunlit ground must read brighter than the flat ambient sky
    // backdrop -- regression guard for "sky 10x brighter than ground".
    EXPECT_GT(stats.bottom_third_mean, stats.top_third_mean)
        << "sky backdrop is brighter than the sunlit ground";
    // dark_adas authors palette.fog == palette.sky, so the far-field ground
    // just below the horizon should read close to the flat sky backdrop
    // (golden.hpp's FrameStats comment has the "why not exact" caveat). The
    // mean-band check above can't express this: a fog scale ~10-15x too hot
    // still lands inside that band.
    //
    // 45.0: dark_adas's ground plane is only 40m across (kGroundHalfExtent),
    // so even the farthest on-plane ray never reaches near-total fog
    // extinction -- full convergence to sky-row-exact isn't physically
    // reachable, and the honest live gap measures ~37.3. An un-scaled fog
    // color bug measured a ~139-level gap, so this bound is still a real
    // regression guard. See renderer.cpp's setFogOptions comment for the
    // color-scale fix this depends on.
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
    // Same "fog == sky" convergence guard as EmptyWorld_DarkAdas above.
    // 55.0: light_clay deliberately runs a near-zero fog density (0.0015 --
    // ref-2 is crisp to the horizon), so the honest residual horizon/sky
    // gap measures ~44.6; manufacturing fog mass to force it lower would be
    // the mistake dark_adas's own guard avoids. A genuinely over/
    // under-scaled fog color still trips this at ~55+.
    EXPECT_LT(std::abs(stats.horizon_row_mean - stats.sky_row_mean), 55.0)
        << "far-field ground (" << stats.horizon_row_mean << ") doesn't fade "
           "into the sky (" << stats.sky_row_mean << ") -- fog is over/under-scaled";
    mpviz::destroy_renderer(r);
}
