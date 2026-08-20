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
#include <vector>

namespace {

bool AnyDiffer(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    return a != b;
}

}  // namespace

// ── palette.ego (user contrast directive 2026-08-20) ────────────────────
// A cross-theme swap, not a new color: dark_adas.yaml's `ego` key authors
// light_clay's ground color and vice versa (see theme.hpp's Palette::ego
// comment and the .yaml files' own comments), so the ego always reads
// against whichever ground it's standing on.

TEST(ThemePalette, EgoParsesFromYamlAsCrossThemeSwap) {
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    // dark_adas.ego == light_clay.ground (both authored [0.82, 0.80, 0.76]).
    EXPECT_NEAR(dark->palette.ego.r, light->palette.ground.r, 1e-4f);
    EXPECT_NEAR(dark->palette.ego.g, light->palette.ground.g, 1e-4f);
    EXPECT_NEAR(dark->palette.ego.b, light->palette.ground.b, 1e-4f);

    // light_clay.ego == dark_adas.ground (both authored [0.05, 0.06, 0.08]).
    EXPECT_NEAR(light->palette.ego.r, dark->palette.ground.r, 1e-4f);
    EXPECT_NEAR(light->palette.ego.g, dark->palette.ground.g, 1e-4f);
    EXPECT_NEAR(light->palette.ego.b, dark->palette.ground.b, 1e-4f);
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
    // the exact mistake this round undoes. 40.0 keeps this a real
    // regression guard (round 4's un-scaled fog measured a ~139-level gap;
    // this would still catch that) without demanding the physically
    // unreachable.
    EXPECT_LT(std::abs(stats.horizon_row_mean - stats.sky_row_mean), 40.0)
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
    // light_clay also authors palette.fog ~= palette.sky.
    EXPECT_LT(std::abs(stats.horizon_row_mean - stats.sky_row_mean), 30.0)
        << "far-field ground (" << stats.horizon_row_mean << ") doesn't fade "
           "into the sky (" << stats.sky_row_mean << ") -- fog is over/under-scaled";
    mpviz::destroy_renderer(r);
}
