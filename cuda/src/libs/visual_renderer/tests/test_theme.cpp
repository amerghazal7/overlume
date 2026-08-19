// test_theme.cpp — Epic 1 Task 2 (VM-011): theme system on a real lit
// pipeline + golden-image harness
// (docs/superpowers/plans/2026-08-18-visual-mode-epic1.md).
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "golden.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

bool AnyDiffer(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    return a != b;
}

}  // namespace

TEST(ClayMaterial, RespondsToLightDirection) {
    // Two renderers with different themes (different sun direction/color/
    // intensity, different IBL) rendering the same static ground+grid scene
    // from the same pose must NOT produce identical pixels -- proves
    // clay.mat actually responds to scene lighting, unlike
    // Engine::getDefaultMaterial() and Epic 0's simple_color.mat (both
    // confirmed lighting-independent, "Known Epic 0 deviation" in the plan).
    constexpr uint32_t kWidth = 320, kHeight = 240;
    mpviz::RenderConfig cfgDark{kWidth, kHeight, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::RenderConfig cfgLight{kWidth, kHeight, /*quality=*/1, kThemeDir, "light_clay"};

    mpviz::VisualRenderer* darkR = mpviz::create_renderer(cfgDark);
    if (darkR == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::VisualRenderer* lightR = mpviz::create_renderer(cfgLight);
    ASSERT_NE(lightR, nullptr);

    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> darkPixels(static_cast<size_t>(kWidth) * kHeight * 3);
    std::vector<uint8_t> lightPixels(static_cast<size_t>(kWidth) * kHeight * 3);
    mpviz::FrameView darkView{darkPixels.data(), kWidth, kHeight};
    mpviz::FrameView lightView{lightPixels.data(), kWidth, kHeight};

    ASSERT_TRUE(mpviz::render_frame(darkR, pose, darkView));
    ASSERT_TRUE(mpviz::render_frame(lightR, pose, lightView));

    EXPECT_TRUE(AnyDiffer(darkPixels, lightPixels))
        << "dark_adas and light_clay rendered identical pixels -- clay.mat "
           "isn't actually responding to theme-driven lighting.";

    mpviz::destroy_renderer(darkR);
    mpviz::destroy_renderer(lightR);
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
    EXPECT_GT(stats.distinct_levels, 40)
        << "too few distinct luminance levels -- grid-vs-ground contrast and "
           "distance fade aren't visible";
    // The sunlit ground must read brighter than the flat ambient sky
    // backdrop -- regression guard for "sky 10x brighter than ground".
    EXPECT_GT(stats.bottom_third_mean, stats.top_third_mean)
        << "sky backdrop is brighter than the sunlit ground";
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
    mpviz::destroy_renderer(r);
}
