// tests/test_renderer_quality_presets.cpp — Epic 3 Task 5 (VM-032) Step 3:
// create_renderer()'s quality dispatch maps spec §8's three preset knobs
// (shadow-map resolution, shadow enable, low-preset render scale) off
// RenderConfig::quality. Filament-free config/test-hook assertions (see
// renderer_quality_test_hooks.hpp) -- not GPU pixel readbacks, same
// "create_renderer() must succeed on this box" convention as
// test_renderer_projection.cpp.
#include "visual_renderer/api.h"
#include "renderer_quality_test_hooks.hpp"

#include <gtest/gtest.h>

namespace {

mpviz::VisualRenderer* MakeRenderer(uint8_t quality, uint32_t width = 1280, uint32_t height = 720) {
    mpviz::RenderConfig config{};
    config.width = width;
    config.height = height;
    config.quality = quality;
    config.theme_assets_dir = nullptr;  // compiled-in fallback theme
    config.initial_theme = nullptr;
    return mpviz::create_renderer(config);
}

}  // namespace

TEST(RendererQuality, ShadowMapResolutionMatchesPresetTable) {
    mpviz::VisualRenderer* high = MakeRenderer(2);
    ASSERT_NE(high, nullptr);
    EXPECT_EQ(mpviz::testing::quality_shadow_map_size(high), 2048u);
    mpviz::destroy_renderer(high);

    mpviz::VisualRenderer* medium = MakeRenderer(1);
    ASSERT_NE(medium, nullptr);
    EXPECT_EQ(mpviz::testing::quality_shadow_map_size(medium), 1024u);
    mpviz::destroy_renderer(medium);
}

TEST(RendererQuality, ShadowsDisabledAtLowPreset) {
    mpviz::VisualRenderer* low = MakeRenderer(0);
    ASSERT_NE(low, nullptr);
    EXPECT_FALSE(mpviz::testing::quality_shadows_enabled(low));
    mpviz::destroy_renderer(low);

    // Medium/high both cast shadows -- only low disables them.
    mpviz::VisualRenderer* medium = MakeRenderer(1);
    ASSERT_NE(medium, nullptr);
    EXPECT_TRUE(mpviz::testing::quality_shadows_enabled(medium));
    mpviz::destroy_renderer(medium);
}

TEST(RendererQuality, LowPresetRendersAtUpscaledRenderScale) {
    mpviz::VisualRenderer* low = MakeRenderer(0, 1280, 720);
    ASSERT_NE(low, nullptr);
    const mpviz::testing::QualityRenderSize size = mpviz::testing::quality_internal_render_size(low);
    EXPECT_EQ(size.width, 960u);
    EXPECT_EQ(size.height, 540u);
    mpviz::destroy_renderer(low);

    // Medium/high render at the requested output size -- no internal
    // downscale.
    mpviz::VisualRenderer* high = MakeRenderer(2, 1280, 720);
    ASSERT_NE(high, nullptr);
    const mpviz::testing::QualityRenderSize highSize = mpviz::testing::quality_internal_render_size(high);
    EXPECT_EQ(highSize.width, 1280u);
    EXPECT_EQ(highSize.height, 720u);
    mpviz::destroy_renderer(high);
}
