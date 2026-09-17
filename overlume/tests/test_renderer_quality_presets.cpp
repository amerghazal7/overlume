// tests/test_renderer_quality_presets.cpp — Epic 3 Task 5 (VM-032) Step 3:
// create_renderer()'s quality dispatch maps spec §8's three preset knobs
// (shadow-map resolution, shadow enable, low-preset render scale) off
// RenderConfig::quality. Filament-free config/test-hook assertions (see
// renderer_quality_test_hooks.hpp) -- not GPU pixel readbacks, same
// "create_renderer() must succeed on this box" convention as
// test_renderer_projection.cpp.
//
// VM-040 (Epic 5) extends this file with set_quality()/get_quality()
// coverage: the SAME preset table applied LIVE, against an
// already-constructed VisualRenderer*, with no renderer re-create (P4
// decision, scene.h's set_quality() comment).
#include "overlume/api.h"
#include "overlume/scene.h"
#include "renderer_quality_test_hooks.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace {

overlume::VisualRenderer* MakeRenderer(uint8_t quality, uint32_t width = 1280, uint32_t height = 720) {
    overlume::RenderConfig config{};
    config.width = width;
    config.height = height;
    config.quality = quality;
    config.theme_assets_dir = nullptr;  // compiled-in fallback theme
    config.initial_theme = nullptr;
    return overlume::create_renderer(config);
}

}  // namespace

TEST(RendererQuality, ShadowMapResolutionMatchesPresetTable) {
    overlume::VisualRenderer* high = MakeRenderer(2);
    ASSERT_NE(high, nullptr);
    EXPECT_EQ(overlume::testing::quality_shadow_map_size(high), 2048u);
    overlume::destroy_renderer(high);

    overlume::VisualRenderer* medium = MakeRenderer(1);
    ASSERT_NE(medium, nullptr);
    EXPECT_EQ(overlume::testing::quality_shadow_map_size(medium), 1024u);
    overlume::destroy_renderer(medium);
}

TEST(RendererQuality, ShadowsDisabledAtLowPreset) {
    overlume::VisualRenderer* low = MakeRenderer(0);
    ASSERT_NE(low, nullptr);
    EXPECT_FALSE(overlume::testing::quality_shadows_enabled(low));
    overlume::destroy_renderer(low);

    // Medium/high both cast shadows -- only low disables them.
    overlume::VisualRenderer* medium = MakeRenderer(1);
    ASSERT_NE(medium, nullptr);
    EXPECT_TRUE(overlume::testing::quality_shadows_enabled(medium));
    overlume::destroy_renderer(medium);
}

TEST(RendererQuality, LowPresetRendersAtUpscaledRenderScale) {
    overlume::VisualRenderer* low = MakeRenderer(0, 1280, 720);
    ASSERT_NE(low, nullptr);
    const overlume::testing::QualityRenderSize size = overlume::testing::quality_internal_render_size(low);
    EXPECT_EQ(size.width, 960u);
    EXPECT_EQ(size.height, 540u);
    overlume::destroy_renderer(low);

    // Medium/high render at the requested output size -- no internal
    // downscale.
    overlume::VisualRenderer* high = MakeRenderer(2, 1280, 720);
    ASSERT_NE(high, nullptr);
    const overlume::testing::QualityRenderSize highSize = overlume::testing::quality_internal_render_size(high);
    EXPECT_EQ(highSize.width, 1280u);
    EXPECT_EQ(highSize.height, 720u);
    overlume::destroy_renderer(high);
}

// VM-040: set_quality() must move every one of the three preset knobs this
// file already pins for create_renderer() -- live, on the SAME pointer, with
// no destroy_renderer()/create_renderer() round trip anywhere in the test.
TEST(RendererQuality, SetQualitySwitchesLivePresetsWithoutRecreate) {
    overlume::VisualRenderer* r = MakeRenderer(2);  // start high
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(overlume::testing::quality_shadow_map_size(r), 2048u);
    EXPECT_TRUE(overlume::testing::quality_shadows_enabled(r));
    EXPECT_TRUE(overlume::testing::quality_ssao(r).enabled);
    EXPECT_FLOAT_EQ(overlume::testing::quality_ssao(r).resolution, 1.0f);
    EXPECT_EQ(overlume::testing::quality_antialiasing(r), overlume::testing::QualityAntiAliasing::NONE);
    EXPECT_TRUE(overlume::testing::quality_taa_enabled(r));

    overlume::set_quality(r, 0);  // drop to low -- same renderer, no re-create
    EXPECT_FALSE(overlume::testing::quality_shadows_enabled(r));
    EXPECT_EQ(overlume::testing::quality_shadow_map_size(r), 1024u);
    const overlume::testing::QualityRenderSize low = overlume::testing::quality_internal_render_size(r);
    EXPECT_EQ(low.width, 960u);
    EXPECT_EQ(low.height, 540u);
    EXPECT_FALSE(overlume::testing::quality_ssao(r).enabled);
    EXPECT_EQ(overlume::testing::quality_antialiasing(r), overlume::testing::QualityAntiAliasing::FXAA);
    EXPECT_FALSE(overlume::testing::quality_taa_enabled(r));

    overlume::set_quality(r, 2);  // recover to high -- still the same renderer
    EXPECT_TRUE(overlume::testing::quality_shadows_enabled(r));
    EXPECT_EQ(overlume::testing::quality_shadow_map_size(r), 2048u);
    const overlume::testing::QualityRenderSize high = overlume::testing::quality_internal_render_size(r);
    EXPECT_EQ(high.width, 1280u);
    EXPECT_EQ(high.height, 720u);
    EXPECT_TRUE(overlume::testing::quality_ssao(r).enabled);
    EXPECT_FLOAT_EQ(overlume::testing::quality_ssao(r).resolution, 1.0f);
    EXPECT_EQ(overlume::testing::quality_antialiasing(r), overlume::testing::QualityAntiAliasing::NONE);
    EXPECT_TRUE(overlume::testing::quality_taa_enabled(r));

    overlume::destroy_renderer(r);
}

// AC (VM-040): render_frame() keeps working across a live preset switch --
// the whole point of the appended-entry-point decision over a re-create.
TEST(RendererQuality, RenderFrameKeepsWorkingAcrossALiveQualitySwitch) {
    overlume::VisualRenderer* r = MakeRenderer(1, 1280, 720);
    ASSERT_NE(r, nullptr);

    overlume::CameraPose pose{{-8.0, -12.0, 8.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> rgb(1280u * 720u * 3);
    overlume::FrameView view{rgb.data(), 1280, 720};

    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    overlume::set_quality(r, 0);
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    overlume::set_quality(r, 2);
    EXPECT_TRUE(overlume::render_frame(r, pose, view));

    overlume::destroy_renderer(r);
}

// AC (VM-040): "a hook mirrors the active preset" -- get_quality() reads
// back exactly what create_renderer()/set_quality() last applied, clamped
// the same way set_quality() itself clamps an out-of-range preset.
TEST(RendererQuality, GetQualityMirrorsTheActivePreset) {
    overlume::VisualRenderer* r = MakeRenderer(1);
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(overlume::get_quality(r), 1u);

    overlume::set_quality(r, 0);
    EXPECT_EQ(overlume::get_quality(r), 0u);

    overlume::set_quality(r, 99);  // above 2 clamps to high, same as api.h's contract
    EXPECT_EQ(overlume::get_quality(r), 2u);

    overlume::destroy_renderer(r);
}

TEST(RendererQuality, SetQualityAndGetQualityAreNoOpsOnNullRenderer) {
    overlume::set_quality(nullptr, 0);  // must not crash
    EXPECT_EQ(overlume::get_quality(nullptr), 0u);
}
