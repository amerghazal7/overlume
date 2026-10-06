// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "height_grid_test_hooks.hpp"
#include "test_paths.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

// theme.height_grid.ground_bias_m of the shipped themes (spec section 8).
constexpr float kBiasM = -0.05f;

void render_once(overlume::VisualRenderer* r) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    overlume::CameraPose pose{{-8, -8, 6}, {0, 0, 0}, 60.0};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
}

overlume::HeightGridLayer make_layer(uint32_t w, uint32_t h, const std::vector<float>& heights,
                                     double last_update_sec) {
    overlume::HeightGridLayer l{};
    l.origin = {0.0, 0.0, 0.0};
    l.yaw_rad = 0.0;
    l.resolution_m = 1.0;
    l.width_cells = w;
    l.height_cells = h;
    l.heights_m = heights.data();
    l.last_update_sec = last_update_sec;
    return l;
}

overlume::SceneGraph make_scene(const overlume::HeightGridLayer* layers, uint32_t n,
                                double sim_time_sec) {
    overlume::SceneGraph s{};
    s.sim_time_sec = sim_time_sec;
    s.height_grids = layers;
    s.height_grid_count = n;
    return s;
}

overlume::VisualRenderer* make_renderer() {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    return overlume::create_renderer(cfg);
}

// Renderer on the Task 2 fixture themes: sun_dir_a has ground_bias_m -0.05 (block absent),
// height_grid_partial has ground_bias_m 0.2 and roughness 0.4.
overlume::VisualRenderer* make_fixture_renderer() {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "sun_dir_a"};
    return overlume::create_renderer(cfg);
}

}

TEST(HeightGrid, VertexAndIndexCountsFollowDimensions) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> a(5 * 4, 0.0f);
    overlume::HeightGridLayer layer = make_layer(5, 4, a, 1.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 1.0);
    overlume::set_scene(r, s);
    render_once(r);
    ASSERT_EQ(overlume::testing::height_grid_slot_count(r), 1u);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 0), 20u);
    EXPECT_EQ(overlume::testing::height_grid_index_count(r, 0), 6u * 4u * 3u);

    std::vector<float> b(3 * 3, 0.0f);
    layer = make_layer(3, 3, b, 2.0);
    s = make_scene(&layer, 1, 2.0);
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 0), 9u);
    EXPECT_EQ(overlume::testing::height_grid_index_count(r, 0), 6u * 2u * 2u);
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, KnownAndUnknownVertexHeightAndCustomAttribute) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 3, 0.25f);
    heights[1 * 3 + 1] = 1.5f;
    heights[0 * 3 + 2] = std::numeric_limits<float>::quiet_NaN();
    overlume::HeightGridLayer layer = make_layer(3, 3, heights, 1.0);
    layer.origin = {10.0, -5.0, 2.0};
    layer.resolution_m = 0.5;
    overlume::SceneGraph s = make_scene(&layer, 1, 1.0);
    overlume::set_scene(r, s);
    render_once(r);

    // Positions come back through the renderable's TransformManager transform (origin) plus the
    // uploaded relative position, so a missing setTransform fails here.
    float pos[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, pos, custom));
    EXPECT_NEAR(pos[0], 10.75f, 1e-4f);
    EXPECT_NEAR(pos[1], -4.25f, 1e-4f);
    EXPECT_NEAR(pos[2], 2.0f + 1.5f + kBiasM, 1e-4f);
    EXPECT_NEAR(custom[0], 1.5f, 1e-6f);
    EXPECT_NEAR(custom[1], 1.0f, 1e-6f);

    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 2, 0, pos, custom));
    EXPECT_NEAR(pos[0], 11.25f, 1e-4f);
    EXPECT_NEAR(pos[1], -4.75f, 1e-4f);
    EXPECT_NEAR(pos[2], 2.0f + kBiasM, 1e-4f);
    EXPECT_NEAR(custom[0], 0.0f, 1e-6f);
    EXPECT_NEAR(custom[1], 0.0f, 1e-6f);

    EXPECT_FALSE(overlume::testing::height_grid_vertex(r, 0, 3, 0, pos, custom));
    EXPECT_FALSE(overlume::testing::height_grid_vertex(r, 1, 0, 0, pos, custom));
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, YawRotatesVertexPositionsAboutTheOrigin) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 2, 0.0f);
    overlume::HeightGridLayer layer = make_layer(3, 2, heights, 1.0);
    layer.origin = {1.0, 2.0, 0.0};
    layer.yaw_rad = 1.5707963267948966;
    overlume::SceneGraph s = make_scene(&layer, 1, 1.0);
    overlume::set_scene(r, s);
    render_once(r);

    // cell (2,0) centre is (2.5, 0.5) in the grid frame -> (-0.5, 2.5) after +90 deg yaw.
    float pos[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 2, 0, pos, custom));
    EXPECT_NEAR(pos[0], 1.0f - 0.5f, 1e-4f);
    EXPECT_NEAR(pos[1], 2.0f + 2.5f, 1e-4f);
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, VertexBufferIsReuploadedOnlyWhenLastUpdateChanges) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(4 * 4, 0.5f);
    overlume::HeightGridLayer layer = make_layer(4, 4, heights, 5.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 5.0);
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u);

    render_once(r);
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u)
        << "unchanged last_update_sec must not re-upload";

    heights.assign(heights.size(), 0.75f);
    layer.last_update_sec = 6.0;
    s.sim_time_sec = 6.0;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 2u);
    float pos[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 0, 0, pos, custom));
    EXPECT_NEAR(custom[0], 0.75f, 1e-6f);
    overlume::destroy_renderer(r);
}

// The state is read from Filament (entity in scene? which material instance is bound?), so
// removing the bind_material / scene add / scene remove calls makes this test fail.
TEST(HeightGrid, StaleFadeSwitchesOpaqueThenFadedThenRemoved) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 3, 0.0f);
    overlume::HeightGridLayer layer = make_layer(3, 3, heights, 10.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 10.0);
    overlume::set_scene(r, s);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 0);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 1);

    s.sim_time_sec = 10.75;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 2);

    s.sim_time_sec = 11.5;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 0);

    s.sim_time_sec = 10.0;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 1)
        << "fresh data must bring the terrain back";
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u);
    overlume::destroy_renderer(r);
}

// The common live case: a short stale gap, then a new grid. The slot must go straight from its
// per-slot faded instance back to the shared opaque one (and release the faded instance).
TEST(HeightGrid, FreshDataAfterFadeReturnsToOpaqueInstance) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 3, 0.0f);
    overlume::HeightGridLayer layer = make_layer(3, 3, heights, 10.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 10.75);
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 2);

    layer.last_update_sec = 10.75;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 1);
    EXPECT_EQ(overlume::testing::height_grid_fade_roughness(r, 0), -1.0f)
        << "the faded instance must be released once the slot is fresh again";
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, SlotCountFollowsLayerCount) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> a(3 * 3, 0.0f), b(4 * 4, 1.0f);
    overlume::HeightGridLayer layers[2] = {make_layer(3, 3, a, 1.0), make_layer(4, 4, b, 1.0)};
    overlume::SceneGraph s = make_scene(layers, 2, 1.0);
    overlume::set_scene(r, s);
    render_once(r);
    ASSERT_EQ(overlume::testing::height_grid_slot_count(r), 2u);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 1), 16u);

    s.height_grid_count = 1;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_slot_count(r), 1u);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 1), 0u);

    s.height_grids = nullptr;
    s.height_grid_count = 0;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_slot_count(r), 0u);
    overlume::destroy_renderer(r);
}

// Spec section 5: a ground_bias_m change re-uploads the vertex buffer (z = h + new bias) and the
// theme push updates the per-slot faded instance too. Run once opaque, once mid-fade.
TEST(HeightGrid, ThemeSwitchChangingGroundBiasReuploads) {
    for (const double sim : {10.0, 10.75}) {
        auto* r = make_fixture_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const bool faded = sim != 10.0;
        const int expectedState = faded ? 2 : 1;

        std::vector<float> heights(3 * 3, 0.5f);
        overlume::HeightGridLayer layer = make_layer(3, 3, heights, 10.0);
        overlume::SceneGraph s = make_scene(&layer, 1, sim);
        overlume::set_scene(r, s);
        render_once(r);

        float pos[3], custom[2];
        EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u) << "sim=" << sim;
        EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), expectedState);
        ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, pos, custom));
        EXPECT_NEAR(pos[2], 0.45f, 1e-5f) << "sun_dir_a: default bias -0.05";
        const float roughnessBefore = overlume::testing::height_grid_fade_roughness(r, 0);
        if (!faded) {
            EXPECT_EQ(roughnessBefore, -1.0f) << "no per-slot faded instance while opaque";
        } else {
            // sun_dir_a has no height_grid block: the roughness default comes from Task 2 and is
            // 0.9 (spec section 8). It must differ from height_grid_partial's 0.4 by > 0.1.
            EXPECT_NEAR(roughnessBefore, 0.9f, 1e-5f)
                << "Task 2's absent-block default roughness is expected to be 0.9";
        }

        // The transition starts one second in the past and lasts 0.2 s, so it has completed when
        // the next frame applies the theme (set_theme would turn a 0 s duration into 0.8 s).
        // Same layer, same last_update_sec; the new theme has ground_bias_m 0.2, roughness 0.4.
        ASSERT_TRUE(overlume::set_theme(r, "height_grid_partial", sim - 1.0, 0.2));
        render_once(r);
        EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 2u)
            << "bias change must re-upload exactly once, sim=" << sim;
        ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, pos, custom));
        EXPECT_NEAR(pos[2], 0.7f, 1e-5f) << "z = h + new bias, sim=" << sim;
        EXPECT_NEAR(custom[0], 0.5f, 1e-6f);
        EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), expectedState)
            << "the slot must keep its material state across the switch";

        const float roughnessAfter = overlume::testing::height_grid_fade_roughness(r, 0);
        if (!faded) {
            EXPECT_EQ(roughnessAfter, -1.0f);
        } else {
            EXPECT_NEAR(roughnessAfter, 0.4f, 1e-5f)
                << "the theme push must reach the per-slot faded instance";
            EXPECT_GT(std::fabs(roughnessAfter - roughnessBefore), 0.1f)
                << "faded-instance roughness did not change across the theme switch";
        }

        render_once(r);
        EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 2u)
            << "no further re-upload once the bias settled";
        overlume::destroy_renderer(r);
    }
}

// Deliberate deviation from spec section 5: the origin is NOT a re-upload trigger. It only moves
// the renderable transform; positions are stored relative to the origin. The moved position is
// read back from the TransformManager, so a missing setTransform fails here.
TEST(HeightGrid, OriginMoveWithoutNewDataDoesNotReupload) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 3, 0.0f);
    overlume::HeightGridLayer layer = make_layer(3, 3, heights, 5.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 5.0);
    overlume::set_scene(r, s);
    render_once(r);

    float before[3], after[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, before, custom));
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u);

    layer.origin = {5.0, -2.0, 0.0};
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u)
        << "origin-only change must not re-upload";
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, after, custom));
    EXPECT_NEAR(after[0] - before[0], 5.0f, 1e-5f);
    EXPECT_NEAR(after[1] - before[1], -2.0f, 1e-5f);
    EXPECT_NEAR(after[2] - before[2], 0.0f, 1e-5f);
    overlume::destroy_renderer(r);
}
