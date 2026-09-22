// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "ground_grid_test_hooks.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<uint8_t> render_once(overlume::VisualRenderer* r, const overlume::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    return pixels;
}

}

TEST(GroundGridGolden, TwoLayers_OffroadLightClay) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "light_clay"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::testing::GridScene grids = overlume::testing::make_two_layer_grids(10.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    s.grids = grids.grids.data();
    s.grid_count = static_cast<uint32_t>(grids.grids.size());
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{-14, -14, 10}, {0, 0, 0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/ogm_offroad_light_clay.png",
        "/tmp/ogm_offroad_light_clay_actual.png");
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
}

TEST(GroundGrid, TextureIsUpdatedInPlaceNotRecreated) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    constexpr uint32_t kW = 4, kH = 4;
    std::vector<uint8_t> cellsA(static_cast<size_t>(kW) * kH, 10);
    overlume::GroundGridLayer layer{};
    layer.kind = 0;
    layer.origin = {0.0, 0.0, 0.0};
    layer.resolution_m = 1.0;
    layer.width_cells = kW;
    layer.height_cells = kH;
    layer.cells = cellsA.data();
    layer.last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.grids = &layer;
    s.grid_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    const void* handleBefore = overlume::testing::ground_grid_texture_handle(r, 0);
    ASSERT_NE(handleBefore, nullptr);

    std::vector<uint8_t> cellsB(static_cast<size_t>(kW) * kH, 90);
    layer.cells = cellsB.data();
    layer.last_update_sec = 2.0;
    s.sim_time_sec = 2.0;
    overlume::set_scene(r, s);
    render_once(r, pose);
    const void* handleAfter = overlume::testing::ground_grid_texture_handle(r, 0);
    EXPECT_EQ(handleBefore, handleAfter)
        << "same-dims content update recreated the texture instead of updating it in place";
    overlume::destroy_renderer(r);
}

TEST(GroundGrid, DimensionChangeRecreatesTheTexture) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<uint8_t> cellsA(4 * 4, 10);
    overlume::GroundGridLayer layer{};
    layer.kind = 0;
    layer.origin = {0.0, 0.0, 0.0};
    layer.resolution_m = 1.0;
    layer.width_cells = 4;
    layer.height_cells = 4;
    layer.cells = cellsA.data();
    layer.last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.grids = &layer;
    s.grid_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    const uint32_t genBefore = overlume::testing::ground_grid_texture_generation(r, 0);
    ASSERT_EQ(genBefore, 1u);

    std::vector<uint8_t> cellsB(8 * 8, 50);
    layer.width_cells = 8;
    layer.height_cells = 8;
    layer.cells = cellsB.data();
    layer.last_update_sec = 2.0;
    s.sim_time_sec = 2.0;
    overlume::set_scene(r, s);
    render_once(r, pose);
    const uint32_t genAfter = overlume::testing::ground_grid_texture_generation(r, 0);
    EXPECT_EQ(genAfter, 2u) << "a dimension change did not destroy+rebuild the texture -- would "
                               "corrupt the upload (byte count no longer matches the texture's "
                               "allocated size)";
    overlume::destroy_renderer(r);
}

TEST(GroundGrid, NoRedundantUploadWithoutNewIngest) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<uint8_t> cells(4 * 4, 10);
    overlume::GroundGridLayer layer{};
    layer.kind = 0;
    layer.origin = {0.0, 0.0, 0.0};
    layer.resolution_m = 1.0;
    layer.width_cells = 4;
    layer.height_cells = 4;
    layer.cells = cells.data();
    layer.last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.grids = &layer;
    s.grid_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};

    render_once(r, pose);
    render_once(r, pose);
    render_once(r, pose);
    EXPECT_EQ(overlume::testing::ground_grid_texture_upload_count(r, 0), 1u)
        << "re-uploaded byte-identical occupancy data on frames with no new ingest";

    std::vector<uint8_t> cellsB(4 * 4, 90);
    layer.cells = cellsB.data();
    layer.last_update_sec = 2.0;
    s.sim_time_sec = 2.0;
    overlume::set_scene(r, s);
    render_once(r, pose);
    EXPECT_EQ(overlume::testing::ground_grid_texture_upload_count(r, 0), 2u)
        << "a new ingest (advanced last_update_sec) did not trigger an upload";
    overlume::destroy_renderer(r);
}

TEST(GroundGrid, StaleGridFadesViaSharedStalenessAlpha) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<uint8_t> cells(4 * 4, 50);
    overlume::GroundGridLayer layer{};
    layer.kind = 0;
    layer.origin = {0.0, 0.0, 0.0};
    layer.resolution_m = 1.0;
    layer.width_cells = 4;
    layer.height_cells = 4;
    layer.cells = cells.data();
    layer.last_update_sec = 9.25;
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.grids = &layer;
    s.grid_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    const float alpha = overlume::testing::ground_grid_material_alpha(r, 0);
    EXPECT_GT(alpha, 0.0f);
    EXPECT_LT(alpha, 1.0f);
    overlume::destroy_renderer(r);
}

TEST(GroundGrid, MaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<uint8_t> cellsA(4 * 4, 20);
    std::vector<uint8_t> cellsB(4 * 4, 60);
    overlume::GroundGridLayer layers[2]{};
    layers[0].kind = 0;
    layers[0].origin = {0.0, 0.0, 0.0};
    layers[0].resolution_m = 1.0;
    layers[0].width_cells = 4;
    layers[0].height_cells = 4;
    layers[0].cells = cellsA.data();
    layers[1].kind = 1;
    layers[1].origin = {5.0, 0.0, 0.0};
    layers[1].resolution_m = 1.0;
    layers[1].width_cells = 4;
    layers[1].height_cells = 4;
    layers[1].cells = cellsB.data();
    overlume::SceneGraph s{};
    s.grids = layers;
    s.grid_count = 2;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    const auto freeColor = overlume::testing::ground_grid_free_color(r);
    EXPECT_NEAR(freeColor.r, theme->palette.ground.r, 1e-4);
    EXPECT_NEAR(freeColor.g, theme->palette.ground.g, 1e-4);
    EXPECT_NEAR(freeColor.b, theme->palette.ground.b, 1e-4);

    const auto occupiedColor = overlume::testing::ground_grid_occupied_color(r);
    EXPECT_NEAR(occupiedColor.r, theme->palette.alert.warning.r, 1e-4);
    EXPECT_NEAR(occupiedColor.g, theme->palette.alert.warning.g, 1e-4);
    EXPECT_NEAR(occupiedColor.b, theme->palette.alert.warning.b, 1e-4);

    EXPECT_FLOAT_EQ(overlume::testing::ground_grid_material_alpha(r, 0), 1.0f);
    EXPECT_FLOAT_EQ(overlume::testing::ground_grid_material_alpha(r, 1), 1.0f);
    overlume::destroy_renderer(r);
}
