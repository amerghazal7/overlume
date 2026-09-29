// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "ground_grid_test_hooks.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

namespace {

overlume::GroundGridLayer MakeLayer(uint8_t kind, const std::vector<uint8_t>& cells,
                                    overlume::Vec3 origin, double yaw_rad = 0.0) {
    overlume::GroundGridLayer g{};
    g.kind = kind;
    g.origin = origin;
    g.resolution_m = 0.2;
    g.width_cells = 250;
    g.height_cells = 250;
    g.cells = cells.data();
    g.last_update_sec = 10.0;
    g.yaw_rad = yaw_rad;
    return g;
}

std::vector<uint8_t> RenderGrids(const std::vector<overlume::GroundGridLayer>& gs,
                                 const overlume::Vec3& ego) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    std::vector<uint8_t> px(320u * 240u * 3u);
    auto* r = overlume::create_renderer(cfg);
    if (!r) return {};
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {ego, 0.0, 0.0, 1};
    s.grids = gs.data();
    s.grid_count = static_cast<uint32_t>(gs.size());
    overlume::set_scene(r, s);
    const overlume::CameraPose pose{{ego.x - 10, ego.y - 10, 20}, {ego.x, ego.y, 0}, 60.0};
    overlume::render_frame(r, pose, {px.data(), 320, 240});
    overlume::render_frame(r, pose, {px.data(), 320, 240});
    overlume::destroy_renderer(r);
    return px;
}

size_t ChangedPixels(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    size_t n = 0;
    for (size_t i = 0; i < a.size(); i += 3) {
        int d = 0;
        for (int c = 0; c < 3; ++c) d = std::max(d, std::abs(int(a[i + c]) - int(b[i + c])));
        if (d > 20) ++n;
    }
    return n;
}

}

TEST(GroundGrid, AllFreeUpperLayerDoesNotHideTheLayerBelow) {
    const overlume::Vec3 ego{343.3, -185.9, 0.0};
    std::vector<uint8_t> occupied(250u * 250u, 0), allFree(250u * 250u, 0);
    for (int y = 125; y < 175; ++y)
        for (int x = 125; x < 175; ++x) occupied[y * 250 + x] = 100;
    const overlume::Vec3 origin{ego.x - 25.0, ego.y - 25.0, 0.0};
    const auto none = RenderGrids({}, ego);
    if (none.empty()) GTEST_SKIP() << "no GPU/EGL";
    const size_t alone = ChangedPixels(none, RenderGrids({MakeLayer(1, occupied, origin)}, ego));
    const size_t stacked = ChangedPixels(
        none, RenderGrids({MakeLayer(0, allFree, origin), MakeLayer(1, occupied, origin)}, ego));
    ASSERT_GT(alone, 1000u);
    EXPECT_GT(stacked, alone * 9 / 10)
        << "free cells of the upper (dynamic) layer painted over the lower layer's obstacles: "
        << stacked << " px visible vs " << alone << " alone";
    EXPECT_LT(ChangedPixels(none, RenderGrids({MakeLayer(0, allFree, origin)}, ego)), 50u)
        << "an all-free grid must leave the frame unchanged";
}

TEST(GroundGrid, YawRotatesTheQuadAboutTheGridOrigin) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> cells(250u * 250u, 0);
    const overlume::GroundGridLayer g = MakeLayer(0, cells, {100.0, 50.0, 0.0}, M_PI / 2.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.grids = &g;
    s.grid_count = 1;
    overlume::set_scene(r, s);
    std::vector<uint8_t> px(320u * 240u * 3u);
    overlume::render_frame(r, {{0, -10, 10}, {0, 0, 0}, 60.0}, {px.data(), 320, 240});
    overlume::Vec3 c1{}, c3{};
    ASSERT_TRUE(overlume::testing::ground_grid_corner(r, 0, 1, &c1));
    ASSERT_TRUE(overlume::testing::ground_grid_corner(r, 0, 3, &c3));
    EXPECT_NEAR(c1.x, 100.0, 1e-3) << "the grid's +x edge (50 m) must point along map +y";
    EXPECT_NEAR(c1.y, 100.0, 1e-3);
    EXPECT_NEAR(c3.x, 50.0, 1e-3) << "the grid's +y edge must point along map -x";
    EXPECT_NEAR(c3.y, 50.0, 1e-3);
    overlume::destroy_renderer(r);
}
