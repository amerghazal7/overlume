// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "test_paths.hpp"

#include <cmath>
#include <string>

#include <gtest/gtest.h>

namespace {

// Renders the synthetic terrain scene under `theme` and compares to `golden_name`. The actual frame
// lands in OVERLUME_TMP_DIR/<golden_name>_actual.png (the candidate a human promotes). Returns -1
// without a renderer (no GPU/EGL), 0.0 when the golden is missing or the frame fails.
double render_terrain(const char* theme, const std::string& golden_name, double yaw_rad) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, theme};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return -1.0;

    overlume::testing::HeightGridScene t =
        overlume::testing::make_height_grid_terrain_scene(10.0, yaw_rad);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    s.height_grids = t.height_grids.data();
    s.height_grid_count = static_cast<uint32_t>(t.height_grids.size());
    s.grids = t.grids.data();
    s.grid_count = static_cast<uint32_t>(t.grids.size());
    overlume::set_scene(r, s);

    const std::string golden =
        std::string(OVERLUME_TEST_DATA_DIR) + "/tests/goldens/" + golden_name + ".png";
    const std::string actual = std::string(OVERLUME_TMP_DIR "/") + golden_name + "_actual.png";
    const overlume::CameraPose pose{{-4, -16, 10}, {16, 0, 0}, 60.0};
    const double ssim =
        overlume::testing::render_and_compare(r, pose, golden.c_str(), actual.c_str());
    overlume::destroy_renderer(r);
    return ssim;
}

}

// CPU-only: guards the builder itself, so a golden can't silently pin a flat or empty scene.
TEST(HeightGridTerrainScene, BuilderPlacesTheFourFeatures) {
    const auto t = overlume::testing::make_height_grid_terrain_scene(10.0);
    ASSERT_EQ(t.height_grids.size(), 1u);
    ASSERT_EQ(t.grids.size(), 1u);
    const auto& g = t.height_grids[0];
    ASSERT_EQ(g.width_cells, 120u);
    ASSERT_EQ(g.height_cells, 120u);
    auto h = [&](uint32_t i, uint32_t j) { return g.heights_m[j * g.width_cells + i]; };
    EXPECT_NEAR(h(30, 30), 1.2f, 1e-4) << "berm crest";
    EXPECT_NEAR(h(30, 90), -0.8f, 1e-4) << "ditch floor";
    EXPECT_NEAR(h(119, 10), 1.5f, 1e-4) << "ramp top";
    EXPECT_NEAR(h(64, 10), 0.0f, 1e-4) << "ramp foot";
    EXPECT_TRUE(std::isnan(h(30, 68))) << "unknown patch";
    EXPECT_FALSE(std::isnan(h(30, 40)));
    EXPECT_EQ(t.grids[0].cells[48 * 120 + 12], 100);
    EXPECT_EQ(t.grids[0].width_cells, g.width_cells);
    EXPECT_DOUBLE_EQ(g.yaw_rad, 0.0);
    EXPECT_DOUBLE_EQ(t.grids[0].yaw_rad, 0.0);
}

// CPU-only: the yawed variant rotates both grids by the same angle about the same corner and keeps
// the heights identical, so the yawed golden differs from the plain one only by placement.
TEST(HeightGridTerrainScene, YawRotatesBothGridsAndKeepsHeights) {
    const auto plain = overlume::testing::make_height_grid_terrain_scene(10.0);
    const auto yawed = overlume::testing::make_height_grid_terrain_scene(10.0, 0.35);
    EXPECT_DOUBLE_EQ(yawed.height_grids[0].yaw_rad, 0.35);
    EXPECT_DOUBLE_EQ(yawed.grids[0].yaw_rad, 0.35);
    EXPECT_DOUBLE_EQ(yawed.height_grids[0].origin.x, plain.height_grids[0].origin.x);
    EXPECT_DOUBLE_EQ(yawed.grids[0].origin.y, plain.grids[0].origin.y);
    ASSERT_EQ(yawed.heights.size(), plain.heights.size());
    EXPECT_NEAR(yawed.heights[30 * 120 + 30], 1.2f, 1e-4);
}

TEST(HeightGridGolden, TerrainDarkAdas) {
    const double ssim = render_terrain("dark_adas", "height_grid_terrain_dark_adas", 0.0);
    if (ssim < 0.0) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
}

TEST(HeightGridGolden, TerrainLightClay) {
    const double ssim = render_terrain("light_clay", "height_grid_terrain_light_clay", 0.0);
    if (ssim < 0.0) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
}

// Pixel coverage for yaw != 0: the grid and the ground-plane hole under it are both rotated, so a
// hole that ignored yaw would show ground through the terrain or a missing ground strip.
TEST(HeightGridGolden, TerrainYawedDarkAdas) {
    const double ssim = render_terrain("dark_adas", "height_grid_terrain_yawed_dark_adas", 0.35);
    if (ssim < 0.0) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
}
