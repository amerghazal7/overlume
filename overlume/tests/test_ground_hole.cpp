// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "height_grid_test_hooks.hpp"
#include "test_paths.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

namespace {

constexpr uint32_t kW = 320;
constexpr uint32_t kH = 240;

overlume::VisualRenderer* make_renderer() {
    overlume::RenderConfig cfg{kW, kH, 1, kThemeDir, "light_clay"};
    return overlume::create_renderer(cfg);
}

// Renders one frame; the ground hole is recomputed inside render_frame.
std::vector<uint8_t> render_scene(overlume::VisualRenderer* r, const overlume::SceneGraph& s,
                                  const overlume::CameraPose& pose) {
    overlume::set_scene(r, s);
    std::vector<uint8_t> pixels(static_cast<size_t>(kW) * kH * 3u);
    overlume::FrameView view{pixels.data(), kW, kH};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    return pixels;
}

overlume::SceneGraph base_scene(double now) {
    overlume::SceneGraph s{};
    s.sim_time_sec = now;
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    return s;
}

// A yawed 40x20 layer at 0.5 m: centre = origin + R(30 deg) * (10, 5) = (16.160254, 5.330127),
// half extents = (39/2 * 0.5, 19/2 * 0.5) = (9.75, 4.75), axis_x = (cos 30, sin 30).
struct YawedLayer {
    std::vector<float> heights = std::vector<float>(40u * 20u, 0.0f);
    overlume::HeightGridLayer layer{};
    explicit YawedLayer(double last_update_sec) {
        layer.origin = {10.0, -4.0, 0.0};
        layer.yaw_rad = 0.5235987755982988;  // 30 degrees
        layer.resolution_m = 0.5;
        layer.width_cells = 40;
        layer.height_cells = 20;
        layer.heights_m = heights.data();
        layer.last_update_sec = last_update_sec;
    }
};

// Mean absolute RGB difference over the 9x9 patch at the image centre.
double centre_patch_diff(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    double diff = 0.0;
    for (uint32_t y = kH / 2 - 4; y <= kH / 2 + 4; ++y) {
        for (uint32_t x = kW / 2 - 4; x <= kW / 2 + 4; ++x) {
            for (uint32_t c = 0; c < 3; ++c) {
                const size_t i = (static_cast<size_t>(y) * kW + x) * 3u + c;
                diff += std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
            }
        }
    }
    return diff / (81.0 * 3.0);
}

}

TEST(GroundHole, EnabledWithExpectedFootprintForYawedLayer) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    YawedLayer l(10.0);
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &l.layer;
    s.height_grid_count = 1;
    render_scene(r, s, {{-14, -14, 10}, {0, 0, 0}, 60.0});

    float center[2] = {}, axis[2] = {}, half[2] = {};
    ASSERT_TRUE(overlume::testing::ground_hole_state(r, center, axis, half));
    EXPECT_NEAR(center[0], 16.160254f, 1e-4f);
    EXPECT_NEAR(center[1], 5.330127f, 1e-4f);
    EXPECT_NEAR(axis[0], 0.8660254f, 1e-5f);
    EXPECT_NEAR(axis[1], 0.5f, 1e-5f);
    EXPECT_NEAR(half[0], 9.75f, 1e-5f);
    EXPECT_NEAR(half[1], 4.75f, 1e-5f);
    overlume::destroy_renderer(r);
}

TEST(GroundHole, DisabledWithoutLayerAndAfterLayerRemoved) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{-14, -14, 10}, {0, 0, 0}, 60.0};
    render_scene(r, base_scene(10.0), pose);
    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));

    YawedLayer l(10.0);
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &l.layer;
    s.height_grid_count = 1;
    render_scene(r, s, pose);
    EXPECT_TRUE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));

    render_scene(r, base_scene(10.1), pose);
    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    overlume::destroy_renderer(r);
}

TEST(GroundHole, DisabledOnceFullyStale) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{-14, -14, 10}, {0, 0, 0}, 60.0};
    YawedLayer l(10.0);
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &l.layer;
    s.height_grid_count = 1;

    render_scene(r, s, pose);  // age 0: alpha 1
    EXPECT_TRUE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    s.sim_time_sec = 10.7;  // age 0.7: alpha 0.6, terrain fading, hole still open
    render_scene(r, s, pose);
    EXPECT_TRUE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    s.sim_time_sec = 11.0;  // age 1.0: alpha 0, terrain removed, ground restored
    render_scene(r, s, pose);
    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    overlume::destroy_renderer(r);
}

// Valid dims and resolution but heights_m == nullptr (what OwnedScene::assign produces for an
// empty copy, and what any caller with a bad buffer sends). Task 3 draws no terrain, so the
// ground must stay intact: no hole.
TEST(GroundHole, DisabledWhenLayerHasNoHeights) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::HeightGridLayer layer{};
    layer.origin = {0.0, 0.0, 0.0};
    layer.yaw_rad = 0.0;
    layer.resolution_m = 1.0;
    layer.width_cells = 10;
    layer.height_cells = 10;
    layer.heights_m = nullptr;
    layer.last_update_sec = 10.0;
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &layer;
    s.height_grid_count = 1;
    render_scene(r, s, {{-14, -14, 10}, {0, 0, 0}, 60.0});

    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 0), 0u);
    overlume::destroy_renderer(r);
}

// The shader discard itself: a uniform -1 m ditch is invisible under the opaque z=0 ground
// quad unless the quad has a hole. Looking straight down at the footprint centre, the pixels
// must differ from the same frame without a layer. Layer 21x21 at 1 m with origin (-10,-10)
// covers x,y in [-9.5, 10.5]; the camera ray hits (0, 0, 0), inside it. Reverting the discard
// leaves the clay ground visible and this test fails. (No ego model is loaded in tests, so
// nothing occludes the centre patch in either frame.)
TEST(GroundHole, DiscardRevealsTerrainBelowGround) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{0.0, -0.5, 10.0}, {0, 0, 0}, 60.0};
    const std::vector<uint8_t> without = render_scene(r, base_scene(10.0), pose);

    std::vector<float> heights(21u * 21u, -1.0f);
    overlume::HeightGridLayer layer{};
    layer.origin = {-10.0, -10.0, 0.0};
    layer.yaw_rad = 0.0;
    layer.resolution_m = 1.0;
    layer.width_cells = 21;
    layer.height_cells = 21;
    layer.heights_m = heights.data();
    layer.last_update_sec = 10.0;
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &layer;
    s.height_grid_count = 1;
    const std::vector<uint8_t> with = render_scene(r, s, pose);

    // Measured (light_clay, quality 1): ~9.5 with the hole, ~0.5 with the discard reverted.
    EXPECT_GT(centre_patch_diff(with, without), 5.0)
        << "centre pixels did not change: the ground is not discarded";
    overlume::destroy_renderer(r);
}

// Same check with layer and camera ~200 m from the world origin, as on the live rig where map
// coordinates are far from 0. The camera-relative getWorldPosition() differs from the true
// position by the eye offset (~200 m), so a hole built on it lands far from the footprint and
// this test fails; getUserWorldPosition() passes. Layer 21x21 at 1 m with origin (190,190)
// covers x,y in [190.5, 210.5] (cell-centre vertices) around the centre (200.5, 200.5); the
// camera looks straight down at (200, 200). The ego sits at (200, 200) in both frames so the
// 60 m ground quad snaps under the layer and the "without" frame shows clay, not background;
// otherwise the diff would be vacuous.
TEST(GroundHole, DiscardRevealsTerrainFarFromWorldOrigin) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{200.0, 199.5, 10.0}, {200, 200, 0}, 60.0};
    overlume::SceneGraph s0 = base_scene(10.0);
    s0.ego = {{200, 200, 0}, 0.0, 0.0, 1};
    const std::vector<uint8_t> without = render_scene(r, s0, pose);

    std::vector<float> heights(21u * 21u, -1.0f);
    overlume::HeightGridLayer layer{};
    layer.origin = {190.0, 190.0, 0.0};
    layer.yaw_rad = 0.0;
    layer.resolution_m = 1.0;
    layer.width_cells = 21;
    layer.height_cells = 21;
    layer.heights_m = heights.data();
    layer.last_update_sec = 10.0;
    overlume::SceneGraph s = base_scene(10.0);
    s.ego = {{200, 200, 0}, 0.0, 0.0, 1};
    s.height_grids = &layer;
    s.height_grid_count = 1;
    const std::vector<uint8_t> with = render_scene(r, s, pose);

    // Measured (light_clay, quality 1): ~9.5 with the hole, ~0.5 with the discard reverted or
    // with getWorldPosition() in place of getUserWorldPosition().
    EXPECT_GT(centre_patch_diff(with, without), 5.0)
        << "hole is not under the layer footprint far from the world origin";
    overlume::destroy_renderer(r);
}
