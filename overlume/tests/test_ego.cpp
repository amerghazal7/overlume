// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "ego_test_hooks.hpp"
#include "golden.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(Ego, LoadValidGltf_RendersNonEmptyBoundingBox) {
    overlume::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    overlume::SceneGraph scene{};
    scene.ego = {{0, 0, 0}, 0, 0, 1};

    auto* base_r = overlume::create_renderer(cfg);
    if (!base_r) GTEST_SKIP();
    overlume::set_scene(base_r, scene);
    std::vector<uint8_t> baseline_pixels(320u * 240u * 3u);
    overlume::FrameView baseline_view{baseline_pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(base_r, pose, baseline_view));
    overlume::destroy_renderer(base_r);

    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_TRUE(overlume::set_ego_model(r, OVERLUME_TEST_DATA_DIR "/tests/fixtures/test_cube.glb",
                                        {4.5, 2.0, 1.8}));
    overlume::set_scene(r, scene);
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    EXPECT_GT(overlume::testing::rendered_bounding_box_diagonal(r), 0.0);

    size_t differing_bytes = 0;
    for (size_t i = 0; i < pixels.size(); ++i) {
        if (pixels[i] != baseline_pixels[i]) ++differing_bytes;
    }
    EXPECT_GT(differing_bytes, 0u) << "loaded glTF produced no visible difference from the no-ego "
                                      "baseline -- loadResources()/addEntities() may have been "
                                      "skipped even though the asset parsed (bounding box was "
                                      "non-empty above)";

    overlume::destroy_renderer(r);
}

TEST(Ego, InvalidEgo_RendersIdenticalToNoEgoBaseline) {
    overlume::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};

    overlume::SceneGraph baseline_scene{};
    baseline_scene.ego = {{0, 0, 0}, 0, 0, 1};
    auto* base_r = overlume::create_renderer(cfg);
    if (!base_r) GTEST_SKIP();
    overlume::set_scene(base_r, baseline_scene);
    std::vector<uint8_t> baseline_pixels(320u * 240u * 3u);
    overlume::FrameView baseline_view{baseline_pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(base_r, pose, baseline_view));
    overlume::destroy_renderer(base_r);

    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_TRUE(overlume::set_ego_model(r, OVERLUME_TEST_DATA_DIR "/tests/fixtures/test_cube.glb",
                                        {4.5, 2.0, 1.8}));
    overlume::SceneGraph scene{};
    scene.ego = {{0, 0, 0}, 0, 0, 0};
    overlume::set_scene(r, scene);
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    overlume::destroy_renderer(r);

    EXPECT_EQ(pixels, baseline_pixels)
        << "ego.valid=0 must render identically to no ego loaded at all -- "
           "hidden, not a visible clay box/glTF at the origin";
}

TEST(Ego, LoadMissingFile_FallsBackToClayBoxNonFatally) {
    overlume::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_FALSE(overlume::set_ego_model(r, "/nonexistent/path.glb", {4.5, 2.0, 1.8}));
    overlume::SceneGraph scene{};
    scene.ego = {{0, 0, 0}, 0, 0, 1};
    overlume::set_scene(r, scene);
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    EXPECT_NEAR(overlume::testing::rendered_bounding_box_diagonal(r), 5.2431, 1e-3);
    overlume::destroy_renderer(r);
}

TEST(Ego, NoEgoModelSet_BoundingBoxDiagonalIsZero) {
    overlume::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_EQ(overlume::testing::rendered_bounding_box_diagonal(r), 0.0);
    overlume::destroy_renderer(r);
}

TEST(EgoGolden, ClayBoxFallback_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    EXPECT_FALSE(overlume::set_ego_model(r, "/nonexistent/path.glb", {4.5, 2.0, 1.8}));

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    scene.ego = {{0.0, 0.0, 0.0}, 0.0, 0.0, 1};
    overlume::set_scene(r, scene);

    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/ego_clay_box_dark_adas.png",
        OVERLUME_TMP_DIR "/ego_clay_box_dark_adas_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);

    overlume::destroy_renderer(r);
}

TEST(Ego, EgoMaterialIsThemedOnFirstRenderWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    overlume::set_scene(r, scene);
    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));

    const auto p = overlume::testing::ego_material_base_color(r);
    EXPECT_NEAR(p.r, theme->palette.ego.r, 1e-4);
    EXPECT_NEAR(p.g, theme->palette.ego.g, 1e-4);
    EXPECT_NEAR(p.b, theme->palette.ego.b, 1e-4);

    overlume::destroy_renderer(r);
}
