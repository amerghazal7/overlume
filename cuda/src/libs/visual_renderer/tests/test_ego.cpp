// test_ego.cpp — Epic 1 Task 4 (VM-012): ego robot via Filament gltfio,
// clay-box fallback on load failure
// (docs/superpowers/plans/2026-08-18-visual-mode-epic1.md).
//
// Deliberately does NOT include renderer_internal.hpp: per Task 2 Step 7e
// that header defines `Mesh` and pulls in <filament/...>, and this test
// binary is only linked against `visual_renderer` (target_link_libraries(
// ${_test_name} PRIVATE visual_renderer gtest gtest_main EGL) in
// CMakeLists.txt), never granted visual_renderer's own PRIVATE Filament
// include dir — so it wouldn't compile here. ego_test_hooks.hpp declares
// the one test-only introspection hook this file needs against nothing but
// api.h's already-forward-declared opaque mpviz::VisualRenderer instead.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "ego_test_hooks.hpp"
#include "golden.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(Ego, LoadValidGltf_RendersNonEmptyBoundingBox) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    mpviz::SceneGraph scene{};
    scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/1};

    // Baseline: set_ego_model() never called at all, so
    // r->egoTransformEntity stays null and update_ego_transform() is a
    // no-op (ego.cpp's own comment) -- ground+grid only. This is exactly
    // the pixel result the "silently renders nothing" trap below would
    // produce if loadResources()/addEntities() were skipped despite the
    // asset having parsed successfully.
    auto* base_r = mpviz::create_renderer(cfg);
    if (!base_r) GTEST_SKIP();
    mpviz::set_scene(base_r, scene);
    std::vector<uint8_t> baseline_pixels(320u * 240u * 3u);
    mpviz::FrameView baseline_view{baseline_pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(base_r, pose, baseline_view));
    mpviz::destroy_renderer(base_r);

    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_TRUE(mpviz::set_ego_model(r, MPVIZ_TEST_DATA_DIR "/tests/fixtures/test_cube.glb",
                                      {4.5, 2.0, 1.8}));
    mpviz::set_scene(r, scene);
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    EXPECT_GT(mpviz::testing::rendered_bounding_box_diagonal(r), 0.0);

    // Discriminating check (review finding): the parse-time bounding box
    // just checked comes from the glTF's JSON accessor min/max, computed by
    // gltfio at createAsset() time -- it stays non-empty even if
    // ResourceLoader::loadResources() or Scene::addEntities() were
    // silently skipped afterward, so it alone cannot tell "parsed" apart
    // from "actually rendered." Comparing real rendered pixels against the
    // no-ego baseline above closes that gap: if the loaded mesh never
    // reached the framebuffer, this frame would be pixel-identical to the
    // baseline despite the bounding-box check above passing.
    size_t differing_bytes = 0;
    for (size_t i = 0; i < pixels.size(); ++i) {
        if (pixels[i] != baseline_pixels[i]) ++differing_bytes;
    }
    EXPECT_GT(differing_bytes, 0u)
        << "loaded glTF produced no visible difference from the no-ego "
           "baseline -- loadResources()/addEntities() may have been "
           "skipped even though the asset parsed (bounding box was "
           "non-empty above)";

    mpviz::destroy_renderer(r);
}

// Pins scene.h's frozen "ego.valid==0 -> hidden, not a clay box at origin"
// contract (review round 8): this is the node's default state at startup
// (no TF yet), before any real ego pose has arrived.
TEST(Ego, InvalidEgo_RendersIdenticalToNoEgoBaseline) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};

    // Baseline: set_ego_model() never called, ground+grid only -- same
    // baseline construction as LoadValidGltf_RendersNonEmptyBoundingBox
    // above.
    mpviz::SceneGraph baseline_scene{};
    baseline_scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/1};
    auto* base_r = mpviz::create_renderer(cfg);
    if (!base_r) GTEST_SKIP();
    mpviz::set_scene(base_r, baseline_scene);
    std::vector<uint8_t> baseline_pixels(320u * 240u * 3u);
    mpviz::FrameView baseline_view{baseline_pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(base_r, pose, baseline_view));
    mpviz::destroy_renderer(base_r);

    // set_ego_model() succeeds (real glTF loaded), but scene.ego.valid=0 --
    // update_ego_transform() must still zero-scale it to invisible, so the
    // rendered frame should be pixel-identical to the no-ego baseline.
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_TRUE(mpviz::set_ego_model(r, MPVIZ_TEST_DATA_DIR "/tests/fixtures/test_cube.glb",
                                      {4.5, 2.0, 1.8}));
    mpviz::SceneGraph scene{};
    scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/0};
    mpviz::set_scene(r, scene);
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    mpviz::destroy_renderer(r);

    EXPECT_EQ(pixels, baseline_pixels)
        << "ego.valid=0 must render identically to no ego loaded at all -- "
           "hidden, not a visible clay box/glTF at the origin";
}

TEST(Ego, LoadMissingFile_FallsBackToClayBoxNonFatally) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_FALSE(mpviz::set_ego_model(r, "/nonexistent/path.glb", {4.5, 2.0, 1.8}));
    mpviz::SceneGraph scene{};
    scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/1};
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));  // must not crash/fail -- clay box instead
    // True diagonal of the {4.5, 2.0, 1.8} box build_ego_box() actually
    // built -- sqrt(4.5^2+2.0^2+1.8^2) ~= 5.24. Was EXPECT_GT(..., 0.0)
    // before review round 8: that passed vacuously off
    // RenderableManager::getAxisAlignedBoundingBox()'s declared culling AABB
    // (add_mesh()'s hard-coded kGroundHalfExtent box, ~56.6 diagonal),
    // never actually reading what build_ego_box() produced.
    EXPECT_NEAR(mpviz::testing::rendered_bounding_box_diagonal(r), 5.2431, 1e-3);
    mpviz::destroy_renderer(r);
}

// Wrong-dims guard (review round 8): a never-built fallback (no
// set_ego_model() call at all) must report a 0 diagonal, not whatever stale
// AABB state Filament happens to hold.
TEST(Ego, NoEgoModelSet_BoundingBoxDiagonalIsZero) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_EQ(mpviz::testing::rendered_bounding_box_diagonal(r), 0.0);
    mpviz::destroy_renderer(r);
}

// Golden (Step 6): ego at a fixed pose using the CLAY-BOX FALLBACK path —
// no glTF asset — since the CI-committed golden must not depend on the
// user's non-git M02P asset (test_cube.glb above is a real, git-committed
// fixture, but it's exercised by the correctness test above, not baked
// into a golden here, per the plan's own Step 6 note: "the committed
// golden must not depend on the user's non-git M02P asset"). Same
// quality=1 (medium) RenderConfig as every other golden this epic — this
// is the first golden with an actual box in frame, so it's the first to
// visibly exercise Step 7a's SSAO contact darkening.
TEST(EgoGolden, ClayBoxFallback_DarkAdas) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    EXPECT_FALSE(mpviz::set_ego_model(r, "/nonexistent/path.glb", {4.5, 2.0, 1.8}));

    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    scene.ego = {{0.0, 0.0, 0.0}, /*heading_rad=*/0.0, /*speed_mps=*/0.0, /*valid=*/1};
    mpviz::set_scene(r, scene);

    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = mpviz::testing::render_and_compare(
        r, pose, MPVIZ_TEST_DATA_DIR "/tests/goldens/ego_clay_box_dark_adas.png",
        "/tmp/ego_clay_box_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);

    mpviz::destroy_renderer(r);
}

// Regression guard (user contrast directive 2026-08-20), mirrors
// MapElements.LaneMaterialIsThemedOnFirstDataWithNoTransition (test_map_
// elements.cpp): egoMaterial is created EAGERLY in create_renderer() and
// registered by push_theme_to_scene() at that same call, BEFORE
// set_ego_model() is ever invoked -- so it must already read as
// theme.palette.ego on the very first render, with no set_theme() call
// anywhere in this test. Catches the exact bug this task fixes: binding
// r.egoMaterial's baseColor from theme.palette.ground instead of
// theme.palette.ego (or never wiring it up at all, in which case this
// reads clay.mat's compiled-in zero default).
TEST(Ego, EgoMaterialIsThemedOnFirstRenderWithNoTransition) {
    const auto theme = mpviz::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }

    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));

    const auto p = mpviz::testing::ego_material_base_color(r);
    EXPECT_NEAR(p.r, theme->palette.ego.r, 1e-4);
    EXPECT_NEAR(p.g, theme->palette.ego.g, 1e-4);
    EXPECT_NEAR(p.b, theme->palette.ego.b, 1e-4);

    mpviz::destroy_renderer(r);
}
