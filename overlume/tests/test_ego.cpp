// test_ego.cpp — ego robot via Filament gltfio, clay-box fallback on load
// failure.
//
// Deliberately does NOT include renderer_internal.hpp: that header defines
// `Mesh` and pulls in <filament/...>, and this test binary is only linked
// against `overlume` (CMakeLists.txt), never granted
// overlume's own PRIVATE Filament include dir -- so it wouldn't
// compile here. ego_test_hooks.hpp declares the one test-only introspection
// hook this file needs against nothing but api.h's already-forward-declared
// opaque overlume::VisualRenderer instead.
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
    scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/1};

    // Baseline: set_ego_model() never called, so r->egoTransformEntity
    // stays null and update_ego_transform() is a no-op (ego.cpp) --
    // ground+grid only.
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

    // The parse-time bounding box just checked comes from the glTF's JSON
    // accessor min/max, computed by gltfio at createAsset() time -- it
    // stays non-empty even if ResourceLoader::loadResources() or
    // Scene::addEntities() were silently skipped afterward, so it alone
    // cannot tell "parsed" apart from "actually rendered." Comparing real
    // rendered pixels against the no-ego baseline above closes that gap.
    size_t differing_bytes = 0;
    for (size_t i = 0; i < pixels.size(); ++i) {
        if (pixels[i] != baseline_pixels[i]) ++differing_bytes;
    }
    EXPECT_GT(differing_bytes, 0u)
        << "loaded glTF produced no visible difference from the no-ego "
           "baseline -- loadResources()/addEntities() may have been "
           "skipped even though the asset parsed (bounding box was "
           "non-empty above)";

    overlume::destroy_renderer(r);
}

// Pins scene.h's frozen "ego.valid==0 -> hidden, not a clay box at origin"
// contract: this is the node's default state at startup (no TF yet),
// before any real ego pose has arrived.
TEST(Ego, InvalidEgo_RendersIdenticalToNoEgoBaseline) {
    overlume::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};

    // Baseline: set_ego_model() never called, ground+grid only -- same
    // baseline construction as LoadValidGltf_RendersNonEmptyBoundingBox
    // above.
    overlume::SceneGraph baseline_scene{};
    baseline_scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/1};
    auto* base_r = overlume::create_renderer(cfg);
    if (!base_r) GTEST_SKIP();
    overlume::set_scene(base_r, baseline_scene);
    std::vector<uint8_t> baseline_pixels(320u * 240u * 3u);
    overlume::FrameView baseline_view{baseline_pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(base_r, pose, baseline_view));
    overlume::destroy_renderer(base_r);

    // set_ego_model() succeeds (real glTF loaded), but scene.ego.valid=0 --
    // update_ego_transform() must still zero-scale it to invisible, so the
    // rendered frame should be pixel-identical to the no-ego baseline.
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_TRUE(overlume::set_ego_model(r, OVERLUME_TEST_DATA_DIR "/tests/fixtures/test_cube.glb",
                                      {4.5, 2.0, 1.8}));
    overlume::SceneGraph scene{};
    scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/0};
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
    scene.ego = {{0, 0, 0}, 0, 0, /*valid=*/1};
    overlume::set_scene(r, scene);
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));  // must not crash/fail -- clay box instead
    // True diagonal of the {4.5, 2.0, 1.8} box build_ego_box() actually
    // built -- sqrt(4.5^2+2.0^2+1.8^2) ~= 5.24. Must be an exact check, not
    // EXPECT_GT(..., 0.0): a loose check passes vacuously off
    // RenderableManager::getAxisAlignedBoundingBox()'s declared culling AABB
    // (add_mesh()'s hard-coded kGroundHalfExtent box, ~56.6 diagonal),
    // never actually reading what build_ego_box() produced.
    EXPECT_NEAR(overlume::testing::rendered_bounding_box_diagonal(r), 5.2431, 1e-3);
    overlume::destroy_renderer(r);
}

// Wrong-dims guard: a never-built fallback (no set_ego_model() call at
// all) must report a 0 diagonal, not whatever stale AABB state Filament
// happens to hold.
TEST(Ego, NoEgoModelSet_BoundingBoxDiagonalIsZero) {
    overlume::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_EQ(overlume::testing::rendered_bounding_box_diagonal(r), 0.0);
    overlume::destroy_renderer(r);
}

// Golden: ego at a fixed pose using the CLAY-BOX FALLBACK path — no glTF
// asset — since the CI-committed golden must not depend on the user's
// non-git M02P asset (test_cube.glb above is git-committed, but exercised
// by the correctness test above, not baked into a golden here). First
// golden with an actual box in frame, so first to visibly exercise SSAO
// contact darkening.
TEST(EgoGolden, ClayBoxFallback_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    EXPECT_FALSE(overlume::set_ego_model(r, "/nonexistent/path.glb", {4.5, 2.0, 1.8}));

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    scene.ego = {{0.0, 0.0, 0.0}, /*heading_rad=*/0.0, /*speed_mps=*/0.0, /*valid=*/1};
    overlume::set_scene(r, scene);

    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/ego_clay_box_dark_adas.png",
        "/tmp/ego_clay_box_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);

    overlume::destroy_renderer(r);
}

// Regression guard, mirrors
// MapElements.LaneMaterialIsThemedOnFirstDataWithNoTransition (test_map_
// elements.cpp): egoMaterial is created EAGERLY in create_renderer() and
// registered by push_theme_to_scene() at that same call, BEFORE
// set_ego_model() is ever invoked -- so it must already read as
// theme.palette.ego on the very first render, with no set_theme() call
// anywhere in this test. Catches binding r.egoMaterial's baseColor from
// theme.palette.ground instead of theme.palette.ego (or never wiring it up
// at all, in which case this reads clay.mat's compiled-in zero default).
TEST(Ego, EgoMaterialIsThemedOnFirstRenderWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
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
