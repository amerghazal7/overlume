// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "polyline.hpp"
#include "ribbon_test_hooks.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <algorithm>
#include <cstdlib>
#include <string>
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

TEST(RibbonFadeAlpha, DisabledUnlessEndIsBeyondStart) {
    EXPECT_FLOAT_EQ(overlume::testing::ribbon_fade_alpha(0.0, 0.0, 0.0f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(overlume::testing::ribbon_fade_alpha(50.0, 0.0, 10.0f, 10.0f), 1.0f);
    EXPECT_FLOAT_EQ(overlume::testing::ribbon_fade_alpha(50.0, 0.0, 10.0f, 5.0f), 1.0f);
}

TEST(RibbonFadeAlpha, RampsLinearlyInMetresFromTheOrigin) {
    EXPECT_FLOAT_EQ(overlume::testing::ribbon_fade_alpha(0.0, 0.0, 5.0f, 10.0f), 1.0f);
    EXPECT_FLOAT_EQ(overlume::testing::ribbon_fade_alpha(5.0, 0.0, 5.0f, 10.0f), 1.0f);
    EXPECT_NEAR(overlume::testing::ribbon_fade_alpha(7.5, 0.0, 5.0f, 10.0f), 0.5f, 1e-6f);
    EXPECT_NEAR(overlume::testing::ribbon_fade_alpha(10.0, 0.0, 5.0f, 10.0f), 0.0f, 1e-6f);
    EXPECT_NEAR(overlume::testing::ribbon_fade_alpha(4000.0, 0.0, 5.0f, 10.0f), 0.0f, 1e-6f)
        << "a kilometre-long ribbon fades out at fade_end_m regardless of its total length";
}

TEST(RibbonFadeAlpha, OriginShiftsTheRampToTheEgo) {
    EXPECT_FLOAT_EQ(overlume::testing::ribbon_fade_alpha(30.0, 60.0, 5.0f, 10.0f), 1.0f);
    EXPECT_FLOAT_EQ(overlume::testing::ribbon_fade_alpha(65.0, 60.0, 5.0f, 10.0f), 1.0f);
    EXPECT_NEAR(overlume::testing::ribbon_fade_alpha(67.5, 60.0, 5.0f, 10.0f), 0.5f, 1e-6f);
    EXPECT_NEAR(overlume::testing::ribbon_fade_alpha(70.0, 60.0, 5.0f, 10.0f), 0.0f, 1e-6f);
}

TEST(RibbonGolden, ThreeRoles_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::testing::RibbonScene ribbons = overlume::testing::make_three_role_ribbons(10.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = ribbons.ego;
    s.paths = ribbons.ribbons.data();
    s.path_count = static_cast<uint32_t>(ribbons.ribbons.size());
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{-4, -8, 6}, {4, 1, 0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/ribbons_three_roles_dark_adas.png",
        OVERLUME_TMP_DIR "/ribbons_three_roles_dark_adas_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
    overlume::destroy_renderer(r);
}

TEST(Ribbon, PathChangeRebuildsGeometry) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 longPts[] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::LOCAL;
    ribbon.points = longPts;
    ribbon.point_count = 5;
    ribbon.last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), 5u * 2);

    const overlume::Vec3 shortPts[] = {{0, 0, 0}, {1, 0, 0}};
    ribbon.points = shortPts;
    ribbon.point_count = 2;
    ribbon.last_update_sec = 2.0;
    s.sim_time_sec = 2.0;
    overlume::set_scene(r, s);
    render_once(r, pose);
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), 2u * 2)
        << "publishing a shorter path did not shrink slot 0's geometry -- a stale-cache bug";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, WidthChangeRebuildsGeometry) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_width_a"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-5, 0, 0}, {-3, 0, 0}, {-1, 0, 0}, {1, 0, 0}, {3, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::LOCAL;
    ribbon.points = pts;
    ribbon.point_count = 5;
    ribbon.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    EXPECT_NEAR(overlume::testing::ribbon_slot_half_width_m(r, 0), 0.12f, 1e-4f);
    const size_t vertsBefore = overlume::testing::ribbon_vertex_count(r, 0);

    ASSERT_TRUE(overlume::set_theme(r, "ribbon_width_b", 0.0, 0.2));
    s.sim_time_sec = 0.2;
    overlume::set_scene(r, s);
    render_once(r, pose);

    EXPECT_NEAR(overlume::testing::ribbon_slot_half_width_m(r, 0), 0.30f, 1e-4f)
        << "a width-only theme change (no PathRibbon point data touched) did not rebuild "
           "slot 0's geometry at the new width -- width isn't part of the slot signature";
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), vertsBefore);

    overlume::destroy_renderer(r);
}

TEST(Ribbon, TwoLocalRibbonsBothRender) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 ptsA[] = {{-5, 0, 0}, {-3, 0, 0}, {-1, 0, 0}};
    const overlume::Vec3 ptsB[] = {{1, 0, 0}, {3, 0, 0}, {5, 0, 0}, {7, 0, 0}};
    overlume::PathRibbon ribbons[2]{};
    ribbons[0].role = overlume::PathRole::LOCAL;
    ribbons[0].points = ptsA;
    ribbons[0].point_count = 3;
    ribbons[0].last_update_sec = 1.0;
    ribbons[1].role = overlume::PathRole::LOCAL;
    ribbons[1].points = ptsB;
    ribbons[1].point_count = 4;
    ribbons[1].last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.paths = ribbons;
    s.path_count = 2;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    EXPECT_GT(overlume::testing::ribbon_mesh_count(r, 0), 0u);
    EXPECT_GT(overlume::testing::ribbon_mesh_count(r, 1), 0u);
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), 3u * 2)
        << "slot 0 (the first LOCAL row) has no geometry of its own -- keyed by role, not slot";
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 1), 4u * 2)
        << "slot 1 (the second LOCAL row) has no geometry of its own -- overwritten by slot 0";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, StaleRibbonFadesViaSharedStalenessAlpha) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 behPts[] = {{0, 0, 0}, {2, 0, 0}};
    const overlume::Vec3 locPts[] = {{0, 5, 0}, {2, 5, 0}};
    overlume::PathRibbon ribbons[2]{};
    ribbons[0].role = overlume::PathRole::BEHAVIOR;
    ribbons[0].points = behPts;
    ribbons[0].point_count = 2;
    ribbons[0].last_update_sec = 9.25;
    ribbons[1].role = overlume::PathRole::LOCAL;
    ribbons[1].points = locPts;
    ribbons[1].point_count = 2;
    ribbons[1].last_update_sec = 9.25;
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.paths = ribbons;
    s.path_count = 2;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    const auto beh = overlume::testing::ribbon_slot_material_info(r, 0);
    EXPECT_TRUE(beh.bound_to_translucent);
    EXPECT_GT(beh.alpha, 0.0f);
    EXPECT_LT(beh.alpha, 1.0f);

    const auto loc = overlume::testing::ribbon_slot_material_info(r, 1);
    EXPECT_TRUE(loc.bound_to_translucent);
    EXPECT_GT(loc.alpha, 0.0f);
    EXPECT_LT(loc.alpha, 1.0f);
    overlume::destroy_renderer(r);
}

TEST(Ribbon, LongPathSplitsAcrossMeshesWithoutTruncation) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    constexpr uint32_t kN = 40000;
    std::vector<overlume::Vec3> pts(kN);
    for (uint32_t i = 0; i < kN; ++i) pts[i] = {static_cast<double>(i) * 0.1, 0.0, 0.0};

    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::GLOBAL;
    ribbon.points = pts.data();
    ribbon.point_count = kN;
    ribbon.last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 1.0;
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    const auto chunks = overlume::detail::polyline_chunks(kN);
    ASSERT_EQ(chunks.size(), 2u) << "40000 points should split into exactly 2 chunks at "
                                    "kMaxPointsPerMesh=32000 -- fixture assumption changed?";
    size_t expectedVerts = 0;
    for (const auto& [a, b] : chunks) expectedVerts += 2 * static_cast<size_t>(b - a);

    EXPECT_EQ(overlume::testing::ribbon_mesh_count(r, 0), chunks.size());
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), expectedVerts)
        << "vertex count doesn't match polyline_chunks()'s own math -- a point was lost "
           "at the chunk-split seam";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, MaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {5, 0, 0}};
    overlume::PathRibbon ribbons[3]{};
    ribbons[0].role = overlume::PathRole::BEHAVIOR;
    ribbons[0].points = pts;
    ribbons[0].point_count = 2;
    ribbons[1].role = overlume::PathRole::GLOBAL;
    ribbons[1].points = pts;
    ribbons[1].point_count = 2;
    ribbons[2].role = overlume::PathRole::LOCAL;
    ribbons[2].points = pts;
    ribbons[2].point_count = 2;
    overlume::SceneGraph s{};
    s.paths = ribbons;
    s.path_count = 3;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    const auto beh = overlume::testing::ribbon_role_base_color(r, overlume::PathRole::BEHAVIOR);
    EXPECT_NEAR(beh.r, theme->palette.ribbon_core.r, 1e-4);
    EXPECT_NEAR(beh.g, theme->palette.ribbon_core.g, 1e-4);
    EXPECT_NEAR(beh.b, theme->palette.ribbon_core.b, 1e-4);

    const auto glob = overlume::testing::ribbon_role_base_color(r, overlume::PathRole::GLOBAL);
    EXPECT_NEAR(glob.r, theme->palette.ribbon_global.r, 1e-4);
    EXPECT_NEAR(glob.g, theme->palette.ribbon_global.g, 1e-4);
    EXPECT_NEAR(glob.b, theme->palette.ribbon_global.b, 1e-4);

    const auto loc = overlume::testing::ribbon_role_base_color(r, overlume::PathRole::LOCAL);
    EXPECT_NEAR(loc.r, theme->palette.ribbon_local.r, 1e-4);
    EXPECT_NEAR(loc.g, theme->palette.ribbon_local.g, 1e-4);
    EXPECT_NEAR(loc.b, theme->palette.ribbon_local.b, 1e-4);
    overlume::destroy_renderer(r);
}

TEST(Ribbon, MarginChangeRebuildsGeometry) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_margin_a"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-5, 0, 0}, {-3, 0, 0}, {-1, 0, 0}, {1, 0, 0}, {3, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::LOCAL;
    ribbon.points = pts;
    ribbon.point_count = 5;
    ribbon.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    EXPECT_NEAR(overlume::testing::ribbon_slot_half_width_m(r, 0), 1.25f, 1e-4f);
    const size_t vertsBefore = overlume::testing::ribbon_vertex_count(r, 0);

    ASSERT_TRUE(overlume::set_theme(r, "ribbon_margin_b", 0.0, 0.2));
    s.sim_time_sec = 0.2;
    overlume::set_scene(r, s);
    render_once(r, pose);

    EXPECT_NEAR(overlume::testing::ribbon_slot_half_width_m(r, 0), 0.75f, 1e-4f)
        << "a margin-only theme change (no PathRibbon point data touched) did not rebuild "
           "slot 0's geometry at the new margin -- margins aren't part of the slot signature";
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), vertsBefore);
    overlume::destroy_renderer(r);
}

TEST(Ribbon, EffectiveHalfWidthClampsToTheHalfWidthFloor) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_margin_extreme"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-5, 0, 0}, {5, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::LOCAL;
    ribbon.points = pts;
    ribbon.point_count = 2;
    overlume::SceneGraph s{};
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    EXPECT_NEAR(overlume::testing::ribbon_slot_half_width_m(r, 0), 0.12f, 1e-4f);
    overlume::destroy_renderer(r);
}

TEST(Ribbon, ClipStartsAtInterpolatedPointWhenEgoIsMidRibbon) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-10, 0, 0}, {-5, 0, 0}, {5, 0, 0}, {10, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::GLOBAL;
    ribbon.points = pts;
    ribbon.point_count = 4;
    overlume::SceneGraph s{};
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 10}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    overlume::Vec3 firstPoint{};
    ASSERT_TRUE(overlume::testing::ribbon_slot_first_point(r, 0, &firstPoint));
    EXPECT_NEAR(firstPoint.x, 0.0, 0.5) << "clip station should land near x=0, the ego's own "
                                           "closest-approach point on the ribbon";
    EXPECT_GE(firstPoint.x, 0.0) << "clipped geometry still starts behind the ego";
    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), 4u * 2)
        << "clip must not change vertex/mesh count -- it's a position collapse, never a rebuild";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, ProximityGateSkipsClipWhenEgoIsFarFromTheRibbon) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-10, 0, 0}, {-5, 0, 0}, {5, 0, 0}, {10, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::GLOBAL;
    ribbon.points = pts;
    ribbon.point_count = 4;
    overlume::SceneGraph s{};
    s.ego = {{0, 20, 0}, 0.0, 0.0, 1};
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 30}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), 4u * 2)
        << "a ribbon the ego is nowhere near must render whole, not clipped";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, ClipAppliesOnlyWhenEgoIsValid) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-10, 0, 0}, {-5, 0, 0}, {5, 0, 0}, {10, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::GLOBAL;
    ribbon.points = pts;
    ribbon.point_count = 4;
    overlume::SceneGraph s{};
    s.ego = {{0, 0, 0}, 0.0, 0.0, 0};
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 10}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    EXPECT_EQ(overlume::testing::ribbon_vertex_count(r, 0), 4u * 2)
        << "an invalid ego must never clip a ribbon";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, ParkedEgoCausesZeroRibbonRebuilds) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-10, 0, 0}, {-5, 0, 0}, {5, 0, 0}, {10, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::GLOBAL;
    ribbon.points = pts;
    ribbon.point_count = 4;
    overlume::SceneGraph s{};
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::CameraPose pose{{0, -8, 10}, {0, 0, 0}, 60.0};

    overlume::set_scene(r, s);
    render_once(r, pose);
    const uint64_t afterFirst = overlume::testing::ribbon_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);

    for (int i = 0; i < 10; ++i) {
        overlume::set_scene(r, s);
        render_once(r, pose);
    }
    EXPECT_EQ(overlume::testing::ribbon_rebuild_count(r), afterFirst)
        << "a parked ego re-triggered rebuilds -- the quantized clip station isn't stable";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, FreshRibbonsFadeMaterialWhenThemeEnablesOpacityAndLengthFade) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_fade"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::testing::RibbonScene ribbons = overlume::testing::make_three_role_ribbons(10.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = ribbons.ego;
    s.paths = ribbons.ribbons.data();
    s.path_count = static_cast<uint32_t>(ribbons.ribbons.size());
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{-4, -8, 6}, {4, 1, 0}, 60.0};
    render_once(r, pose);

    for (size_t i = 0; i < ribbons.ribbons.size(); ++i) {
        const auto info = overlume::testing::ribbon_slot_material_info(r, i);
        EXPECT_TRUE(info.bound_to_translucent) << "slot " << i;
        EXPECT_NEAR(info.alpha, 0.6f, 1e-4f) << "slot " << i;
    }
    overlume::destroy_renderer(r);
}

TEST(Ribbon, FreshRibbonsStayOpaqueWhenThemeDisablesOpacityAndLengthFade) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_margin_a"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::testing::RibbonScene ribbons = overlume::testing::make_three_role_ribbons(10.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = ribbons.ego;
    s.paths = ribbons.ribbons.data();
    s.path_count = static_cast<uint32_t>(ribbons.ribbons.size());
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{-4, -8, 6}, {4, 1, 0}, 60.0};
    render_once(r, pose);

    for (size_t i = 0; i < ribbons.ribbons.size(); ++i) {
        const auto info = overlume::testing::ribbon_slot_material_info(r, i);
        EXPECT_FALSE(info.bound_to_translucent) << "slot " << i;
        EXPECT_FLOAT_EQ(info.alpha, 1.0f) << "slot " << i;
    }
    overlume::destroy_renderer(r);
}

TEST(Ribbon, FreshRibbonsFadeMaterialWhenThemeEnablesLengthFadeOnly) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_fade_b"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::testing::RibbonScene ribbons = overlume::testing::make_three_role_ribbons(10.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = ribbons.ego;
    s.paths = ribbons.ribbons.data();
    s.path_count = static_cast<uint32_t>(ribbons.ribbons.size());
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{-4, -8, 6}, {4, 1, 0}, 60.0};
    render_once(r, pose);

    for (size_t i = 0; i < ribbons.ribbons.size(); ++i) {
        const auto info = overlume::testing::ribbon_slot_material_info(r, i);
        EXPECT_TRUE(info.bound_to_translucent)
            << "slot " << i
            << " -- opacity==1.0 with a metre fade enabled must still bind the blended material";
        EXPECT_FLOAT_EQ(info.alpha, 1.0f) << "slot " << i;
    }
    overlume::destroy_renderer(r);
}

TEST(Ribbon, LengthFadeRampReachesZeroAtEndOfLongMultiChunkRibbon) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    constexpr uint32_t kN = 40000;
    std::vector<overlume::Vec3> pts(kN);
    for (uint32_t i = 0; i < kN; ++i) pts[i] = {static_cast<double>(i) * 0.1, 0.0, 0.0};
    ASSERT_GT(kN, overlume::detail::kMaxPointsPerMesh)
        << "fixture must span more than one chunk to cover the whole-ribbon (not per-chunk) "
           "length fade";

    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::LOCAL;
    ribbon.points = pts.data();
    ribbon.point_count = kN;
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = {0.0, 0.0, 0.0};
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};

    {
        overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_margin_a"};
        auto* r = overlume::create_renderer(cfg);
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        overlume::set_scene(r, s);
        render_once(r, pose);
        const auto info = overlume::testing::ribbon_slot_material_info(r, 0);
        EXPECT_FLOAT_EQ(info.minVertexAlpha, 1.0f)
            << "no length fade configured -- every vertex must stay fully opaque";
        overlume::destroy_renderer(r);
    }
    {
        const std::string fixtureDir =
            std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
        overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_fade"};
        auto* r = overlume::create_renderer(cfg);
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        overlume::set_scene(r, s);
        render_once(r, pose);
        const auto info = overlume::testing::ribbon_slot_material_info(r, 0);
        EXPECT_NEAR(info.minVertexAlpha, 0.0f, 1e-3f)
            << "the far end of a ribbon longer than kMaxPointsPerMesh never reaches alpha 0 -- "
               "the length fade ramp is being computed against a per-chunk length instead of "
               "the whole ribbon, or the ramp isn't reaching build_slot_meshes/apply_ribbon_clip "
               "at all";
        overlume::destroy_renderer(r);
    }
}

TEST(Ribbon, FadeStartChangeRebuildsGeometry) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_fade"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{-5, 0, 0}, {-3, 0, 0}, {-1, 0, 0}, {1, 0, 0}, {3, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::LOCAL;
    ribbon.points = pts;
    ribbon.point_count = 5;
    ribbon.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    const uint64_t before = overlume::testing::ribbon_rebuild_count(r);

    ASSERT_TRUE(overlume::set_theme(r, "ribbon_fade_b", 0.0, 0.2));
    s.sim_time_sec = 0.2;
    ribbon.last_update_sec = 0.2;
    overlume::set_scene(r, s);
    render_once(r, pose);

    EXPECT_GT(overlume::testing::ribbon_rebuild_count(r), before)
        << "a fade-metres-only theme change (no PathRibbon point data touched) did not rebuild "
           "slot 0's geometry -- fade_start_m/fade_end_m aren't part of the slot signature";
    overlume::destroy_renderer(r);
}

namespace {

std::vector<uint8_t> render_fade_probe(const std::string& fixtureDir, const char* theme,
                                       overlume::PathRole role, bool withRibbon) {
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), theme};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return {};
    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = role;
    ribbon.points = pts;
    ribbon.point_count = 2;
    ribbon.last_update_sec = 10.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.ego.position = {0.0, 0.0, 0.0};
    s.paths = &ribbon;
    s.path_count = withRibbon ? 1 : 0;
    overlume::set_scene(r, s);
    const overlume::CameraPose pose{{5, -6, 14}, {5, 0, 0}, 60.0};
    std::vector<uint8_t> px = render_once(r, pose);
    overlume::destroy_renderer(r);
    return px;
}

int max_channel_delta(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, size_t i) {
    int d = 0;
    for (size_t c = 0; c < 3; ++c) d = std::max(d, std::abs(int(a[i + c]) - int(b[i + c])));
    return d;
}

}

TEST(Ribbon, FadeIsMeasuredFromTheEgoClipStationNotTheRibbonStart) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_fade"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::Vec3> pts(101);
    for (size_t i = 0; i < pts.size(); ++i) pts[i] = {static_cast<double>(i), 0.0, 0.0};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::LOCAL;
    ribbon.points = pts.data();
    ribbon.point_count = static_cast<uint32_t>(pts.size());
    ribbon.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.ego.valid = 1;
    s.ego.position = {60.0, 0.0, 0.0};
    s.paths = &ribbon;
    s.path_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{60, -10, 10}, {65, 0, 0}, 60.0});

    EXPECT_NEAR(overlume::testing::ribbon_vertex_fade_alpha(r, 0, 130), 1.0f, 1e-3f)
        << "5 m ahead of the ego (station 65) is still inside fade_start_m";
    EXPECT_NEAR(overlume::testing::ribbon_vertex_fade_alpha(r, 0, 134), 0.6f, 1e-3f)
        << "7 m ahead is 40% into the ramp although it is only 7% along a 100 m ribbon";
    EXPECT_NEAR(overlume::testing::ribbon_vertex_fade_alpha(r, 0, 140), 0.0f, 1e-3f)
        << "10 m ahead of the ego the ribbon is gone";

    s.ego.position = {60.0, 10.0, 0.0};
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{60, -10, 10}, {65, 0, 0}, 60.0});
    EXPECT_NEAR(overlume::testing::ribbon_vertex_fade_alpha(r, 0, 140), 1.0f, 1e-3f)
        << "ego 10 m off the polyline: the clip is inactive, so the ramp must switch off "
           "rather than fade the whole ribbon from its first point";

    s.ego.valid = 0;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{60, -10, 10}, {65, 0, 0}, 60.0});
    EXPECT_NEAR(overlume::testing::ribbon_vertex_fade_alpha(r, 0, 200), 1.0f, 1e-3f)
        << "no ego at all: everything stays visible";
    overlume::destroy_renderer(r);
}

TEST(Ribbon, OpacityAndLengthFadeReachTheFramebuffer) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    for (overlume::PathRole role : {overlume::PathRole::LOCAL, overlume::PathRole::BEHAVIOR}) {
        const auto ground = render_fade_probe(fixtureDir, "ribbon_margin_a", role, false);
        if (ground.empty()) GTEST_SKIP() << "no GPU/EGL";
        const auto opaque = render_fade_probe(fixtureDir, "ribbon_margin_a", role, true);
        const auto faded = render_fade_probe(fixtureDir, "ribbon_fade", role, true);
        ASSERT_EQ(opaque.size(), ground.size());
        ASSERT_EQ(faded.size(), ground.size());

        std::vector<int> columns(320, 0);
        size_t ribbonPixels = 0, changed = 0;
        for (size_t i = 0; i < ground.size(); i += 3) {
            if (max_channel_delta(opaque, ground, i) <= 40) continue;
            ++ribbonPixels;
            ++columns[(i / 3) % 320];
            if (max_channel_delta(faded, opaque, i) > 12) ++changed;
        }
        ASSERT_GT(ribbonPixels, 200u) << "role " << int(role);
        EXPECT_GT(changed, ribbonPixels / 2)
            << "role " << int(role) << ": opacity 0.6 must visibly blend the ribbon, but only "
            << changed << " of " << ribbonPixels << " ribbon pixels differ from the opaque render";

        int x0 = 0, x1 = 319;
        while (x0 < 320 && columns[x0] == 0) ++x0;
        while (x1 > 0 && columns[x1] == 0) --x1;
        ASSERT_LT(x0 + 20, x1) << "role " << int(role);
        const int span = x1 - x0;
        auto band_mean = [&](int from, int to) {
            double sum = 0.0;
            size_t n = 0;
            for (size_t i = 0; i < ground.size(); i += 3) {
                const int x = static_cast<int>((i / 3) % 320);
                if (x < from || x > to || max_channel_delta(opaque, ground, i) <= 40) continue;
                sum += max_channel_delta(faded, ground, i);
                ++n;
            }
            return n ? sum / static_cast<double>(n) : 0.0;
        };
        const double nearMean = band_mean(x0, x0 + span * 2 / 5);
        const double farMean = band_mean(x1 - span / 10, x1);
        EXPECT_LT(farMean, nearMean * 0.35)
            << "role " << int(role)
            << ": with fade_end_m 10 on a 10 m ribbon the far end must "
               "approach the ground colour (near-band mean delta "
            << nearMean << ", far-band mean delta " << farMean << ")";
    }
}
