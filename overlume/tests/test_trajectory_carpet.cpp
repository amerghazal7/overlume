// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "trajectory_carpet_test_hooks.hpp"
#include "test_paths.hpp"
#include "polyline.hpp"

#include <algorithm>
#include <cstdint>
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

std::vector<overlume::PointCloudPoint> make_stations(uint32_t n, uint32_t rgba) {
    std::vector<overlume::PointCloudPoint> pts(n);
    for (uint32_t i = 0; i < n; ++i) {
        pts[i].position = {static_cast<double>(i), 0.0, 0.0};
        pts[i].rgba = rgba;
    }
    return pts;
}

}

TEST(TrajectoryCarpet, BuildsExtrudedRibbonFromCenterlineStations) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::trajectory_carpet_mesh_count(r, 0), 1u);
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_count(r, 0), 8u);
    EXPECT_NEAR(overlume::testing::trajectory_carpet_half_width_m(r, 0), 0.7f, 1e-4f);
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, MarginVelocityChangeRebuildsGeometryAtNewHalfWidth) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_margin_velocity"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_NEAR(overlume::testing::trajectory_carpet_half_width_m(r, 0), 0.85f, 1e-4f);
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, EffectiveHalfWidthClampsToTheHalfWidthFloor) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_margin_velocity_extreme"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(2, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_NEAR(overlume::testing::trajectory_carpet_half_width_m(r, 0), 0.12f, 1e-4f);
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, PerVertexColorPassesThroughUnchangedWhenAlphaByteIsNonzero) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0};
    pts[0].rgba = 0xB30000FFu;
    pts[1].position = {1, 0, 0};
    pts[1].rgba = 0xB300FF00u;
    pts[2].position = {2, 0, 0};
    pts[2].rgba = 0xB3FF0080u;

    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xB30000FFu);
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 1), 0xB30000FFu);
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 2), 0xB300FF00u);
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 3), 0xB300FF00u);
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 4), 0xB3FF0080u);
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 5), 0xB3FF0080u);
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, AlphaZeroSentinelSubstitutesPaletteObjectTintsUnknown) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0};
    pts[1].position = {1, 0, 0};
    pts[2].position = {2, 0, 0};

    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    constexpr uint32_t kExpected = 115u | (115u << 8) | (128u << 16) | (255u << 24);
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, i), kExpected);
    }
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, RibbonChunksAcrossMeshesWithoutTruncation) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    constexpr uint32_t kN = 40000;
    std::vector<overlume::PointCloudPoint> pts(kN);
    for (uint32_t i = 0; i < kN; ++i) {
        pts[i].position = {static_cast<double>(i) * 0.1, 0.0, 0.0};
        pts[i].rgba = 0xFF0000FFu;
    }
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = kN;
    tc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -20, 20}, {2, 0, 0}, 60.0};
    render_once(r, pose);

    const auto chunks = overlume::detail::polyline_chunks(kN);
    ASSERT_EQ(chunks.size(), 2u) << "40000 stations should split into exactly 2 chunks at "
                                    "kMaxPointsPerMesh=32000 -- fixture assumption changed?";
    size_t expectedVerts = 0;
    for (const auto& [a, b] : chunks) expectedVerts += 2 * static_cast<size_t>(b - a);

    EXPECT_EQ(overlume::testing::trajectory_carpet_mesh_count(r, 0), chunks.size());
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_count(r, 0), expectedVerts)
        << "vertex count doesn't match polyline_chunks()'s own math -- a station was lost "
           "at the chunk-split seam";
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, SlotReleasedWhenCarpetCountDrops) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(3, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    ASSERT_EQ(overlume::testing::trajectory_carpet_mesh_count(r, 0), 1u);

    overlume::SceneGraph empty{};
    overlume::set_scene(r, empty);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_EQ(overlume::testing::trajectory_carpet_mesh_count(r, 0), 0u)
        << "a slot past the new (lower) trajectory_carpet_count must be torn down, not left "
           "dangling";

    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, MaterialAlphaFollowsStalenessOpaqueWhileFresh) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_margin_a"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(3, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_FLOAT_EQ(overlume::testing::trajectory_carpet_material_alpha(r), 1.0f);

    overlume::SceneGraph stale = s;
    stale.sim_time_sec = 5.0;
    overlume::set_scene(r, stale);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_FLOAT_EQ(overlume::testing::trajectory_carpet_material_alpha(r), 0.0f);

    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, VertexZIsLiftedAboveTheFlattenedZeroTheAdapterSends) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0.0};
    pts[1].position = {1, 0, 0.0};
    pts[2].position = {2, 0, 0.0};
    for (auto& p : pts) p.rgba = 0xFF0000FFu;

    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    for (size_t i = 0; i < 6; ++i) {
        const float z = overlume::testing::trajectory_carpet_vertex_z(r, 0, i);
        EXPECT_GT(z, 0.046f) << "vertex " << i
                             << " must be lifted ABOVE LOCAL's own z-lift (0.046) -- "
                                "\"stacked on top of local ribbon\" per the user directive";
        EXPECT_LT(z, 0.058f) << "vertex " << i
                             << " must stay BELOW BEHAVIOR's z-lift (0.058) -- the hero ribbon "
                                "must remain topmost of the path/ribbon stack";
    }
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ClipCollapsesGeometryWhenEgoIsMidCarpet) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.ego = {{1, 0, 0}, 0.0, 0.0, 1};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {2, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_count(r, 0), 5u * 2)
        << "clip must not change vertex/mesh count -- it's a position collapse, never a rebuild";
    overlume::Vec3 firstPoint{};
    ASSERT_TRUE(overlume::testing::trajectory_carpet_slot_first_point(r, 0, &firstPoint));
    EXPECT_NEAR(firstPoint.x, 1.0, 0.5) << "clip station should land near x=1, the ego's own "
                                           "closest-approach point on the carpet";
    EXPECT_GE(firstPoint.x, 1.0) << "clipped geometry still starts behind the ego";
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ProximityGateSkipsClipWhenEgoIsFarFromTheCarpet) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.ego = {{1, 20, 0}, 0.0, 0.0, 1};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 30}, {2, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_count(r, 0), 5u * 2)
        << "a carpet the ego is nowhere near must render whole, not clipped";
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ClipAppliesOnlyWhenEgoIsValid) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.ego = {{1, 0, 0}, 0.0, 0.0, 0};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {2, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_count(r, 0), 5u * 2)
        << "an invalid ego must never clip a carpet";
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ParkedEgoCausesZeroTrajectoryCarpetRebuilds) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.ego = {{1, 0, 0}, 0.0, 0.0, 1};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::CameraPose pose{{0, -10, 10}, {2, 0, 0}, 60.0};

    overlume::set_scene(r, s);
    render_once(r, pose);
    const uint64_t afterFirst = overlume::testing::trajectory_carpet_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);

    for (int i = 0; i < 10; ++i) {
        overlume::set_scene(r, s);
        render_once(r, pose);
    }
    EXPECT_EQ(overlume::testing::trajectory_carpet_rebuild_count(r), afterFirst)
        << "a parked ego re-triggered rebuilds -- the quantized clip station isn't stable";
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, SameStationPositionsWithDriftingColorAloneCausesNoRebuild) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    const uint64_t afterFirst = overlume::testing::trajectory_carpet_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xFF0000FFu);

    for (uint32_t i = 0; i < 20; ++i) {
        for (auto& p : pts) p.rgba = 0xFF000000u | (i + 1);
        tc.last_update_sec = 0.0;
        overlume::set_scene(r, s);
        render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    }

    EXPECT_EQ(overlume::testing::trajectory_carpet_rebuild_count(r), afterFirst)
        << "color-only drift (identical station positions) rebuilt the mesh -- this is the "
           "exact VM-077 H2 flicker regression: color must not be part of the content signature";
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xFF0000FFu);
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, IdenticalRepublishCausesNoRebuild) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;

    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    const uint64_t afterFirst = overlume::testing::trajectory_carpet_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);

    for (int i = 0; i < 5; ++i) {
        overlume::set_scene(r, s);
        render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    }
    EXPECT_EQ(overlume::testing::trajectory_carpet_rebuild_count(r), afterFirst);
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ThemeOpacityAndLengthFadeApplyToFreshCarpet) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_fade"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(11, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{5, -10, 10}, {5, 0, 0}, 60.0});

    EXPECT_NEAR(overlume::testing::trajectory_carpet_material_alpha(r), 0.6f, 1e-5f)
        << "ribbon.opacity must multiply the carpet's material alpha even while fresh";
    EXPECT_EQ(overlume::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xFF0000FFu)
        << "the resolved data colour is untouched; the fade lives in the uploaded alpha byte";
    EXPECT_NEAR(overlume::testing::trajectory_carpet_vertex_fade_alpha(r, 0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(overlume::testing::trajectory_carpet_vertex_fade_alpha(r, 0, 10), 1.0f, 1e-5f)
        << "station 5 of 10 sits exactly at fade_start 0.5 and is still fully opaque";
    EXPECT_NEAR(overlume::testing::trajectory_carpet_vertex_fade_alpha(r, 0, 14), 0.6f, 1e-5f)
        << "station 7 of 10 is 40% into the ramp";
    EXPECT_NEAR(overlume::testing::trajectory_carpet_vertex_fade_alpha(r, 0, 21), 0.0f, 1e-5f)
        << "the last vertex pair reaches zero";
    overlume::destroy_renderer(r);
}

TEST(TrajectoryCarpet, FadeStartChangeRebuildsGeometry) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "ribbon_fade"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    const uint64_t before = overlume::testing::trajectory_carpet_rebuild_count(r);

    ASSERT_TRUE(overlume::set_theme(r, "ribbon_fade_b", 0.0, 0.2));
    s.sim_time_sec = 0.2;
    tc.last_update_sec = 0.2;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_GT(overlume::testing::trajectory_carpet_rebuild_count(r), before)
        << "a fade_start-only theme change must rebuild the carpet so the ramp is re-baked";
    overlume::destroy_renderer(r);
}

namespace {

std::vector<uint8_t> render_carpet_probe(const std::string& fixtureDir, const char* theme,
                                         bool withCarpet) {
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), theme};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return {};
    const overlume::PointCloudPoint pts[] = {{{0, 0, 0}, 0xFF3399FFu}, {{10, 0, 0}, 0xFF3399FFu}};
    overlume::TrajectoryCarpet tc{};
    tc.points = pts;
    tc.point_count = 2;
    tc.last_update_sec = 10.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = withCarpet ? 1 : 0;
    overlume::set_scene(r, s);
    std::vector<uint8_t> px = render_once(r, overlume::CameraPose{{5, -6, 14}, {5, 0, 0}, 60.0});
    overlume::destroy_renderer(r);
    return px;
}

int carpet_channel_delta(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, size_t i) {
    int d = 0;
    for (size_t c = 0; c < 3; ++c) d = std::max(d, std::abs(int(a[i + c]) - int(b[i + c])));
    return d;
}

}

TEST(TrajectoryCarpet, OpacityAndLengthFadeReachTheFramebuffer) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const auto ground = render_carpet_probe(fixtureDir, "ribbon_margin_a", false);
    if (ground.empty()) GTEST_SKIP() << "no GPU/EGL";
    const auto opaque = render_carpet_probe(fixtureDir, "ribbon_margin_a", true);
    const auto faded = render_carpet_probe(fixtureDir, "ribbon_fade", true);
    ASSERT_EQ(opaque.size(), ground.size());
    ASSERT_EQ(faded.size(), ground.size());

    std::vector<int> columns(320, 0);
    size_t carpetPixels = 0, changed = 0;
    for (size_t i = 0; i < ground.size(); i += 3) {
        if (carpet_channel_delta(opaque, ground, i) <= 40) continue;
        ++carpetPixels;
        ++columns[(i / 3) % 320];
        if (carpet_channel_delta(faded, opaque, i) > 12) ++changed;
    }
    ASSERT_GT(carpetPixels, 200u);
    EXPECT_GT(changed, carpetPixels / 2)
        << "opacity 0.6 must visibly blend the carpet, but only " << changed << " of "
        << carpetPixels << " carpet pixels differ from the opaque render";

    int x0 = 0, x1 = 319;
    while (x0 < 320 && columns[x0] == 0) ++x0;
    while (x1 > 0 && columns[x1] == 0) --x1;
    ASSERT_LT(x0 + 20, x1);
    const int span = x1 - x0;
    auto band_mean = [&](int from, int to) {
        double sum = 0.0;
        size_t n = 0;
        for (size_t i = 0; i < ground.size(); i += 3) {
            const int x = static_cast<int>((i / 3) % 320);
            if (x < from || x > to || carpet_channel_delta(opaque, ground, i) <= 40) continue;
            sum += carpet_channel_delta(faded, ground, i);
            ++n;
        }
        return n ? sum / static_cast<double>(n) : 0.0;
    };
    const double nearMean = band_mean(x0, x0 + span * 2 / 5);
    const double farMean = band_mean(x1 - span / 10, x1);
    EXPECT_LT(farMean, nearMean * 0.35)
        << "with fade_start 0.5 the far end of the carpet must approach the ground colour "
           "(near-band mean delta "
        << nearMean << ", far-band mean delta " << farMean << ")";
}
