// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "map_elements_test_hooks.hpp"
#include "polyline.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <utility>
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

TEST(Ground, FollowsEgoQuantizedToGridPitch) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    s.ego = {{120.4, -80.6, 0.0}, 0.0, 0.0, 1};
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{120, -88, 4}, {120, -80, 0}, 60.0};
    render_once(r, pose);
    auto c = overlume::testing::ground_patch_centre(r);
    EXPECT_NEAR(c.x, 120.0, 1e-6);
    EXPECT_NEAR(c.y, -80.0, 1e-6);
    overlume::destroy_renderer(r);
}

TEST(Ground, PatchSnapsAWholeCellAtATime) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    s.ego = {{121.4, -80.6, 0.0}, 0.0, 0.0, 1};
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{121, -88, 4}, {121, -80, 0}, 60.0};
    render_once(r, pose);
    auto c = overlume::testing::ground_patch_centre(r);
    EXPECT_NEAR(c.x, 122.0, 1e-6);
    EXPECT_NEAR(c.y, -80.0, 1e-6);
    overlume::destroy_renderer(r);
}

TEST(Ground, NoEgoYet_StaysAtOrigin) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    s.ego = {{500.0, 500.0, 0.0}, 0.0, 0.0, 0};
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    auto c = overlume::testing::ground_patch_centre(r);
    EXPECT_NEAR(c.x, 0.0, 1e-6);
    EXPECT_NEAR(c.y, 0.0, 1e-6);
    overlume::destroy_renderer(r);
}

TEST(Ground, EpicOneEmptyWorldGoldenStillMatches) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png",
        OVERLUME_TMP_DIR "/map_elements_empty_world_regression_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
    overlume::destroy_renderer(r);
}

TEST(MapElements, LaneMaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement elem{};
    elem.points = pts;
    elem.point_count = 2;
    elem.is_polygon = 0;
    overlume::SceneGraph s{};
    s.map_elements = &elem;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    auto p = overlume::testing::lane_material_base_color(r);
    EXPECT_NEAR(p.r, theme->palette.lane_paint.r, 1e-4);
    EXPECT_NEAR(p.g, theme->palette.lane_paint.g, 1e-4);
    EXPECT_NEAR(p.b, theme->palette.lane_paint.b, 1e-4);
    overlume::destroy_renderer(r);
}

TEST(MapElements, KindDrivesMaterialDispatchToTheMatchingThemeToken) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement elem{};
    elem.points = pts;
    elem.point_count = 2;
    elem.is_polygon = 0;
    overlume::SceneGraph s{};
    s.map_elements = &elem;
    s.map_element_count = 1;

    auto expect_kind_color = [&](overlume::MapKind kind, const overlume::detail::Float3& expected) {
        elem.kind = kind;
        overlume::set_scene(r, s);
        overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
        render_once(r, pose);
        auto p = overlume::testing::map_kind_base_color(r, kind);
        EXPECT_NEAR(p.r, expected.r, 1e-4) << "kind=" << static_cast<int>(kind);
        EXPECT_NEAR(p.g, expected.g, 1e-4) << "kind=" << static_cast<int>(kind);
        EXPECT_NEAR(p.b, expected.b, 1e-4) << "kind=" << static_cast<int>(kind);
    };

    expect_kind_color(overlume::MapKind::CENTERLINE, theme->palette.lane_centerline);
    expect_kind_color(overlume::MapKind::LEFT_BOUNDARY, theme->palette.lane_boundary);
    expect_kind_color(overlume::MapKind::RIGHT_BOUNDARY, theme->palette.lane_boundary);
    expect_kind_color(overlume::MapKind::CROSSWALK, theme->palette.crosswalk);
    expect_kind_color(overlume::MapKind::ROAD_SURFACE, theme->palette.road);
    expect_kind_color(overlume::MapKind::ROAD_EDGE, theme->palette.road_edge);
    expect_kind_color(overlume::MapKind::STOPLINE, theme->palette.lane_paint);

    overlume::destroy_renderer(r);
}

TEST(MapElements, CrosswalkHatchFiresOnRecordedFivePointClosedPolyline) {
    const overlume::Vec3 pts_after_dedupe[4] = {
        {-39.50850289011474, 45.33743457749722, -0.000284586101770401},
        {-54.35884356129442, 45.2288915511252, -0.00039308611303567886},
        {-54.476791014440394, 47.18497371095612, -0.0004083588719367981},
        {-39.52170872121907, 47.27801090662989, -0.0002988511696457863},
    };
    auto tris = overlume::detail::build_crosswalk_hatch(pts_after_dedupe, 4, 0.02f);
    EXPECT_FALSE(tris.empty());
}

TEST(MapElements, CrosswalkHatchBarsAreOrientedAlongTheShortAxis) {
    const overlume::Vec3 pts[4] = {
        {-39.50850289011474, 45.33743457749722, -0.000284586101770401},
        {-54.35884356129442, 45.2288915511252, -0.00039308611303567886},
        {-54.476791014440394, 47.18497371095612, -0.0004083588719367981},
        {-39.52170872121907, 47.27801090662989, -0.0002988511696457863},
    };
    auto tris = overlume::detail::build_crosswalk_hatch(pts, 4, 0.0f);
    ASSERT_FALSE(tris.empty());
    ASSERT_EQ(tris.size() % 6, 0u) << "not a whole number of 2-triangle stripe quads";

    const auto& a0 = tris[0];
    const auto& b0 = tris[5];
    const double barDx = b0.x - a0.x, barDy = b0.y - a0.y;
    const double barLen = std::sqrt(barDx * barDx + barDy * barDy);

    auto edge_vec = [&](int i, int j) {
        return std::pair<double, double>{pts[j].x - pts[i].x, pts[j].y - pts[i].y};
    };
    auto cos_angle = [&](std::pair<double, double> e) {
        const double eLen = std::sqrt(e.first * e.first + e.second * e.second);
        if (barLen <= 0.0 || eLen <= 0.0) return 0.0;
        return std::abs((barDx * e.first + barDy * e.second) / (barLen * eLen));
    };
    const double cosShort = std::max(cos_angle(edge_vec(1, 2)), cos_angle(edge_vec(3, 0)));
    const double cosLong = std::max(cos_angle(edge_vec(0, 1)), cos_angle(edge_vec(2, 3)));
    EXPECT_GT(cosShort, 0.99) << "bar long axis should be ~parallel to the quad's SHORT edges";
    EXPECT_LT(cosLong, 0.2) << "bar long axis should be ~perpendicular to the quad's LONG edges";
}

TEST(MapElements, CrosswalkHatchStripeCountIsPitchDerivedOnRealFixture) {
    const overlume::Vec3 pts[4] = {
        {-39.50850289011474, 45.33743457749722, -0.000284586101770401},
        {-54.35884356129442, 45.2288915511252, -0.00039308611303567886},
        {-54.476791014440394, 47.18497371095612, -0.0004083588719367981},
        {-39.52170872121907, 47.27801090662989, -0.0002988511696457863},
    };
    auto tris = overlume::detail::build_crosswalk_hatch(pts, 4, 0.0f);
    ASSERT_FALSE(tris.empty());
    ASSERT_EQ(tris.size() % 6, 0u);
    EXPECT_EQ(tris.size() / 6, 12u) << "5 fixed bars across a ~15m crossing was the old, wrong "
                                       "behavior -- stripe count must scale with crossing length";
}

TEST(MapElements, CrosswalkHatchStripeCountClampsToRange) {
    const overlume::Vec3 tiny[4] = {{0, 0, 0}, {1, 0, 0}, {1, 2, 0}, {0, 2, 0}};
    auto trisTiny = overlume::detail::build_crosswalk_hatch(tiny, 4, 0.0f);
    ASSERT_FALSE(trisTiny.empty());
    EXPECT_EQ(trisTiny.size() / 6, 3u);

    const overlume::Vec3 huge[4] = {{0, 0, 0}, {200, 0, 0}, {200, 4, 0}, {0, 4, 0}};
    auto trisHuge = overlume::detail::build_crosswalk_hatch(huge, 4, 0.0f);
    ASSERT_FALSE(trisHuge.empty());
    EXPECT_EQ(trisHuge.size() / 6, 24u);
}

TEST(MapElementsGolden, DashedBoundaryProducesSameDashRunsAsThePreMoveAlgorithm) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::LEFT_BOUNDARY;
    overlume::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::map_element_mesh_count(r), 4u);
    overlume::destroy_renderer(r);
}

TEST(MapElementsGolden, CenterlineOfSameGeometryProducesOneMeshChunkNotDashSplit) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    overlume::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::map_element_mesh_count(r), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapElementsGolden, RoadEdgeOfSameGeometryProducesOneMeshChunkNeverDashed) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::ROAD_EDGE;
    overlume::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::map_element_mesh_count(r), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapElements, CenterlineRendersAsDotDiscsNotAStrip) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    overlume::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::map_element_mesh_count(r), 1u);
    EXPECT_EQ(overlume::testing::map_element_total_vertex_count(r), 180u)
        << "expected 6 dots * 10 segments * 3 verts/wedge -- a strip of this same "
           "2-point geometry would be 6 vertices, not 180";
    overlume::destroy_renderer(r);
}

TEST(MapElements, RoadSurfaceKindTriangulatesTheTwoRailEncodingIntoAStrip) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};

    auto* baseR = overlume::create_renderer(cfg);
    if (!baseR) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph empty{};
    overlume::set_scene(baseR, empty);
    const std::vector<uint8_t> baseline = render_once(baseR, pose);
    overlume::destroy_renderer(baseR);

    auto* r = overlume::create_renderer(cfg);
    ASSERT_TRUE(r);
    constexpr uint32_t kN = 16;
    overlume::Vec3 pts[2 * kN];
    for (uint32_t i = 0; i < kN; ++i) {
        const double y = -3.0 + 6.0 * static_cast<double>(i) / static_cast<double>(kN - 1);
        pts[i] = {-1.0, y, 0.0};
        pts[kN + i] = {1.0, y, 0.0};
    }
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2 * kN;
    e.kind = overlume::MapKind::ROAD_SURFACE;
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    const std::vector<uint8_t> withRoad = render_once(r, pose);
    overlume::destroy_renderer(r);

    ASSERT_EQ(baseline.size(), withRoad.size());
    size_t differing = 0;
    for (size_t i = 0; i < baseline.size(); ++i) {
        if (baseline[i] != withRoad[i]) ++differing;
    }
    EXPECT_GT(differing, 0u) << "a ROAD_SURFACE element produced no visible pixel "
                                "difference from a scene with no map data at all";
}

TEST(MapElements, RoadSurfaceMalformedPointCountBuildsNothing) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::Vec3 pts[5] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 5;
    e.kind = overlume::MapKind::ROAD_SURFACE;
    overlume::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::map_element_mesh_count(r), 0u);
    overlume::destroy_renderer(r);
}

TEST(MapElements, RebuildCountStaysZeroOnUnchangedContentSignature) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    overlume::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};

    overlume::set_scene(r, s);
    render_once(r, pose);
    const uint64_t afterFirst = overlume::testing::map_element_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);

    overlume::set_scene(r, s);
    render_once(r, pose);
    EXPECT_EQ(overlume::testing::map_element_rebuild_count(r), afterFirst)
        << "publishing the identical MapElement a second time triggered a rebuild -- "
           "the content-signature cache isn't actually a cache";
    overlume::destroy_renderer(r);
}

TEST(MapElements, SyntheticLaneAndCrosswalkChangePixelsVsBaseline) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};

    auto* baseR = overlume::create_renderer(cfg);
    if (!baseR) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph empty{};
    overlume::set_scene(baseR, empty);
    const std::vector<uint8_t> baseline = render_once(baseR, pose);
    overlume::destroy_renderer(baseR);

    auto* r = overlume::create_renderer(cfg);
    ASSERT_TRUE(r);
    const overlume::Vec3 lanePts[] = {{0, -6, 0}, {0, -2, 0}, {0, 2, 0}, {0, 6, 0}};
    const overlume::Vec3 crosswalkPts[] = {
        {-1.5, -0.5, 0}, {1.5, -0.5, 0}, {1.5, 0.5, 0}, {-1.5, 0.5, 0}};
    overlume::MapElement elems[2]{};
    elems[0].points = lanePts;
    elems[0].point_count = 4;
    elems[0].is_polygon = 0;
    elems[1].points = crosswalkPts;
    elems[1].point_count = 4;
    elems[1].is_polygon = 1;
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.map_elements = elems;
    s.map_element_count = 2;
    overlume::set_scene(r, s);
    const std::vector<uint8_t> withMap = render_once(r, pose);
    overlume::testing::render_and_compare(r, pose, "/nonexistent-golden.png",
                                          OVERLUME_TMP_DIR "/map_elements_synthetic_actual.png");
    overlume::destroy_renderer(r);

    ASSERT_EQ(baseline.size(), withMap.size());
    size_t differing = 0;
    for (size_t i = 0; i < baseline.size(); ++i) {
        if (baseline[i] != withMap[i]) ++differing;
    }
    EXPECT_GT(differing, 0u) << "lane + crosswalk map elements produced no visible pixel "
                                "difference from a scene with no map data at all";
}

TEST(MapElements, ElementCountShrinksWhenElementsVanishBetweenUpdates) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 a[] = {{-4, -4, 0}, {-4, 4, 0}};
    const overlume::Vec3 b[] = {{0, -4, 0}, {0, 4, 0}};
    const overlume::Vec3 c[] = {{4, -4, 0}, {4, 4, 0}};
    overlume::MapElement three[3]{};
    three[0].points = a;
    three[0].point_count = 2;
    three[1].points = b;
    three[1].point_count = 2;
    three[2].points = c;
    three[2].point_count = 2;
    overlume::SceneGraph s3{};
    s3.map_elements = three;
    s3.map_element_count = 3;
    overlume::set_scene(r, s3);
    const std::vector<uint8_t> withThree = render_once(r, pose);

    overlume::SceneGraph s1{};
    s1.map_elements = three;
    s1.map_element_count = 1;
    overlume::set_scene(r, s1);
    const std::vector<uint8_t> withOne = render_once(r, pose);

    EXPECT_NE(withThree, withOne)
        << "removing 2 of 3 lane elements produced an identical frame -- vanished "
           "elements were not evicted from the diff cache";
    overlume::destroy_renderer(r);
}

namespace {

bool RunMapGolden(const char* theme_name, const char* golden_name, const char* out_name) {
    const std::string geomPath =
        std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/hd_map_local_elements_0.geom";
    overlume::testing::MapGeom g = overlume::testing::load_map_geom(geomPath.c_str());
    auto& elems = g.elements;
    EXPECT_FALSE(elems.empty());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, theme_name};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return true;

    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    for (auto& e : elems) e.last_update_sec = s.sim_time_sec;
    const overlume::Vec3 c = overlume::testing::centroid(elems);
    EXPECT_GT(std::hypot(c.x, c.y), 45.0);
    s.ego = {c, 0.0, 3.0, 1};
    s.map_elements = elems.data();
    s.map_element_count = static_cast<uint32_t>(elems.size());
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{c.x - 8, c.y - 8, 6}, {c.x, c.y, c.z}, 60.0};
    const std::string goldenPath =
        std::string(OVERLUME_TEST_DATA_DIR) + "/tests/goldens/" + golden_name;
    const std::string outPath = std::string(OVERLUME_TMP_DIR "/") + out_name;
    double ssim =
        overlume::testing::render_and_compare(r, pose, goldenPath.c_str(), outPath.c_str());
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
    overlume::destroy_renderer(r);
    return false;
}

}

TEST(MapGolden, LaneNetworkAtEgoOffset_DarkAdas) {
    if (RunMapGolden("dark_adas", "map_ego_offset_dark_adas.png",
                     "map_ego_offset_dark_adas_actual.png")) {
        GTEST_SKIP() << "no GPU/EGL";
    }
}

TEST(MapGolden, LaneNetworkAtEgoOffset_LightClay) {
    if (RunMapGolden("light_clay", "map_ego_offset_light_clay.png",
                     "map_ego_offset_light_clay_actual.png")) {
        GTEST_SKIP() << "no GPU/EGL";
    }
}

TEST(MapGolden, CenterlineDotsOnState_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -10, 8}, {0, 0, 0}, 60.0};

    auto* baseR = overlume::create_renderer(cfg);
    if (!baseR) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph empty{};
    overlume::set_scene(baseR, empty);
    const std::vector<uint8_t> baseline = render_once(baseR, pose);
    overlume::destroy_renderer(baseR);

    auto* r = overlume::create_renderer(cfg);
    ASSERT_TRUE(r);
    const overlume::Vec3 line_a[] = {{-6, -6, 0}, {-6, 6, 0}};
    const overlume::Vec3 line_b[] = {{0, -6, 0}, {0, 0, 0}, {2, 6, 0}};
    const overlume::Vec3 line_c[] = {{6, -6, 0}, {6, 6, 0}};
    overlume::MapElement elems[3]{};
    elems[0].points = line_a;
    elems[0].point_count = 2;
    elems[0].kind = overlume::MapKind::CENTERLINE;
    elems[1].points = line_b;
    elems[1].point_count = 3;
    elems[1].kind = overlume::MapKind::CENTERLINE;
    elems[2].points = line_c;
    elems[2].point_count = 2;
    elems[2].kind = overlume::MapKind::CENTERLINE;
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.map_elements = elems;
    s.map_element_count = 3;
    overlume::set_scene(r, s);
    const std::vector<uint8_t> withDots = render_once(r, pose);

    const double dotSsim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/centerline_dots_dark_adas.png",
        OVERLUME_TMP_DIR "/centerline_dots_dark_adas_actual.png");
    EXPECT_GT(dotSsim, overlume::testing::kSsimMin);
    overlume::destroy_renderer(r);

    ASSERT_EQ(baseline.size(), withDots.size());
    size_t differing = 0;
    for (size_t i = 0; i < baseline.size(); ++i) {
        if (baseline[i] != withDots[i]) ++differing;
    }
    EXPECT_GT(differing, 0u) << "CENTERLINE dot-disc elements produced no visible pixel "
                                "difference from a scene with no map data at all";
}

TEST(MapGolden, JunctionCleanupOnState_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -14, 12}, {0, 0, 0}, 60.0};

    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 a_north_w[] = {{-10, 2, 0}, {-3, 2, 0}};
    const overlume::Vec3 a_north_e[] = {{3, 2, 0}, {10, 2, 0}};
    const overlume::Vec3 a_south_w[] = {{-10, -2, 0}, {-3, -2, 0}};
    const overlume::Vec3 a_south_e[] = {{3, -2, 0}, {10, -2, 0}};
    const overlume::Vec3 b_east_s[] = {{2, -10, 0}, {2, -3, 0}};
    const overlume::Vec3 b_east_n[] = {{2, 3, 0}, {2, 10, 0}};
    const overlume::Vec3 b_west_s[] = {{-2, -10, 0}, {-2, -3, 0}};
    const overlume::Vec3 b_west_n[] = {{-2, 3, 0}, {-2, 10, 0}};
    const overlume::Vec3 junction_ring[] = {
        {-3, -3, 0}, {3, -3, 0}, {3, 3, 0}, {-3, 3, 0}, {-3, -3, 0}};
    const overlume::Vec3 sep_v[] = {{0, -10, 0}, {0, 10, 0}};
    const overlume::Vec3 sep_h[] = {{-10, 0, 0}, {10, 0, 0}};

    overlume::MapElement elems[10]{};
    elems[0].points = a_north_w;
    elems[0].point_count = 2;
    elems[0].kind = overlume::MapKind::ROAD_EDGE;
    elems[1].points = a_north_e;
    elems[1].point_count = 2;
    elems[1].kind = overlume::MapKind::ROAD_EDGE;
    elems[2].points = a_south_w;
    elems[2].point_count = 2;
    elems[2].kind = overlume::MapKind::ROAD_EDGE;
    elems[3].points = a_south_e;
    elems[3].point_count = 2;
    elems[3].kind = overlume::MapKind::ROAD_EDGE;
    elems[4].points = b_east_s;
    elems[4].point_count = 2;
    elems[4].kind = overlume::MapKind::ROAD_EDGE;
    elems[5].points = b_east_n;
    elems[5].point_count = 2;
    elems[5].kind = overlume::MapKind::ROAD_EDGE;
    elems[6].points = b_west_s;
    elems[6].point_count = 2;
    elems[6].kind = overlume::MapKind::ROAD_EDGE;
    elems[7].points = b_west_n;
    elems[7].point_count = 2;
    elems[7].kind = overlume::MapKind::ROAD_EDGE;
    elems[8].points = sep_v;
    elems[8].point_count = 2;
    elems[8].kind = overlume::MapKind::LEFT_BOUNDARY;
    elems[9].points = sep_h;
    elems[9].point_count = 2;
    elems[9].kind = overlume::MapKind::RIGHT_BOUNDARY;
    overlume::MapElement junction_elem{};
    junction_elem.points = junction_ring;
    junction_elem.point_count = 5;
    junction_elem.kind = overlume::MapKind::JUNCTION;

    std::vector<overlume::MapElement> all(elems, elems + 10);
    all.push_back(junction_elem);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.map_elements = all.data();
    s.map_element_count = static_cast<uint32_t>(all.size());
    overlume::set_scene(r, s);

    const double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/junction_cleanup_dark_adas.png",
        OVERLUME_TMP_DIR "/junction_cleanup_dark_adas_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
    overlume::destroy_renderer(r);
}

TEST(MapElements, DuplicateElementsDoNotLeakMeshesOrRebuildEveryFrame) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement dup[2]{};
    for (auto& e : dup) {
        e.points = pts;
        e.point_count = 2;
        e.kind = overlume::MapKind::ROAD_EDGE;
        e.last_update_sec = 10.0;
    }
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.map_elements = dup;
    s.map_element_count = 2;
    const overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};

    overlume::set_scene(r, s);
    render_once(r, pose);
    EXPECT_EQ(overlume::testing::map_element_mesh_count(r), 1u)
        << "two identical elements must share one cached mesh";
    const uint64_t rebuildsAfterFirstFrame = overlume::testing::map_element_rebuild_count(r);

    for (int i = 0; i < 5; ++i) {
        overlume::set_scene(r, s);
        render_once(r, pose);
    }
    EXPECT_EQ(overlume::testing::map_element_mesh_count(r), 1u);
    EXPECT_EQ(overlume::testing::map_element_rebuild_count(r), rebuildsAfterFirstFrame)
        << "an unchanged duplicate-bearing scene must not rebuild (and leak) every frame";
    overlume::destroy_renderer(r);
}

TEST(MapElements, FadesViaSharedStalenessAlpha) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    e.last_update_sec = 10.0 - 0.75;
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    const auto info = overlume::testing::map_element_material_info(r);
    EXPECT_TRUE(info.bound_to_translucent)
        << "a stale map element's renderable must be bound to clay_translucent.mat, not "
           "its opaque per-kind template";
    EXPECT_NEAR(info.alpha, 0.5f, 0.02f);

    overlume::destroy_renderer(r);
}

TEST(MapElements, FreshMapElementStaysOnTheOpaqueTemplate) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    e.last_update_sec = 10.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    const auto info = overlume::testing::map_element_material_info(r);
    EXPECT_FALSE(info.bound_to_translucent)
        << "a FRESH map element must stay on the opaque shared template, not get a "
           "per-entity instance";
    EXPECT_NEAR(info.alpha, 1.0f, 1e-4);

    overlume::destroy_renderer(r);
}

TEST(MapElements, EgoInvalidFadesMapElementsRatherThanLeavingThemAtFullOpacity) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    e.last_update_sec = 10.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};

    overlume::set_scene(r, s);
    render_once(r, pose);
    EXPECT_NEAR(overlume::testing::map_element_material_info(r).alpha, 1.0f, 1e-4)
        << "sanity check: ego valid + fresh element -> full opacity, before the flip below";

    s.ego.valid = 0;
    overlume::set_scene(r, s);
    render_once(r, pose);
    const auto afterEgoInvalid = overlume::testing::map_element_material_info(r);
    EXPECT_LT(afterEgoInvalid.alpha, 1.0f)
        << "ego.valid==0 must fade map elements toward invisible, not hold them at full "
           "opacity while the ground/grid patch has already snapped to the origin";
    EXPECT_NEAR(afterEgoInvalid.alpha, 0.0f, 1e-4);

    overlume::destroy_renderer(r);
}

namespace {

std::vector<uint8_t> RenderZFightScene(const overlume::CameraPose& pose,
                                       overlume::MapElement* elems, uint32_t count) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return {};
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.map_elements = elems;
    s.map_element_count = count;
    overlume::set_scene(r, s);
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    const bool ok = overlume::render_frame(r, pose, view);
    overlume::destroy_renderer(r);
    if (!ok) return {};
    return pixels;
}

bool PixelDiffers(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, size_t px) {
    for (int c = 0; c < 3; ++c) {
        int d = static_cast<int>(a[px * 3 + c]) - static_cast<int>(b[px * 3 + c]);
        if (d < 0) d = -d;
        if (d > 8) return true;
    }
    return false;
}

}

TEST(MapElementsZFight, CrosswalkOverBoundaryStaysStableAcrossTinyCameraMove) {
    overlume::Vec3 crosswalk_ring[] = {{-3, -1, 0}, {3, -1, 0}, {3, 1, 0}, {-3, 1, 0}, {-3, -1, 0}};
    overlume::Vec3 boundary_line[] = {{-5, 0, 0}, {5, 0, 0}};

    overlume::MapElement both[2]{};
    both[0].points = crosswalk_ring;
    both[0].point_count = 5;
    both[0].is_polygon = 1;
    both[0].kind = overlume::MapKind::CROSSWALK;
    both[1].points = boundary_line;
    both[1].point_count = 2;
    both[1].kind = overlume::MapKind::LEFT_BOUNDARY;

    overlume::MapElement crosswalk_only[1]{both[0]};
    overlume::MapElement boundary_only[1]{both[1]};

    const overlume::CameraPose poseA{{0, -10, 6}, {0, 0, 0}, 60.0};
    const overlume::CameraPose poseB{{0.004, -10, 6}, {0, 0, 0}, 60.0};

    const std::vector<uint8_t> background = RenderZFightScene(poseA, nullptr, 0);
    if (background.empty()) GTEST_SKIP() << "no GPU/EGL";
    const std::vector<uint8_t> cwOnly = RenderZFightScene(poseA, crosswalk_only, 1);
    const std::vector<uint8_t> lineOnly = RenderZFightScene(poseA, boundary_only, 1);
    ASSERT_EQ(background.size(), cwOnly.size());
    ASSERT_EQ(background.size(), lineOnly.size());

    const size_t numPixels = background.size() / 3;
    std::vector<bool> overlapMask(numPixels, false);
    size_t overlapCount = 0;
    for (size_t px = 0; px < numPixels; ++px) {
        if (PixelDiffers(cwOnly, background, px) && PixelDiffers(lineOnly, background, px)) {
            overlapMask[px] = true;
            ++overlapCount;
        }
    }
    ASSERT_GE(overlapCount, 20u)
        << "the boundary/crosswalk fixture produced too small a screen-space "
           "overlap to measure a flip rate -- test geometry/camera needs "
           "adjusting";

    const std::vector<uint8_t> combinedA = RenderZFightScene(poseA, both, 2);
    const std::vector<uint8_t> combinedB = RenderZFightScene(poseB, both, 2);
    ASSERT_EQ(combinedA.size(), combinedB.size());

    size_t flipped = 0;
    for (size_t px = 0; px < numPixels; ++px) {
        if (overlapMask[px] && PixelDiffers(combinedA, combinedB, px)) ++flipped;
    }
    EXPECT_LT(static_cast<double>(flipped) / static_cast<double>(overlapCount), 0.10)
        << flipped << " of " << overlapCount
        << " overlap pixels changed between two camera positions 4mm apart -- "
           "z-fighting flip between the crosswalk and the boundary line, not a "
           "camera-induced content change";
}
