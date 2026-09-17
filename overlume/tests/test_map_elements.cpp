// test_map_elements.cpp — ego-following ground/grid, lane MaterialInstance
// theming, and HD-map lane/crosswalk rendering. Same "no Filament type"
// boundary as every other tests/*.cpp — see map_elements_test_hooks.hpp /
// ego_test_hooks.hpp.
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

}  // namespace

// ── ego-following ground/grid patch ──────────────────────────────────────

TEST(Ground, FollowsEgoQuantizedToGridPitch) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    s.ego = {{120.4, -80.6, 0.0}, 0.0, 0.0, /*valid=*/1};
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{120, -88, 4}, {120, -80, 0}, 60.0};
    render_once(r, pose);
    auto c = overlume::testing::ground_patch_centre(r);
    // Quantized to the real grid pitch (2 m, renderer_internal.hpp's
    // kGridPitchM, the same symbol build_grid_lines() draws lines at):
    // round(120.4/2)*2 = 120; round(-80.6/2)*2 = -80. Snapping to 1 m
    // instead would shift the 2 m lines by half a cell every time the ego
    // crosses an odd metre.
    EXPECT_NEAR(c.x, 120.0, 1e-6);
    EXPECT_NEAR(c.y, -80.0, 1e-6);
    overlume::destroy_renderer(r);
}

TEST(Ground, PatchSnapsAWholeCellAtATime) {
    // ego (121.4, -80.6) -> (122, -80): a full pitch of movement in X, none
    // in Y. A test that only ever checks one position can't tell 1 m from
    // 2 m snapping; this one can.
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    s.ego = {{121.4, -80.6, 0.0}, 0.0, 0.0, /*valid=*/1};
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{121, -88, 4}, {121, -80, 0}, 60.0};
    render_once(r, pose);
    auto c = overlume::testing::ground_patch_centre(r);
    EXPECT_NEAR(c.x, 122.0, 1e-6);
    EXPECT_NEAR(c.y, -80.0, 1e-6);
    overlume::destroy_renderer(r);
}

TEST(Ground, NoEgoYet_StaysAtOrigin) {
    // ego.valid == 0 -> patch centre (0,0): identical to the original
    // static placement, so every pre-existing golden stays valid.
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    s.ego = {{500.0, 500.0, 0.0}, 0.0, 0.0, /*valid=*/0};
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    auto c = overlume::testing::ground_patch_centre(r);
    EXPECT_NEAR(c.x, 0.0, 1e-6);
    EXPECT_NEAR(c.y, 0.0, 1e-6);
    overlume::destroy_renderer(r);
}

TEST(Ground, EpicOneEmptyWorldGoldenStillMatches) {
    // ThemeGolden.EmptyWorld_* (test_theme.cpp) renders with ego.valid == 0,
    // the branch that must reproduce the original static placement
    // byte-for-byte.
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph s{};
    overlume::set_scene(r, s);  // default SceneGraph{} -> ego.valid == 0
    overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png",
        "/tmp/map_elements_empty_world_regression_actual.png");
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
}

// ── lane material is themed on first data, no set_theme() needed ────────

TEST(MapElements, LaneMaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // First-ever map data. Nothing calls set_theme().
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

// ── per-kind material dispatch ───────────────────────────────────────────

TEST(MapElements, KindDrivesMaterialDispatchToTheMatchingThemeToken) {
    // Exercises every kind material_for_kind() (map_elements.cpp) dispatches
    // on, reading values off the loaded dark_adas theme itself (not
    // hardcoded literals), so a re-authored palette doesn't stale this test.
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
    // ROAD_EDGE has its own dedicated token.
    expect_kind_color(overlume::MapKind::ROAD_EDGE, theme->palette.road_edge);
    // STOPLINE has no dedicated token (deliberate YAGNI) -- falls back to
    // palette.lane_paint, same as every other undedicated kind.
    expect_kind_color(overlume::MapKind::STOPLINE, theme->palette.lane_paint);

    overlume::destroy_renderer(r);
}

// ── crosswalk-hatch dedupe ───────────────────────────────────────────────

TEST(MapElements, CrosswalkHatchFiresOnRecordedFivePointClosedPolyline) {
    // The exact 5 points from test/fixtures/hd_map_local_elements_0.yaml,
    // ns: crosswalk_8043 -- point[0] == point[4] (closing vertex), a real
    // recorded shape, not an invented 4-point quad. The dedupe to 4 points
    // is adapter-side (hd_map.cpp); this pins the library half: the
    // geometry, once deduped, hatches correctly on a real (non-axis-aligned)
    // quad, not just a synthetic one.
    const overlume::Vec3 pts_after_dedupe[4] = {
        {-39.50850289011474, 45.33743457749722, -0.000284586101770401},
        {-54.35884356129442, 45.2288915511252, -0.00039308611303567886},
        {-54.476791014440394, 47.18497371095612, -0.0004083588719367981},
        {-39.52170872121907, 47.27801090662989, -0.0002988511696457863},
    };
    auto tris = overlume::detail::build_crosswalk_hatch(pts_after_dedupe, 4, 0.02f);
    EXPECT_FALSE(tris.empty());
}

// ── crosswalk orientation: rails = the quad's LONG-edge pair, stripe count
//    is pitch-derived ───────────────────────────────────────────────────

TEST(MapElements, CrosswalkHatchBarsAreOrientedAlongTheShortAxis) {
    // Same real crosswalk_8043 fixture geometry as the test above -- edges
    // 0-1/2-3 are the LONG pair (~14.9m, the crossing WIDTH) and edges
    // 1-2/3-0 are the SHORT pair (~1.95m, the travel-direction DEPTH),
    // verified by direct computation on these exact points. A real zebra
    // stripe's long axis runs along the SHORT (travel) axis.
    const overlume::Vec3 pts[4] = {
        {-39.50850289011474, 45.33743457749722, -0.000284586101770401},
        {-54.35884356129442, 45.2288915511252, -0.00039308611303567886},
        {-54.476791014440394, 47.18497371095612, -0.0004083588719367981},
        {-39.52170872121907, 47.27801090662989, -0.0002988511696457863},
    };
    auto tris = overlume::detail::build_crosswalk_hatch(pts, 4, 0.0f);
    ASSERT_FALSE(tris.empty());
    ASSERT_EQ(tris.size() % 6, 0u) << "not a whole number of 2-triangle stripe quads";

    // First stripe quad is tris[0..5]: (a0, a1, b1, a0, b1, b0). The bar's
    // actual long axis is a0->b0, spanning between the two rails -- it must
    // be close to parallel with one of the quad's SHORT edges (1-2 or 3-0),
    // not the LONG edges (0-1/2-3).
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
    // Same fixture; the long axis (edges 0-1/2-3) averages ~14.90m.
    // clamp(round(14.90 / 1.2), 3, 24) == 12, computed independently of the
    // production formula.
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
    // A tiny (~1m long-axis) and a huge (~200m long-axis) quad both clamp
    // into [3, 24] rather than rounding to an absurd 1 or 166.
    const overlume::Vec3 tiny[4] = {{0, 0, 0}, {1, 0, 0}, {1, 2, 0}, {0, 2, 0}};
    auto trisTiny = overlume::detail::build_crosswalk_hatch(tiny, 4, 0.0f);
    ASSERT_FALSE(trisTiny.empty());
    EXPECT_EQ(trisTiny.size() / 6, 3u);

    const overlume::Vec3 huge[4] = {{0, 0, 0}, {200, 0, 0}, {200, 4, 0}, {0, 4, 0}};
    auto trisHuge = overlume::detail::build_crosswalk_hatch(huge, 4, 0.0f);
    ASSERT_FALSE(trisHuge.empty());
    EXPECT_EQ(trisHuge.size() / 6, 24u);
}

// ── dash-kind flip: boundary dashes, centerline stays one solid mesh ────

TEST(MapElementsGolden, DashedBoundaryProducesSameDashRunsAsThePreMoveAlgorithm) {
    // A 10 m straight polyline on a kind==LEFT_BOUNDARY element, dashed
    // librarywise into 4 mesh chunks at kDashLenM=1.5/kGapLenM=1.5
    // ([0,1.5],[3,4.5],[6,7.5],[9,10]) -- checked via the mesh-count hook,
    // not a full-frame SSIM (which would also carry road-fill/per-kind
    // color, a separate concern).
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
    // The flip's other half: the same geometry on kind==CENTERLINE stays
    // one mesh chunk -- never dash-split.
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

// ── ROAD_EDGE is solid (never dashed), CENTERLINE renders as dot discs ──

TEST(MapElementsGolden, RoadEdgeOfSameGeometryProducesOneMeshChunkNeverDashed) {
    // The dash-flip's third case: ROAD_EDGE is NOT a BOUNDARY kind
    // (IsBoundaryKind() only matches LEFT_BOUNDARY/RIGHT_BOUNDARY), so the
    // same 10 m geometry that dashes into 4 chunks under LEFT_BOUNDARY
    // stays ONE solid mesh chunk under ROAD_EDGE.
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
    // build_centerline_dots() (map_elements.cpp) replaces the solid ribbon
    // strip for CENTERLINE with arc-length-spaced filled discs. Proven via
    // vertex count, not a full-frame SSIM: dots at kCenterlineDotSpacingM=
    // 2.0m over a 10m line land at s=0,2,4,6,8,10 (6 dots, endpoint-
    // inclusive), each a kCenterlineDotSegments=10-wedge fan = 30
    // vertices/dot -> 180 total (a strip of the same geometry would be 6).
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

// ── road-surface fill ────────────────────────────────────────────────────

TEST(MapElements, RoadSurfaceKindTriangulatesTheTwoRailEncodingIntoAStrip) {
    // 16 stations per rail (kRoadFillSamples), point_count == 32 -- assert
    // the built mesh renders as something distinguishable from an empty
    // scene (a Filament-free triangle-count hook would need a new export;
    // a pixel-difference check against a no-map-data baseline is the same
    // proof this file's other synthetic map tests use).
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
        pts[i] = {-1.0, y, 0.0};       // left rail
        pts[kN + i] = {1.0, y, 0.0};   // right rail
    }
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2 * kN;
    e.kind = overlume::MapKind::ROAD_SURFACE;
    overlume::SceneGraph s{};
    // Valid ego: this test proves the strip renders, not the ego-invalid
    // fade path (see EgoInvalidFadesMapElementsRatherThanLeavingThemAtFullOpacity).
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
    // An odd point_count can't split evenly into two rails -- silently
    // dropped (spec §9's "missing data renders nothing, not an error"),
    // never a crash or an out-of-bounds read.
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

// ── map_element_rebuild_count hook ───────────────────────────────────────

TEST(MapElements, RebuildCountStaysZeroOnUnchangedContentSignature) {
    // Publishing the IDENTICAL SceneGraph twice must not rebuild the second
    // time (a cache-hit on the unchanged content signature).
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

    overlume::set_scene(r, s);  // identical content signature
    render_once(r, pose);
    EXPECT_EQ(overlume::testing::map_element_rebuild_count(r), afterFirst)
        << "publishing the identical MapElement a second time triggered a rebuild -- "
           "the content-signature cache isn't actually a cache";
    overlume::destroy_renderer(r);
}

// ── map elements actually render (synthetic scenes) ──────────────────────

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
    // A lane centerline running toward the camera...
    const overlume::Vec3 lanePts[] = {{0, -6, 0}, {0, -2, 0}, {0, 2, 0}, {0, 6, 0}};
    // ...and a crosswalk quad straddling it (exercises the polygon + hatch
    // path, not just the polyline path).
    const overlume::Vec3 crosswalkPts[] = {{-1.5, -0.5, 0}, {1.5, -0.5, 0}, {1.5, 0.5, 0}, {-1.5, 0.5, 0}};
    overlume::MapElement elems[2]{};
    elems[0].points = lanePts;
    elems[0].point_count = 4;
    elems[0].is_polygon = 0;
    elems[1].points = crosswalkPts;
    elems[1].point_count = 4;
    elems[1].is_polygon = 1;
    overlume::SceneGraph s{};
    // update_map_elements() gates on ego.valid (fades to 0 while invalid);
    // this test isn't exercising that path, so it needs a valid ego.
    // last_update_sec/sim_time_sec both default to 0.0 (fresh).
    s.ego.valid = 1;
    s.map_elements = elems;
    s.map_element_count = 2;
    overlume::set_scene(r, s);
    const std::vector<uint8_t> withMap = render_once(r, pose);
    // Not a golden (no committed comparison target) -- just a viewable PNG
    // of the lane+crosswalk render path for human sanity-checking.
    overlume::testing::render_and_compare(r, pose, "/nonexistent-golden.png",
                                        "/tmp/map_elements_synthetic_actual.png");
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
    // Diff-cache add/remove sanity: publishing fewer elements than the
    // previous frame must not leave stale geometry rendered forever (a
    // rebuild-once-and-never-again bug would keep showing all 3).
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
    s1.map_elements = three;  // only the first element now
    s1.map_element_count = 1;
    overlume::set_scene(r, s1);
    const std::vector<uint8_t> withOne = render_once(r, pose);

    EXPECT_NE(withThree, withOne)
        << "removing 2 of 3 lane elements produced an identical frame -- vanished "
           "elements were not evicted from the diff cache";
    overlume::destroy_renderer(r);
}

// ── golden: recorded HD-map fixture, both themes ─────────────────────────

namespace {

// `hd_map_local_elements_0.geom` is committed (emitted by the node-side
// HdMapAdapter's own gtest, from the real, filtered
// hd_map_local_elements_0.yaml) -- 74 elements (58 lane/crosswalk markers +
// 16 synthesized ROAD_SURFACE elements). Returns true only for the one
// legitimate runtime skip left (no GPU/EGL), matching every other renderer
// test's convention.
bool RunMapGolden(const char* theme_name, const char* golden_name, const char* out_name) {
    const std::string geomPath =
        std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/hd_map_local_elements_0.geom";
    overlume::testing::MapGeom g = overlume::testing::load_map_geom(geomPath.c_str());
    auto& elems = g.elements;
    EXPECT_FALSE(elems.empty());

    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, theme_name};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return true;  // GTEST_SKIP path, no GPU/EGL

    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    // The `.geom` text-dump format (golden.hpp) carries no last_update_sec,
    // so every loaded element defaults to 0.0 -- against sim_time_sec=10.0
    // that reads as maximally stale. Stamp every element "just refreshed"
    // instead, the same way a live HdMapAdapter would on its next fill().
    for (auto& e : elems) e.last_update_sec = s.sim_time_sec;
    const overlume::Vec3 c = overlume::testing::centroid(elems);
    // Pins that the fixture is far from the map origin. Measured centroid
    // (-46.73, 12.82), hypot ~48.46 -- comfortably outside the 40x40m
    // origin-centred void patch (kGroundHalfExtent=20m); 45.0 is the
    // measured threshold for this dataset.
    EXPECT_GT(std::hypot(c.x, c.y), 45.0);
    s.ego = {c, 0.0, 3.0, 1};
    s.map_elements = elems.data();
    s.map_element_count = static_cast<uint32_t>(elems.size());
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{c.x - 8, c.y - 8, 6}, {c.x, c.y, c.z}, 60.0};
    const std::string goldenPath = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/goldens/" + golden_name;
    const std::string outPath = std::string("/tmp/") + out_name;
    double ssim = overlume::testing::render_and_compare(r, pose, goldenPath.c_str(), outPath.c_str());
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
    return false;
}

}  // namespace

TEST(MapGolden, LaneNetworkAtEgoOffset_DarkAdas) {
    if (RunMapGolden("dark_adas", "map_ego_offset_dark_adas.png", "map_ego_offset_dark_adas_actual.png")) {
        GTEST_SKIP() << "no GPU/EGL";
    }
}

TEST(MapGolden, LaneNetworkAtEgoOffset_LightClay) {
    if (RunMapGolden("light_clay", "map_ego_offset_light_clay.png",
                      "map_ego_offset_light_clay_actual.png")) {
        GTEST_SKIP() << "no GPU/EGL";
    }
}

// ── centerline-ON golden (dot guidance) ──────────────────────────────────
// Profiles gate whether CENTERLINE elements ever reach the library (hidden
// by default, per urban_profile.yaml/sim_profile.yaml) -- but the library
// itself renders them unconditionally whenever present in the SceneGraph.
// Synthetic scene (the committed hd_map_local_elements_0.geom carries no
// CENTERLINE elements by design), feeding them directly to prove the
// dot-disc path renders as something distinct and legible.
TEST(MapGolden, CenterlineDotsOnState_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -10, 8}, {0, 0, 0}, 60.0};

    auto* baseR = overlume::create_renderer(cfg);
    if (!baseR) GTEST_SKIP() << "no GPU/EGL";
    overlume::SceneGraph empty{};
    overlume::set_scene(baseR, empty);
    const std::vector<uint8_t> baseline = render_once(baseR, pose);
    overlume::destroy_renderer(baseR);

    auto* r = overlume::create_renderer(cfg);
    ASSERT_TRUE(r);
    // A handful of straight/curved centerlines across the frame -- enough
    // to show dot spacing/radius at a glance.
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

    // SSIM is asserted, not just returned: an unchecked render_and_compare
    // is a candidate generator that can never go red. The pixel-diff
    // against the empty scene below stays as the mechanism-level check.
    const double dotSsim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/centerline_dots_dark_adas.png",
        "/tmp/centerline_dots_dark_adas_actual.png");
    EXPECT_GT(dotSsim, 0.98);
    overlume::destroy_renderer(r);

    ASSERT_EQ(baseline.size(), withDots.size());
    size_t differing = 0;
    for (size_t i = 0; i < baseline.size(); ++i) {
        if (baseline[i] != withDots[i]) ++differing;
    }
    EXPECT_GT(differing, 0u) << "CENTERLINE dot-disc elements produced no visible pixel "
                                 "difference from a scene with no map data at all";
}

// ── junction-cleanup golden ───────────────────────────────────────────────
// The cut itself (clip against a JUNCTION polygon / mutual-crossing
// back-off) is entirely node-side (HdMapAdapter::fill(), adapter-level
// tests) -- the library only ever renders whatever MapElements it is
// handed, so this golden feeds a synthetic scene shaped like the adapter's
// own post-cut output: two crossing roads' ROAD_EDGE outer edges, each
// already split at the junction box, a MapKind::JUNCTION ring (the box
// itself, generic/OTHER styling, no dedicated token), and two interior
// LEFT_/RIGHT_BOUNDARY dashed separators left uncut, running straight
// through. Proves the rendered result of a cut adapter output reads clean,
// independent of the cut algorithm itself (proven at the adapter level,
// hd_map.cpp).
TEST(MapGolden, JunctionCleanupOnState_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -14, 12}, {0, 0, 0}, 60.0};

    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Road A (east-west) outer edges, each already cut at the box
    // (x in [-3,3]) -- two pieces per rail, four ROAD_EDGE elements.
    const overlume::Vec3 a_north_w[] = {{-10, 2, 0}, {-3, 2, 0}};
    const overlume::Vec3 a_north_e[] = {{3, 2, 0}, {10, 2, 0}};
    const overlume::Vec3 a_south_w[] = {{-10, -2, 0}, {-3, -2, 0}};
    const overlume::Vec3 a_south_e[] = {{3, -2, 0}, {10, -2, 0}};
    // Road B (north-south) outer edges, same shape, cut at y in [-3,3].
    const overlume::Vec3 b_east_s[] = {{2, -10, 0}, {2, -3, 0}};
    const overlume::Vec3 b_east_n[] = {{2, 3, 0}, {2, 10, 0}};
    const overlume::Vec3 b_west_s[] = {{-2, -10, 0}, {-2, -3, 0}};
    const overlume::Vec3 b_west_n[] = {{-2, 3, 0}, {-2, 10, 0}};
    // The junction box itself (closed ring, matches a real recorded
    // JUNCTION marker's own 5-point closed-rectangle shape).
    const overlume::Vec3 junction_ring[] = {
        {-3, -3, 0}, {3, -3, 0}, {3, 3, 0}, {-3, 3, 0}, {-3, -3, 0}};
    // Interior separators: NOT cut -- run straight through the box.
    const overlume::Vec3 sep_v[] = {{0, -10, 0}, {0, 10, 0}};
    const overlume::Vec3 sep_h[] = {{-10, 0, 0}, {10, 0, 0}};

    overlume::MapElement elems[10]{};
    elems[0].points = a_north_w; elems[0].point_count = 2; elems[0].kind = overlume::MapKind::ROAD_EDGE;
    elems[1].points = a_north_e; elems[1].point_count = 2; elems[1].kind = overlume::MapKind::ROAD_EDGE;
    elems[2].points = a_south_w; elems[2].point_count = 2; elems[2].kind = overlume::MapKind::ROAD_EDGE;
    elems[3].points = a_south_e; elems[3].point_count = 2; elems[3].kind = overlume::MapKind::ROAD_EDGE;
    elems[4].points = b_east_s;  elems[4].point_count = 2; elems[4].kind = overlume::MapKind::ROAD_EDGE;
    elems[5].points = b_east_n;  elems[5].point_count = 2; elems[5].kind = overlume::MapKind::ROAD_EDGE;
    elems[6].points = b_west_s;  elems[6].point_count = 2; elems[6].kind = overlume::MapKind::ROAD_EDGE;
    elems[7].points = b_west_n;  elems[7].point_count = 2; elems[7].kind = overlume::MapKind::ROAD_EDGE;
    elems[8].points = sep_v;     elems[8].point_count = 2; elems[8].kind = overlume::MapKind::LEFT_BOUNDARY;
    elems[9].points = sep_h;     elems[9].point_count = 2; elems[9].kind = overlume::MapKind::RIGHT_BOUNDARY;
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

    // SSIM asserted: an unchecked render_and_compare can never go red.
    const double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/junction_cleanup_dark_adas.png",
        "/tmp/junction_cleanup_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
}

// ── duplicate-signature leak regression (found live, 2026-09-09) ─────────

TEST(MapElements, DuplicateElementsDoNotLeakMeshesOrRebuildEveryFrame) {
    // Real feeds carry byte-identical map elements (adjacent lanes share a
    // physical rail; local+global map topics overlap): both hash to ONE
    // chunk signature. Before the `next.count(key)` guard in
    // adopt_or_build, the second occurrence rebuilt a mesh, add_mesh()
    // put its renderable in the scene, and the failed emplace dropped the
    // only handle to it -- one leaked scene renderable PER FRAME, measured
    // live as render_ms climbing 13 -> ~140 ms over a minute of playback.
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement dup[2]{};
    for (auto& e : dup) {
        e.points = pts;
        e.point_count = 2;
        e.kind = overlume::MapKind::ROAD_EDGE;  // solid polyline path, no dash fan-out
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

// ── staleness fade, the one shared path ──────────────────────────────────

TEST(MapElements, FadesViaSharedStalenessAlpha) {
    // Same shape as test_objects.cpp's StaleObjectFadesViaSharedStalenessAlpha:
    // publish ONE MapElement with last_update_sec in the past relative to
    // sim_time_sec, render, assert via the test hook that the bound
    // clay_translucent instance's alpha matches staleness_alpha()'s own
    // computed value, not a pixel comparison. ego.valid=1 so the
    // ego-invalid gate (tested separately below) isn't what's driving this
    // alpha down.
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    e.last_update_sec = 10.0 - 0.75;  // 0.75s behind -> alpha ~0.5, same worked
                                       // example test_objects.cpp's own fade test uses
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
    // The other half of the fade -- fresh (last_update_sec == sim_time_sec)
    // must stay on the shared opaque per-kind template, no per-entity
    // instance at all (test_objects.cpp's own "fresh" half, same shape).
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

// ── the ego-invalid map cosmetic ─────────────────────────────────────────

TEST(MapElements, EgoInvalidFadesMapElementsRatherThanLeavingThemAtFullOpacity) {
    // update_ground_grid_transform() snaps the ego-following ground/grid
    // patch to the world origin whenever ego.valid==0; update_map_elements()
    // must match: ego.valid==0 drives alpha to 0 via the same fade path
    // (not skip-and-freeze, which would leave the last valid frame's
    // geometry at full opacity forever). See renderer.cpp.
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = overlume::MapKind::CENTERLINE;
    e.last_update_sec = 10.0;  // fresh by staleness_alpha's own math
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

    s.ego.valid = 0;  // TF dropout; the SAME MapElement, still "fresh" by staleness_alpha
    overlume::set_scene(r, s);
    render_once(r, pose);
    const auto afterEgoInvalid = overlume::testing::map_element_material_info(r);
    EXPECT_LT(afterEgoInvalid.alpha, 1.0f)
        << "ego.valid==0 must fade map elements toward invisible, not hold them at full "
           "opacity while the ground/grid patch has already snapped to the origin";
    EXPECT_NEAR(afterEgoInvalid.alpha, 0.0f, 1e-4);

    overlume::destroy_renderer(r);
}

// ── crosswalk/boundary z-fight regression (flicker report, 2026-09-16) ───
// Root cause: before the per-kind z-lift table, every non-ROAD_SURFACE
// MapKind shared one z (the old kLaneZLiftM) -- a CROSSWALK polygon and a
// boundary stripe crossing it were exactly coplanar, and the depth buffer
// had no basis to order two coplanar triangles. The winning surface flips
// per-pixel as the camera moves, even by millimetres -- classic
// z-fighting, seen by the user as the thin LANE LINE flickering under the
// crosswalk (the small-area loser).
//
// This finds the overlap region itself (pixels where a crosswalk-only
// render AND a boundary-only render both differ from bare ground), then
// renders the COMBINED scene from two camera positions 4mm apart and
// asserts that region is STABLE between them. A coplanar pair flips there;
// a staggered pair does not. Asserting only that the z-lift constants
// differ would pass even if the renderer ignored them entirely -- this
// checks actual rendered pixels instead.
namespace {

std::vector<uint8_t> RenderZFightScene(const overlume::CameraPose& pose, overlume::MapElement* elems,
                                        uint32_t count) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return {};  // no GPU/EGL
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

// True if pixel `px` (0-based, RGB-interleaved) differs by more than a
// small tolerance in any channel -- tolerance absorbs incidental
// anti-aliasing noise without absorbing an actual surface-color flip.
bool PixelDiffers(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, size_t px) {
    for (int c = 0; c < 3; ++c) {
        int d = static_cast<int>(a[px * 3 + c]) - static_cast<int>(b[px * 3 + c]);
        if (d < 0) d = -d;
        if (d > 8) return true;
    }
    return false;
}

}  // namespace

TEST(MapElementsZFight, CrosswalkOverBoundaryStaysStableAcrossTinyCameraMove) {
    // Crosswalk: x in [-3,3], y in [-1,1]. A 5-point CLOSED ring, not the
    // recorded-data 4-point quad -- build_crosswalk_hatch() only fires for
    // n==4, so this falls to triangulate_convex_polygon()'s plain solid
    // fill: a reliable opaque overlap area, not a hatch pattern that could
    // dodge the fight by landing in a gap.
    overlume::Vec3 crosswalk_ring[] = {
        {-3, -1, 0}, {3, -1, 0}, {3, 1, 0}, {-3, 1, 0}, {-3, -1, 0}};
    // A lane boundary straight through the crosswalk's middle (y=0) --
    // its kLaneHalfWidthM=0.05m ribbon overlaps the crosswalk fill for the
    // whole x in [-3,3] span.
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
    // Finding #11/#32: 20u, not merely >0u -- the bound below is a
    // percentage; too small a mask makes the ORIGINAL integer-division form
    // (overlapCount / 10) unsatisfiable even at flipped==0, and even in the
    // float form now used a tiny denominator makes one stray flip look like
    // a large regression. 20 keeps real headroom under today's measured 54.
    ASSERT_GE(overlapCount, 20u) << "the boundary/crosswalk fixture produced too small a screen-space "
                                     "overlap to measure a flip rate -- test geometry/camera needs "
                                     "adjusting";

    const std::vector<uint8_t> combinedA = RenderZFightScene(poseA, both, 2);
    const std::vector<uint8_t> combinedB = RenderZFightScene(poseB, both, 2);
    ASSERT_EQ(combinedA.size(), combinedB.size());

    size_t flipped = 0;
    for (size_t px = 0; px < numPixels; ++px) {
        if (overlapMask[px] && PixelDiffers(combinedA, combinedB, px)) ++flipped;
    }
    // A coplanar overlap flips which surface wins across a real chunk of
    // the mask between two camera positions 4mm apart: measured against
    // the pre-fix shared-constant behavior (temporarily reverted while
    // writing this test), 15 of 54 mask pixels (~28%) flip and this bound
    // fails. Measured against the per-kind table above, 0 of 54 flip. 10%
    // sits comfortably between the two and clear of both.
    // Finding #11/#32: float division, not `overlapCount / 10` -- the
    // integer form made this assertion unsatisfiable (bound truncates to 0)
    // whenever overlapCount fell below 10, even at flipped==0. The
    // ASSERT_GE(overlapCount, 20u) precondition above keeps this bound
    // meaningful; this form keeps it correct even if that margin ever
    // shrinks.
    EXPECT_LT(static_cast<double>(flipped) / static_cast<double>(overlapCount), 0.10)
        << flipped << " of " << overlapCount
        << " overlap pixels changed between two camera positions 4mm apart -- "
           "z-fighting flip between the crosswalk and the boundary line, not a "
           "camera-induced content change";
}
