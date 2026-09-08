// test_map_elements.cpp — Epic 2 Task 2 (VM-024): retiring the placeholder
// ground/grid (2a), the lane MaterialInstance's theming (8a), and HD-map
// lane/crosswalk rendering (2d). Same "no Filament type" boundary as every
// other tests/*.cpp — see map_elements_test_hooks.hpp / ego_test_hooks.hpp.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "golden.hpp"
#include "map_elements_test_hooks.hpp"
#include "polyline.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <cmath>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<uint8_t> render_once(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    return pixels;
}

}  // namespace

// ── 2a: the ego void is retired ─────────────────────────────────────────

TEST(Ground, FollowsEgoQuantizedToGridPitch) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph s{};
    s.ego = {{120.4, -80.6, 0.0}, 0.0, 0.0, /*valid=*/1};
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{120, -88, 4}, {120, -80, 0}, 60.0};
    render_once(r, pose);
    auto c = mpviz::testing::ground_patch_centre(r);
    // Quantized to the REAL grid pitch (2 m, renderer_internal.hpp's
    // kGridPitchM -- the same symbol build_grid_lines() draws lines at):
    // round(120.4/2)*2 = 120; round(-80.6/2)*2 = -80. Snapping to 1 m
    // instead would shift the 2 m lines by HALF A CELL every time the ego
    // crosses an odd metre -- exactly the crawl the quantization exists to
    // prevent.
    EXPECT_NEAR(c.x, 120.0, 1e-6);
    EXPECT_NEAR(c.y, -80.0, 1e-6);
    mpviz::destroy_renderer(r);
}

TEST(Ground, PatchSnapsAWholeCellAtATime) {
    // ego (121.4, -80.6) -> (122, -80): a full pitch of movement in X, none
    // in Y. A test that only ever checks one position can't tell 1 m from
    // 2 m snapping; this one can.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph s{};
    s.ego = {{121.4, -80.6, 0.0}, 0.0, 0.0, /*valid=*/1};
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{121, -88, 4}, {121, -80, 0}, 60.0};
    render_once(r, pose);
    auto c = mpviz::testing::ground_patch_centre(r);
    EXPECT_NEAR(c.x, 122.0, 1e-6);
    EXPECT_NEAR(c.y, -80.0, 1e-6);
    mpviz::destroy_renderer(r);
}

TEST(Ground, NoEgoYet_StaysAtOrigin) {
    // ego.valid == 0 -> patch centre (0,0): identical to Epic 1's image, so
    // every Epic 1 golden stays valid byte-for-byte.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph s{};
    s.ego = {{500.0, 500.0, 0.0}, 0.0, 0.0, /*valid=*/0};
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);
    auto c = mpviz::testing::ground_patch_centre(r);
    EXPECT_NEAR(c.x, 0.0, 1e-6);
    EXPECT_NEAR(c.y, 0.0, 1e-6);
    mpviz::destroy_renderer(r);
}

TEST(Ground, EpicOneEmptyWorldGoldenStillMatches) {
    // Confirms the ordering section's claim directly: ThemeGolden.
    // EmptyWorld_* (test_theme.cpp) renders with ego.valid == 0, which is
    // exactly the branch that must reproduce Epic 1's original static
    // placement byte-for-byte.
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph s{};
    mpviz::set_scene(r, s);  // default SceneGraph{} -> ego.valid == 0
    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    double ssim = mpviz::testing::render_and_compare(
        r, pose, MPVIZ_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png",
        "/tmp/map_elements_empty_world_regression_actual.png");
    EXPECT_GT(ssim, 0.98);
    mpviz::destroy_renderer(r);
}

// ── 8a: lane material is themed on first data, no set_theme() needed ───

TEST(MapElements, LaneMaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = mpviz::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // First-ever map data. Nothing calls set_theme().
    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement elem{};
    elem.points = pts;
    elem.point_count = 2;
    elem.is_polygon = 0;
    mpviz::SceneGraph s{};
    s.map_elements = &elem;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
    render_once(r, pose);

    auto p = mpviz::testing::lane_material_base_color(r);
    EXPECT_NEAR(p.r, theme->palette.lane_paint.r, 1e-4);
    EXPECT_NEAR(p.g, theme->palette.lane_paint.g, 1e-4);
    EXPECT_NEAR(p.b, theme->palette.lane_paint.b, 1e-4);
    mpviz::destroy_renderer(r);
}

// ── Epic 3 Task 1 (VM-036) decision #6: per-kind material dispatch ──────

TEST(MapElements, KindDrivesMaterialDispatchToTheMatchingThemeToken) {
    // Same shape as LaneMaterialIsThemedOnFirstDataWithNoTransition just
    // above: first-ever data, no set_theme() call, one render. Exercises
    // every kind material_for_kind() (map_elements.cpp) dispatches on,
    // reading the values off the loaded dark_adas theme itself (not
    // hardcoded literals), so a re-authored palette doesn't stale this
    // test out from under itself.
    const auto theme = mpviz::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement elem{};
    elem.points = pts;
    elem.point_count = 2;
    elem.is_polygon = 0;
    mpviz::SceneGraph s{};
    s.map_elements = &elem;
    s.map_element_count = 1;

    auto expect_kind_color = [&](mpviz::MapKind kind, const mpviz::detail::Float3& expected) {
        elem.kind = kind;
        mpviz::set_scene(r, s);
        mpviz::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};
        render_once(r, pose);
        auto p = mpviz::testing::map_kind_base_color(r, kind);
        EXPECT_NEAR(p.r, expected.r, 1e-4) << "kind=" << static_cast<int>(kind);
        EXPECT_NEAR(p.g, expected.g, 1e-4) << "kind=" << static_cast<int>(kind);
        EXPECT_NEAR(p.b, expected.b, 1e-4) << "kind=" << static_cast<int>(kind);
    };

    expect_kind_color(mpviz::MapKind::CENTERLINE, theme->palette.lane_centerline);
    expect_kind_color(mpviz::MapKind::LEFT_BOUNDARY, theme->palette.lane_boundary);
    expect_kind_color(mpviz::MapKind::RIGHT_BOUNDARY, theme->palette.lane_boundary);
    expect_kind_color(mpviz::MapKind::CROSSWALK, theme->palette.crosswalk);
    expect_kind_color(mpviz::MapKind::ROAD_SURFACE, theme->palette.road);
    // ROAD_EDGE (user directive 2026-09-08): its own dedicated token.
    expect_kind_color(mpviz::MapKind::ROAD_EDGE, theme->palette.road_edge);
    // STOPLINE has no dedicated token (decision #6, deliberate YAGNI) --
    // falls back to the pre-existing laneMaterial / palette.lane_paint,
    // unchanged behaviour, same as every other undedicated kind.
    expect_kind_color(mpviz::MapKind::STOPLINE, theme->palette.lane_paint);

    mpviz::destroy_renderer(r);
}

// ── Epic 3 Task 1 (VM-036) Step 2: crosswalk-hatch dedupe fix ───────────

TEST(MapElements, CrosswalkHatchFiresOnRecordedFivePointClosedPolyline) {
    // The exact 5 points from test/fixtures/hd_map_local_elements_0.yaml,
    // ns: crosswalk_8043 (verified against the committed node-package
    // fixture) -- point[0] == point[4] (closing vertex), a REAL recorded
    // shape, not an invented 4-point quad. Before this epic's fix,
    // build_crosswalk_hatch(pts, 5, ...) returns empty (n != 4 guard); the
    // dedupe itself is adapter-side (hd_map.cpp, decision #2) and is what
    // turns this into the 4-point call below on real data for the first
    // time. This regression pins the library half: the geometry, once
    // deduped, hatches correctly on a REAL (non-axis-aligned) quad, not
    // just the synthetic one SyntheticLaneAndCrosswalkChangePixelsVsBaseline
    // already exercises.
    const mpviz::Vec3 pts_after_dedupe[4] = {
        {-39.50850289011474, 45.33743457749722, -0.000284586101770401},
        {-54.35884356129442, 45.2288915511252, -0.00039308611303567886},
        {-54.476791014440394, 47.18497371095612, -0.0004083588719367981},
        {-39.52170872121907, 47.27801090662989, -0.0002988511696457863},
    };
    auto tris = mpviz::detail::build_crosswalk_hatch(pts_after_dedupe, 4, 0.02f);
    EXPECT_FALSE(tris.empty());
}

// ── Epic 3 Task 1 (VM-036) Step 4: dash-kind flip -- boundary dashes,
//    centerline stays one solid mesh ─────────────────────────────────────

TEST(MapElementsGolden, DashedBoundaryProducesSameDashRunsAsThePreMoveAlgorithm) {
    // A 10 m straight polyline (same shape the pre-Epic3 adapter-side
    // ChopIntoDashes worked example used) on a kind==LEFT_BOUNDARY element:
    // one whole element crossing the ABI boundary, dashed librarywise into
    // 4 mesh chunks at kDashLenM=1.5/kGapLenM=1.5 ([0,1.5],[3,4.5],[6,7.5],
    // [9,10]) -- checked via the mesh-count hook, not a full-frame SSIM
    // (which would also carry road-fill/per-kind color, a separate concern
    // per decision #3's own AC).
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::LEFT_BOUNDARY;
    mpviz::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::map_element_mesh_count(r), 4u);
    mpviz::destroy_renderer(r);
}

TEST(MapElementsGolden, CenterlineOfSameGeometryProducesOneMeshChunkNotDashSplit) {
    // The flip's other half (decision #3): the SAME geometry on
    // kind==CENTERLINE stays ONE mesh chunk -- never dash-split.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::CENTERLINE;
    mpviz::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::map_element_mesh_count(r), 1u);
    mpviz::destroy_renderer(r);
}

// ── User directive 2026-09-08: ROAD_EDGE is solid (never dashed), and
//    CENTERLINE renders as dot discs (not a strip) ─────────────────────

TEST(MapElementsGolden, RoadEdgeOfSameGeometryProducesOneMeshChunkNeverDashed) {
    // The dash-flip's third case: ROAD_EDGE is NOT a BOUNDARY kind
    // (IsBoundaryKind() only matches LEFT_BOUNDARY/RIGHT_BOUNDARY), so the
    // same 10 m geometry that dashes into 4 chunks under LEFT_BOUNDARY
    // stays ONE solid mesh chunk under ROAD_EDGE -- "the boundary of the
    // road... should not be dashed" (user directive 2026-09-08, verbatim).
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::ROAD_EDGE;
    mpviz::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::map_element_mesh_count(r), 1u);
    mpviz::destroy_renderer(r);
}

TEST(MapElements, CenterlineRendersAsDotDiscsNotAStrip) {
    // build_centerline_dots() (map_elements.cpp) replaces the old solid
    // ribbon strip for CENTERLINE with arc-length-spaced filled discs.
    // Proven via vertex count, not a full-frame SSIM: a 10 m straight line
    // -> a strip (build_ribbon_flat) would be 2*(2 points) = 4 raw
    // vertices, flattened to 6*(2-1) = 6 triangle-list vertices. Dots at
    // kCenterlineDotSpacingM=2.0 m over a 10 m line land at s=0,2,4,6,8,10
    // (6 dots, the "<=" endpoint-inclusive convention), each a
    // kCenterlineDotSegments=10-wedge fan = 30 vertices/dot -> 180 total --
    // an order of magnitude more vertices than a strip of the same
    // geometry would ever produce, and an exact, derivable number (not
    // "just different").
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::CENTERLINE;
    mpviz::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::map_element_mesh_count(r), 1u);
    EXPECT_EQ(mpviz::testing::map_element_total_vertex_count(r), 180u)
        << "expected 6 dots * 10 segments * 3 verts/wedge -- a strip of this same "
           "2-point geometry would be 6 vertices, not 180";
    mpviz::destroy_renderer(r);
}

// ── Epic 3 Task 1 (VM-036) Step 5: road-surface fill ────────────────────

TEST(MapElements, RoadSurfaceKindTriangulatesTheTwoRailEncodingIntoAStrip) {
    // 16 stations per rail (kRoadFillSamples), point_count == 32 -- assert
    // the built mesh renders as SOMETHING distinguishable from an empty
    // scene (a Filament-free triangle-count hook would need a new export;
    // a pixel-difference check against a no-map-data baseline is the same
    // "actually renders" proof this epic's other synthetic map tests use).
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};

    auto* baseR = mpviz::create_renderer(cfg);
    if (!baseR) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph empty{};
    mpviz::set_scene(baseR, empty);
    const std::vector<uint8_t> baseline = render_once(baseR, pose);
    mpviz::destroy_renderer(baseR);

    auto* r = mpviz::create_renderer(cfg);
    ASSERT_TRUE(r);
    constexpr uint32_t kN = 16;
    mpviz::Vec3 pts[2 * kN];
    for (uint32_t i = 0; i < kN; ++i) {
        const double y = -3.0 + 6.0 * static_cast<double>(i) / static_cast<double>(kN - 1);
        pts[i] = {-1.0, y, 0.0};       // left rail
        pts[kN + i] = {1.0, y, 0.0};   // right rail
    }
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2 * kN;
    e.kind = mpviz::MapKind::ROAD_SURFACE;
    mpviz::SceneGraph s{};
    // Epic 3 Task 2 (VM-034) Step 3: same "give it a valid ego" fallout as
    // SyntheticLaneAndCrosswalkChangePixelsVsBaseline above -- this test
    // proves the strip renders, not the ego-invalid fade path.
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    const std::vector<uint8_t> withRoad = render_once(r, pose);
    mpviz::destroy_renderer(r);

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
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    mpviz::Vec3 pts[5] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 5;
    e.kind = mpviz::MapKind::ROAD_SURFACE;
    mpviz::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::map_element_mesh_count(r), 0u);
    mpviz::destroy_renderer(r);
}

// ── Epic 3 Task 1 (VM-036) Step 7: map_element_rebuild_count hook ───────

TEST(MapElements, RebuildCountStaysZeroOnUnchangedContentSignature) {
    // Epic 2's own untested AC ("cached, no per-frame rebuild"), finally
    // checked: publishing the IDENTICAL SceneGraph twice must not rebuild
    // the second time (a cache-hit on the unchanged content signature).
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::CENTERLINE;
    mpviz::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};

    mpviz::set_scene(r, s);
    render_once(r, pose);
    const uint64_t afterFirst = mpviz::testing::map_element_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);

    mpviz::set_scene(r, s);  // identical content signature
    render_once(r, pose);
    EXPECT_EQ(mpviz::testing::map_element_rebuild_count(r), afterFirst)
        << "publishing the identical MapElement a second time triggered a rebuild -- "
           "the content-signature cache isn't actually a cache";
    mpviz::destroy_renderer(r);
}

// ── 2d: map elements actually render (synthetic — see the STATED
// DEVIATION below re: the recorded-fixture golden) ──────────────────────

TEST(MapElements, SyntheticLaneAndCrosswalkChangePixelsVsBaseline) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};

    auto* baseR = mpviz::create_renderer(cfg);
    if (!baseR) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph empty{};
    mpviz::set_scene(baseR, empty);
    const std::vector<uint8_t> baseline = render_once(baseR, pose);
    mpviz::destroy_renderer(baseR);

    auto* r = mpviz::create_renderer(cfg);
    ASSERT_TRUE(r);
    // A lane centerline running toward the camera...
    const mpviz::Vec3 lanePts[] = {{0, -6, 0}, {0, -2, 0}, {0, 2, 0}, {0, 6, 0}};
    // ...and a crosswalk quad straddling it (exercises the polygon + hatch
    // path, not just the polyline path).
    const mpviz::Vec3 crosswalkPts[] = {{-1.5, -0.5, 0}, {1.5, -0.5, 0}, {1.5, 0.5, 0}, {-1.5, 0.5, 0}};
    mpviz::MapElement elems[2]{};
    elems[0].points = lanePts;
    elems[0].point_count = 4;
    elems[0].is_polygon = 0;
    elems[1].points = crosswalkPts;
    elems[1].point_count = 4;
    elems[1].is_polygon = 1;
    mpviz::SceneGraph s{};
    // Epic 3 Task 2 (VM-034) Step 3: update_map_elements() now gates on
    // ego.valid (fades to 0 while invalid, the ego-invalid cosmetic fix) --
    // this test isn't exercising that path, so it needs a valid ego like
    // every other "prove the geometry actually renders" synthetic scene now
    // does. last_update_sec/sim_time_sec both default to 0.0 (fresh).
    s.ego.valid = 1;
    s.map_elements = elems;
    s.map_element_count = 2;
    mpviz::set_scene(r, s);
    const std::vector<uint8_t> withMap = render_once(r, pose);
    // Not a golden (no committed comparison target) -- just a viewable PNG
    // of the new lane+crosswalk render path for human sanity-checking
    // before Step 8/10's recorded-fixture golden exists to promote.
    mpviz::testing::render_and_compare(r, pose, "/nonexistent-golden.png",
                                        "/tmp/map_elements_synthetic_actual.png");
    mpviz::destroy_renderer(r);

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
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -8, 6}, {0, 0, 0}, 60.0};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 a[] = {{-4, -4, 0}, {-4, 4, 0}};
    const mpviz::Vec3 b[] = {{0, -4, 0}, {0, 4, 0}};
    const mpviz::Vec3 c[] = {{4, -4, 0}, {4, 4, 0}};
    mpviz::MapElement three[3]{};
    three[0].points = a;
    three[0].point_count = 2;
    three[1].points = b;
    three[1].point_count = 2;
    three[2].points = c;
    three[2].point_count = 2;
    mpviz::SceneGraph s3{};
    s3.map_elements = three;
    s3.map_element_count = 3;
    mpviz::set_scene(r, s3);
    const std::vector<uint8_t> withThree = render_once(r, pose);

    mpviz::SceneGraph s1{};
    s1.map_elements = three;  // only the first element now
    s1.map_element_count = 1;
    mpviz::set_scene(r, s1);
    const std::vector<uint8_t> withOne = render_once(r, pose);

    EXPECT_NE(withThree, withOne)
        << "removing 2 of 3 lane elements produced an identical frame -- vanished "
           "elements were not evicted from the diff cache";
    mpviz::destroy_renderer(r);
}

// ── 2d golden: recorded HD-map fixture, both themes ─────────────────────

namespace {

// `hd_map_local_elements_0.geom` is committed (emitted by the node-side
// HdMapAdapter's own gtest, Task 2 Step 7, from the real, filtered
// hd_map_local_elements_0.yaml; format extended and fixture regenerated
// Epic 3 Task 1 / VM-036 -- dashing moved renderer-side, decision #3, and
// road-fill added, decision #5) -- 74 elements (58 lane/crosswalk markers
// + 16 synthesized ROAD_SURFACE elements). Returns true only for the one
// legitimate runtime skip left (no GPU/EGL), matching every other renderer
// test's convention.
bool RunMapGolden(const char* theme_name, const char* golden_name, const char* out_name) {
    const std::string geomPath =
        std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/hd_map_local_elements_0.geom";
    mpviz::testing::MapGeom g = mpviz::testing::load_map_geom(geomPath.c_str());
    auto& elems = g.elements;
    EXPECT_FALSE(elems.empty());

    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, theme_name};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) return true;  // GTEST_SKIP path, no GPU/EGL

    mpviz::SceneGraph s{};
    s.sim_time_sec = 10.0;
    // Epic 3 Task 2 (VM-034) Step 2: the `.geom` text-dump format
    // (golden.hpp's own comment: `<is_polygon> <kind> <lane_id> <n> ...`)
    // carries no last_update_sec, so every loaded element defaults to 0.0 --
    // against sim_time_sec=10.0 that reads as maximally stale and this
    // golden (proving the lane network at an ego offset, not staleness)
    // would fade to near-invisible. Stamp every element "just refreshed"
    // instead, the same way a live HdMapAdapter would on its next fill().
    for (auto& e : elems) e.last_update_sec = s.sim_time_sec;
    const mpviz::Vec3 c = mpviz::testing::centroid(elems);
    // Pins that the fixture really is far from the map origin -- i.e. still
    // the "Epic 1 rendered pure void here" position this task exists to fix.
    // STATED DEVIATION (node-side pass, Task 2 Step 7/8): the plan's own
    // Step 8 pseudocode wrote this threshold as "> 50.0" before the real
    // filtered fixture existed. The committed hd_map_local_elements_0.geom
    // (58 kept elements / 873 points, emitted from the real, filtered
    // hd_map_local_elements_0.yaml via HdMapAdapter) centroids at
    // (-46.73, 12.82), hypot ~48.46 -- comfortably outside Epic 1's 40x40m
    // origin-centred void patch (kGroundHalfExtent=20m) but just under the
    // speculative round number. 45.0 is the honest threshold for this
    // measured, real, recorded-and-filtered dataset -- not a re-guess, a
    // re-measurement.
    EXPECT_GT(std::hypot(c.x, c.y), 45.0);
    s.ego = {c, 0.0, 3.0, 1};
    s.map_elements = elems.data();
    s.map_element_count = static_cast<uint32_t>(elems.size());
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{c.x - 8, c.y - 8, 6}, {c.x, c.y, c.z}, 60.0};
    const std::string goldenPath = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/goldens/" + golden_name;
    const std::string outPath = std::string("/tmp/") + out_name;
    double ssim = mpviz::testing::render_and_compare(r, pose, goldenPath.c_str(), outPath.c_str());
    EXPECT_GT(ssim, 0.98);
    mpviz::destroy_renderer(r);
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

// ── User directive 2026-09-08: centerline-ON golden (dot guidance) ──────
// Profiles gate whether CENTERLINE elements ever reach the library (hidden
// by default now, per urban_profile.yaml/sim_profile.yaml) -- but the
// library itself renders them unconditionally whenever they're present in
// the SceneGraph (decision #5's own precedent: "profiles don't gate
// library tests"). Synthetic scene, not fixture-derived (the committed
// hd_map_local_elements_0.geom no longer carries any CENTERLINE elements,
// by design), feeding CENTERLINE elements directly to prove the dot-disc
// path renders as SOMETHING distinct and legible, independent of whatever
// a shipped profile currently gates.
TEST(MapGolden, CenterlineDotsOnState_DarkAdas) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -10, 8}, {0, 0, 0}, 60.0};

    auto* baseR = mpviz::create_renderer(cfg);
    if (!baseR) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph empty{};
    mpviz::set_scene(baseR, empty);
    const std::vector<uint8_t> baseline = render_once(baseR, pose);
    mpviz::destroy_renderer(baseR);

    auto* r = mpviz::create_renderer(cfg);
    ASSERT_TRUE(r);
    // A handful of straight/curved centerlines across the frame -- enough
    // to show dot spacing/radius at a glance, same synthetic-scene spirit
    // as SyntheticLaneAndCrosswalkChangePixelsVsBaseline above.
    const mpviz::Vec3 line_a[] = {{-6, -6, 0}, {-6, 6, 0}};
    const mpviz::Vec3 line_b[] = {{0, -6, 0}, {0, 0, 0}, {2, 6, 0}};
    const mpviz::Vec3 line_c[] = {{6, -6, 0}, {6, 6, 0}};
    mpviz::MapElement elems[3]{};
    elems[0].points = line_a;
    elems[0].point_count = 2;
    elems[0].kind = mpviz::MapKind::CENTERLINE;
    elems[1].points = line_b;
    elems[1].point_count = 3;
    elems[1].kind = mpviz::MapKind::CENTERLINE;
    elems[2].points = line_c;
    elems[2].point_count = 2;
    elems[2].kind = mpviz::MapKind::CENTERLINE;
    mpviz::SceneGraph s{};
    // Epic 3 Task 2 (VM-034) Step 3: same "give it a valid ego" fallout as
    // SyntheticLaneAndCrosswalkChangePixelsVsBaseline above.
    s.ego.valid = 1;
    s.map_elements = elems;
    s.map_element_count = 3;
    mpviz::set_scene(r, s);
    const std::vector<uint8_t> withDots = render_once(r, pose);

    // No committed golden yet (HARD RULE: candidates to /tmp, no promotion
    // this task) -- SSIM against a nonexistent path reads low/0, same
    // established pattern as SyntheticLaneAndCrosswalkChangePixelsVsBaseline's
    // "/nonexistent-golden.png" above; the real assertion is the pixel-diff
    // against an empty scene, proving the dot-disc path actually renders
    // something.
    mpviz::testing::render_and_compare(
        r, pose, MPVIZ_TEST_DATA_DIR "/tests/goldens/centerline_dots_dark_adas.png",
        "/tmp/centerline_dots_dark_adas_actual.png");
    mpviz::destroy_renderer(r);

    ASSERT_EQ(baseline.size(), withDots.size());
    size_t differing = 0;
    for (size_t i = 0; i < baseline.size(); ++i) {
        if (baseline[i] != withDots[i]) ++differing;
    }
    EXPECT_GT(differing, 0u) << "CENTERLINE dot-disc elements produced no visible pixel "
                                 "difference from a scene with no map data at all";
}

// ── Epic 3 Task 2 (VM-034) Step 2: staleness fade, the one shared path ──

TEST(MapElements, FadesViaSharedStalenessAlpha) {
    // Same test SHAPE as every other category's own "...FadesViaShared
    // StalenessAlpha" (epic2 plan's exemplar; test_objects.cpp's
    // StaleObjectFadesViaSharedStalenessAlpha is the direct precedent) --
    // publish ONE MapElement with last_update_sec in the past relative to
    // sim_time_sec, render, assert via the test hook that the bound
    // clay_translucent instance's alpha matches staleness_alpha()'s own
    // computed value, NOT a pixel comparison. ego.valid=1 so the Step 3
    // ego-invalid gate (tested separately below) can't be what's driving
    // this alpha down.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::CENTERLINE;
    e.last_update_sec = 10.0 - 0.75;  // 0.75s behind -> alpha ~0.5, same worked
                                       // example test_objects.cpp's own fade test uses
    mpviz::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    const auto info = mpviz::testing::map_element_material_info(r);
    EXPECT_TRUE(info.bound_to_translucent)
        << "a stale map element's renderable must be bound to clay_translucent.mat, not "
           "its opaque per-kind template";
    EXPECT_NEAR(info.alpha, 0.5f, 0.02f);

    mpviz::destroy_renderer(r);
}

TEST(MapElements, FreshMapElementStaysOnTheOpaqueTemplate) {
    // The other half of the fade -- FRESH (last_update_sec == sim_time_sec)
    // must stay on the shared opaque per-kind template, no per-entity
    // instance at all (test_objects.cpp's own "fresh" half, same shape).
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::CENTERLINE;
    e.last_update_sec = 10.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -8, 4}, {0, 0, 0}, 60.0});

    const auto info = mpviz::testing::map_element_material_info(r);
    EXPECT_FALSE(info.bound_to_translucent)
        << "a FRESH map element must stay on the opaque shared template, not get a "
           "per-entity instance";
    EXPECT_NEAR(info.alpha, 1.0f, 1e-4);

    mpviz::destroy_renderer(r);
}

// ── Epic 3 Task 2 (VM-034) Step 3: the ego-invalid map cosmetic ─────────

TEST(MapElements, EgoInvalidFadesMapElementsRatherThanLeavingThemAtFullOpacity) {
    // Root cause (renderer.cpp:1458-1470 / Epic 2 gate finding wf_0ff03eb8-
    // 5ec): update_ground_grid_transform() snaps the ego-following ground/
    // grid patch to the world origin whenever ego.valid==0, but
    // update_map_elements() had no matching gate at all -- real map
    // geometry kept rendering, at full opacity, against an origin-snapped
    // ground. Fix: ego.valid==0 drives alpha to 0 via the SAME fade path
    // Step 2 just wired (not skip-and-freeze, which would leave the last
    // valid frame's geometry at full opacity forever -- the identical bug
    // one frame later).
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const mpviz::Vec3 pts[] = {{0, 0, 0}, {10, 0, 0}};
    mpviz::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.kind = mpviz::MapKind::CENTERLINE;
    e.last_update_sec = 10.0;  // fresh by staleness_alpha's own math
    mpviz::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego.valid = 1;
    s.map_elements = &e;
    s.map_element_count = 1;
    mpviz::CameraPose pose{{0, -8, 4}, {0, 0, 0}, 60.0};

    mpviz::set_scene(r, s);
    render_once(r, pose);
    EXPECT_NEAR(mpviz::testing::map_element_material_info(r).alpha, 1.0f, 1e-4)
        << "sanity check: ego valid + fresh element -> full opacity, before the flip below";

    s.ego.valid = 0;  // TF dropout; the SAME MapElement, still "fresh" by staleness_alpha
    mpviz::set_scene(r, s);
    render_once(r, pose);
    const auto afterEgoInvalid = mpviz::testing::map_element_material_info(r);
    EXPECT_LT(afterEgoInvalid.alpha, 1.0f)
        << "ego.valid==0 must fade map elements toward invisible, not hold them at full "
           "opacity while the ground/grid patch has already snapped to the origin";
    EXPECT_NEAR(afterEgoInvalid.alpha, 0.0f, 1e-4);

    mpviz::destroy_renderer(r);
}
