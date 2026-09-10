// test_trajectory_carpet.cpp — output_trajectory_carpet, REDIRECTED
// 2026-09-10 to a velocity-colored ribbon stacked into the ribbon stack
// (user directive; see trajectory_carpet.cpp's file header for the full
// rationale). Same "no Filament type" boundary as every other tests/*.cpp -- see
// trajectory_carpet_test_hooks.hpp. Every scene in this file is hand-built
// synthetic data: each mpviz::PointCloudPoint here represents one
// CENTERLINE STATION (position + packed rgba), not a raw wire vertex --
// see the node-side adapter for how real messages become this shape.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "trajectory_carpet_test_hooks.hpp"
#include "test_paths.hpp"
#include "polyline.hpp"  // detail::kMaxPointsPerMesh/polyline_chunks -- Filament-free

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<uint8_t> render_once(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    return pixels;
}

// n stations along +X, 1m apart, all the same supplied (non-sentinel) color.
std::vector<mpviz::PointCloudPoint> make_stations(uint32_t n, uint32_t rgba) {
    std::vector<mpviz::PointCloudPoint> pts(n);
    for (uint32_t i = 0; i < n; ++i) {
        pts[i].position = {static_cast<double>(i), 0.0, 0.0};
        pts[i].rgba = rgba;
    }
    return pts;
}

}  // namespace

// ── Ribbon geometry: extruded from centerline stations, half-width from
//    the theme's ribbon.margin_velocity_m token ────────────────────────────

TEST(TrajectoryCarpet, BuildsExtrudedRibbonFromCenterlineStations) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::trajectory_carpet_mesh_count(r, 0), 1u);
    // 4 stations extruded (left/right rail per station) -> 8 vertices, NOT
    // the old flat-triangle-list's 1:1 point-to-vertex count.
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_count(r, 0), 8u);
    // dark_adas.yaml doesn't author margin_velocity_m -> soft default 1.05 ->
    // (3.5 - 2*1.05) / 2 == 0.7.
    EXPECT_NEAR(mpviz::testing::trajectory_carpet_half_width_m(r, 0), 0.7f, 1e-4f);
    mpviz::destroy_renderer(r);
}

TEST(TrajectoryCarpet, MarginVelocityChangeRebuildsGeometryAtNewHalfWidth) {
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, fixtureDir.c_str(), "ribbon_margin_velocity"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    // ribbon_margin_velocity.yaml: (3.5 - 2*0.9) / 2 == 0.85.
    EXPECT_NEAR(mpviz::testing::trajectory_carpet_half_width_m(r, 0), 0.85f, 1e-4f);
    mpviz::destroy_renderer(r);
}

TEST(TrajectoryCarpet, EffectiveHalfWidthClampsToTheHalfWidthFloor) {
    // ribbon_margin_velocity_extreme.yaml: margin_velocity_m 1.74 -> raw
    // half-width (3.5 - 2*1.74) / 2 == 0.01, must clamp UP to
    // kRibbonMinHalfWidthM (0.12).
    const std::string fixtureDir = std::string(MPVIZ_TEST_DATA_DIR) + "/tests/fixtures/themes";
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, fixtureDir.c_str(),
                            "ribbon_margin_velocity_extreme"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(2, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_NEAR(mpviz::testing::trajectory_carpet_half_width_m(r, 0), 0.12f, 1e-4f);
    mpviz::destroy_renderer(r);
}

// ── Per-vertex color passes through unchanged; alpha==0 sentinel ───────────

TEST(TrajectoryCarpet, PerVertexColorPassesThroughUnchangedWhenAlphaByteIsNonzero) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // 3 stations, each a DISTINCT supplied color (the measured velocity-
    // gradient shape: r/g vary, b==0). Alpha 0xB3 (179), not 0 or 255 --
    // this library's own sentinel rule only cares whether the byte is
    // zero, so this exercises a real "supplied, non-255" alpha.
    std::vector<mpviz::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0};
    pts[0].rgba = 0xB30000FFu;  // a=0xB3 b=0 g=0 r=0xFF
    pts[1].position = {1, 0, 0};
    pts[1].rgba = 0xB300FF00u;  // a=0xB3 b=0 g=0xFF r=0
    pts[2].position = {2, 0, 0};
    pts[2].rgba = 0xB3FF0080u;  // a=0xB3 b=0xFF g=0 r=0x80

    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    // Each station's color lands on BOTH extruded rail vertices (0/1 for
    // station 0, 2/3 for station 1, 4/5 for station 2) -- color is
    // per-station, not per-rail, per the measurement report.
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xB30000FFu);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 1), 0xB30000FFu);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 2), 0xB300FF00u);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 3), 0xB300FF00u);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 4), 0xB3FF0080u);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 5), 0xB3FF0080u);
    mpviz::destroy_renderer(r);
}

TEST(TrajectoryCarpet, AlphaZeroSentinelSubstitutesPaletteObjectTintsUnknown) {
    // Same substitution point_cloud.cpp's resolve_rgba() already implements
    // -- reuse that free function or an identical one-line copy (ponytail:
    // duplicate the 3-line helper, promote to a shared header if a third
    // caller ever needs it).
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0};
    pts[1].position = {1, 0, 0};
    pts[2].position = {2, 0, 0};
    // rgba left at zero-init (a==0) -- "no real per-station color supplied".

    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    // dark_adas.yaml's palette.object_tints.unknown == [0.5, 0.5, 0.5] ->
    // to_byte(0.5) == 128, alpha forced to 255 (real, substituted color).
    constexpr uint32_t kExpected = 128u | (128u << 8) | (128u << 16) | (255u << 24);
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, i), kExpected);
    }
    mpviz::destroy_renderer(r);
}

// ── Chunked past the per-mesh vertex ceiling ────────────────────────────────

TEST(TrajectoryCarpet, RibbonChunksAcrossMeshesWithoutTruncation) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    constexpr uint32_t kN = 40000;
    std::vector<mpviz::PointCloudPoint> pts(kN);
    for (uint32_t i = 0; i < kN; ++i) {
        pts[i].position = {static_cast<double>(i) * 0.1, 0.0, 0.0};
        pts[i].rgba = 0xFF0000FFu;
    }
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = kN;
    tc.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    mpviz::CameraPose pose{{0, -20, 20}, {2, 0, 0}, 60.0};
    render_once(r, pose);

    const auto chunks = mpviz::detail::polyline_chunks(kN);
    ASSERT_EQ(chunks.size(), 2u) << "40000 stations should split into exactly 2 chunks at "
                                     "kMaxPointsPerMesh=32000 -- fixture assumption changed?";
    size_t expectedVerts = 0;
    for (const auto& [a, b] : chunks) expectedVerts += 2 * static_cast<size_t>(b - a);

    EXPECT_EQ(mpviz::testing::trajectory_carpet_mesh_count(r, 0), chunks.size());
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_count(r, 0), expectedVerts)
        << "vertex count doesn't match polyline_chunks()'s own math -- a station was lost "
           "at the chunk-split seam";
    mpviz::destroy_renderer(r);
}

// ── Slots release when a carpet disappears from the next publish ───────────

TEST(TrajectoryCarpet, SlotReleasedWhenCarpetCountDrops) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(3, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    ASSERT_EQ(mpviz::testing::trajectory_carpet_mesh_count(r, 0), 1u);

    mpviz::SceneGraph empty{};
    mpviz::set_scene(r, empty);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_EQ(mpviz::testing::trajectory_carpet_mesh_count(r, 0), 0u)
        << "a slot past the new (lower) trajectory_carpet_count must be torn down, not left dangling";

    mpviz::destroy_renderer(r);
}

// ── Staleness fade is the ONE shared MaterialInstance's alpha uniform,
//    OPAQUE while fresh (2026-09-10 redirect -- the old 0.7 producer-alpha
//    parity is superseded) ───────────────────────────────────────────────────

TEST(TrajectoryCarpet, MaterialAlphaFollowsStalenessOpaqueWhileFresh) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(3, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    // 1.0, not 0.7 -- the old measured producer opacity (m.color.a) is
    // superseded by the user directive: fresh renders fully opaque.
    EXPECT_FLOAT_EQ(mpviz::testing::trajectory_carpet_material_alpha(r), 1.0f);

    // Past kStaleFadeTimeoutSec (1.0s) with no new publish -- render_frame()
    // re-reads the same last-published scene every call (freeze-frame), so
    // sim_time_sec must be re-published to move the clock forward, same
    // convention every other staleness test in this suite uses.
    mpviz::SceneGraph stale = s;
    stale.sim_time_sec = 5.0;
    mpviz::set_scene(r, stale);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_FLOAT_EQ(mpviz::testing::trajectory_carpet_material_alpha(r), 0.0f);

    mpviz::destroy_renderer(r);
}

// ── Z-STACK: lifted between LOCAL and BEHAVIOR, below alerts ───────────────

TEST(TrajectoryCarpet, VertexZIsLiftedAboveTheFlattenedZeroTheAdapterSends) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0.0};  // exactly the adapter's flatten_z output
    pts[1].position = {1, 0, 0.0};
    pts[2].position = {2, 0, 0.0};
    for (auto& p : pts) p.rgba = 0xFF0000FFu;

    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    for (size_t i = 0; i < 6; ++i) {
        const float z = mpviz::testing::trajectory_carpet_vertex_z(r, 0, i);
        EXPECT_GT(z, 0.045f) << "vertex " << i
                              << " must be lifted ABOVE LOCAL's own z-lift (0.045) -- "
                                 "\"stacked on top of local ribbon\" per the user directive";
        EXPECT_LT(z, 0.05f) << "vertex " << i
                             << " must stay BELOW BEHAVIOR's z-lift (0.05) -- the hero ribbon "
                                "must remain topmost of the path/ribbon stack";
    }
    mpviz::destroy_renderer(r);
}

// ── Ego-proximity clip: identical mechanism every other ribbon uses ────────

TEST(TrajectoryCarpet, ClipShrinksGeometryWhenEgoIsMidCarpet) {
    // A straight carpet along +X; ego sits AT x=1 (well within the
    // proximity gate) -- the behind-ego half must not appear in the built
    // geometry.
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);  // x = 0,1,2,3,4
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.ego = {{1, 0, 0}, 0.0, 0.0, /*valid=*/1};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {2, 0, 0}, 60.0});

    EXPECT_LT(mpviz::testing::trajectory_carpet_vertex_count(r, 0), 5u * 2)
        << "clip did not actually shrink the built geometry";
    mpviz::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ProximityGateSkipsClipWhenEgoIsFarFromTheCarpet) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.ego = {{1, 20, 0}, 0.0, 0.0, /*valid=*/1};  // 20m off to the side
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 30}, {2, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_count(r, 0), 5u * 2)
        << "a carpet the ego is nowhere near must render whole, not clipped";
    mpviz::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ClipAppliesOnlyWhenEgoIsValid) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.ego = {{1, 0, 0}, 0.0, 0.0, /*valid=*/0};  // on the carpet, but invalid
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {2, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_count(r, 0), 5u * 2)
        << "an invalid ego must never clip a carpet";
    mpviz::destroy_renderer(r);
}

TEST(TrajectoryCarpet, ParkedEgoCausesZeroTrajectoryCarpetRebuilds) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(5, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.ego = {{1, 0, 0}, 0.0, 0.0, /*valid=*/1};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::CameraPose pose{{0, -10, 10}, {2, 0, 0}, 60.0};

    mpviz::set_scene(r, s);
    render_once(r, pose);
    const uint64_t afterFirst = mpviz::testing::trajectory_carpet_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);

    for (int i = 0; i < 10; ++i) {
        mpviz::set_scene(r, s);  // identical content + identical parked ego, every frame
        render_once(r, pose);
    }
    EXPECT_EQ(mpviz::testing::trajectory_carpet_rebuild_count(r), afterFirst)
        << "a parked ego re-triggered rebuilds -- the quantized clip station isn't stable";
    mpviz::destroy_renderer(r);
}

// ── SIGNATURE PROPERTY PIN (VM-077): a message that changes ONLY per-vertex
//    color, with byte-identical station positions, must NOT rebuild the
//    mesh -- this property is independently worth pinning regardless of
//    root cause (the plan's 2026-09-10 "Live verification, CORRECTED"
//    section found H2 was NOT the demonstrated driver of the reported
//    flicker; this test only pins that color-only drift is signature-inert).
//    An accepted tradeoff, not silently invented around: colors freeze at
//    the last-built values until the next position-changing rebuild (see
//    trajectory_carpet.cpp's file header). ────────────────────────────────

TEST(TrajectoryCarpet, SameStationPositionsWithDriftingColorAloneCausesNoRebuild) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    tc.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    const uint64_t afterFirst = mpviz::testing::trajectory_carpet_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xFF0000FFu);

    // Same positions, EVERY message's color drifts (exactly the measured
    // "planner only advances the near station every 3-4 callbacks, but the
    // packed rgba drifts every message" shape) -- 20 republishes, each a
    // different color, none touching a single station's position.
    for (uint32_t i = 0; i < 20; ++i) {
        for (auto& p : pts) p.rgba = 0xFF000000u | (i + 1);  // distinct color each time
        tc.last_update_sec = 0.0;
        mpviz::set_scene(r, s);
        render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    }

    EXPECT_EQ(mpviz::testing::trajectory_carpet_rebuild_count(r), afterFirst)
        << "color-only drift (identical station positions) rebuilt the mesh -- this is the "
           "exact VM-077 H2 flicker regression: color must not be part of the content signature";
    // The displayed color stays frozen at the first-built value -- the
    // explicit, accepted tradeoff (see trajectory_carpet.cpp's file header).
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xFF0000FFu);
    mpviz::destroy_renderer(r);
}

TEST(TrajectoryCarpet, IdenticalRepublishCausesNoRebuild) {
    // Baseline hygiene: even a byte-identical republish (same positions,
    // same colors) must not rebuild.
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_stations(4, 0xFF0000FFu);
    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;

    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    const uint64_t afterFirst = mpviz::testing::trajectory_carpet_rebuild_count(r);
    EXPECT_GT(afterFirst, 0u);

    for (int i = 0; i < 5; ++i) {
        mpviz::set_scene(r, s);
        render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    }
    EXPECT_EQ(mpviz::testing::trajectory_carpet_rebuild_count(r), afterFirst);
    mpviz::destroy_renderer(r);
}
