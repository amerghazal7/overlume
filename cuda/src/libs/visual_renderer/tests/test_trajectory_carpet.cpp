// test_trajectory_carpet.cpp — output_trajectory_carpet (VM-077). Same
// "no Filament type" boundary as every other tests/*.cpp -- see
// trajectory_carpet_test_hooks.hpp. Every scene in this file is hand-built
// synthetic data (mirrors test_point_cloud.cpp's own fixture style).
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "trajectory_carpet_test_hooks.hpp"
#include "test_paths.hpp"
#include "polyline.hpp"  // detail::kMaxPointsPerMesh only -- Filament-free, see its own header comment

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<uint8_t> render_once(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    mpviz::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    return pixels;
}

// One flat triangle per "n/3" group, laid out along +X so an oversized
// carpet (Step 1's chunk test) spans a wide bounding volume like a real
// planning-horizon ribbon would.
std::vector<mpviz::PointCloudPoint> make_triangles(uint32_t n_points, uint32_t rgba) {
    std::vector<mpviz::PointCloudPoint> pts(n_points);
    for (uint32_t i = 0; i < n_points; ++i) {
        const double tri = static_cast<double>(i / 3);
        pts[i].position = {tri * 0.01, static_cast<double>(i % 3) * 0.01, 0.0};
        pts[i].rgba = rgba;
    }
    return pts;
}

}  // namespace

// ── Step 1: one TRIANGLES mesh from a flat multiple-of-three point list ────

TEST(TrajectoryCarpet, BuildsOneTriangleMeshFromAFlatMultipleOfThreePointList) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // 2 triangles = 6 points, opaque red (a==255 -> real color, not the flat
    // sentinel).
    std::vector<mpviz::PointCloudPoint> pts = make_triangles(6, 0xFF0000FFu);
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
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_count(r, 0), 6u);
    mpviz::destroy_renderer(r);
}

// ── Step 1: per-vertex color passes through unchanged when alpha!=0 ────────

TEST(TrajectoryCarpet, PerVertexColorPassesThroughUnchangedWhenAlphaByteIsNonzero) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // 3-point triangle, each vertex a DISTINCT supplied color (the measured
    // velocity-gradient shape: r/g vary, b==0, a==0.70*255≈179 -- but this
    // library's own sentinel rule only cares whether a byte is zero, so 179
    // exercises the real "supplied, non-255" alpha this producer actually
    // sends, not just the point_cloud fixture convention of 255).
    std::vector<mpviz::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0};
    pts[0].rgba = 0xB30000FFu;           // a=0xB3(179) b=0 g=0 r=0xFF
    pts[1].position = {1, 0, 0};
    pts[1].rgba = 0xB300FF00u;           // a=0xB3 b=0 g=0xFF r=0
    pts[2].position = {1, 1, 0};
    pts[2].rgba = 0xB3FF0080u;           // a=0xB3 b=0xFF g=0 r=0x80

    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 0), 0xB30000FFu);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 1), 0xB300FF00u);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 2), 0xB3FF0080u);
    mpviz::destroy_renderer(r);
}

// ── Step 1: alpha==0 substitutes palette.object_tints.unknown ──────────────

TEST(TrajectoryCarpet, AlphaZeroSentinelSubstitutesPaletteObjectTintsUnknown) {
    // Same substitution point_cloud.cpp's resolve_rgba() already implements
    // -- reuse that free function or an identical one-line copy (it's three
    // lines, not worth extracting into a shared header for one second
    // caller yet -- ponytail: duplicate the 3-line helper, promote to a
    // shared header if a third caller ever needs it).
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0};
    pts[1].position = {1, 0, 0};
    pts[2].position = {1, 1, 0};
    // rgba left at zero-init (a==0) -- "no real per-point color supplied".

    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    // dark_adas.yaml's palette.object_tints.unknown == [0.5, 0.5, 0.5] ->
    // to_byte(0.5) == 128 (round(0.5*255+0.5)==128.0), alpha forced to 255
    // (real, substituted color): 128 | (128<<8) | (128<<16) | (255<<24).
    constexpr uint32_t kExpected = 128u | (128u << 8) | (128u << 16) | (255u << 24);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 0), kExpected);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 1), kExpected);
    EXPECT_EQ(mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, 2), kExpected);
    mpviz::destroy_renderer(r);
}

// ── Step 1: chunked past the per-mesh vertex ceiling, triangle-aligned ─────
//
// Regression test for the VM-077 review fix (2026-09-09): this test
// originally asserted vertexCount == kN + (meshCount - 1), baking in
// polyline_chunks()'s LINE_STRIP overlap-by-one-point convention as if it
// were correct for a TRIANGLES primitive. It passed even though the
// implementation was reusing polyline_chunks() (whose 32000-point ceiling
// is not a multiple of 3) on a flat triangle list -- which drops an
// incomplete triangle at 32000 and then stitches every triangle after it
// from three different source triangles. A test that cannot fail is not a
// check: the assertions below read back real per-vertex data at the chunk
// boundary instead of trusting an invariant carried over from the wrong
// primitive type.
TEST(TrajectoryCarpet, TrianglesChunkExactlyWithNoOverlapPastThePerMeshVertexCeiling) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // > kMaxPointsPerMesh (32000, polyline.hpp) and a multiple of 3, so this
    // must chunk across more than one mesh -- none truncated. Per-vertex
    // rgba encodes the vertex's own source index in its low 24 bits with
    // alpha forced to 0xFF (a real, non-sentinel color -- see
    // PerVertexColorPassesThroughUnchangedWhenAlphaByteIsNonzero), so the
    // chunk-boundary vertex read back below can be checked against the
    // exact source index it must map to, not just assumed.
    constexpr uint32_t kN = 40002;
    std::vector<mpviz::PointCloudPoint> pts(kN);
    for (uint32_t i = 0; i < kN; ++i) {
        const double tri = static_cast<double>(i / 3);
        pts[i].position = {tri * 0.01, static_cast<double>(i % 3) * 0.01, 0.0};
        pts[i].rgba = 0xFF000000u | (i & 0x00FFFFFFu);
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

    const size_t meshCount = mpviz::testing::trajectory_carpet_mesh_count(r, 0);
    EXPECT_GT(meshCount, 1u)
        << "a carpet past the per-mesh vertex ceiling must split across multiple meshes";

    // Triangle-aligned, non-overlapping chunks (trajectory_carpet.cpp's
    // triangle_chunks(), NOT polyline.hpp's polyline_chunks() -- a triangle
    // list has no shared join vertex between chunks the way a line strip
    // does): total vertex count must equal the source count EXACTLY, no
    // "+ (meshCount - 1)" overlap term.
    const size_t vertexCount = mpviz::testing::trajectory_carpet_vertex_count(r, 0);
    EXPECT_EQ(vertexCount, static_cast<size_t>(kN))
        << "chunking must conserve every vertex exactly once -- no overlap, none dropped";

    // The first mesh's last vertex must be the source vertex at index
    // kMaxPointsPerMesh/3*3 - 1 == 31997 -- a multiple-of-3 chunk boundary
    // (ending on a whole triangle), not the old ceiling (32000, NOT a
    // multiple of 3) that split a triangle in half and misaligned every
    // triangle in the next chunk.
    constexpr uint32_t kFirstChunkLastIndex = (mpviz::detail::kMaxPointsPerMesh / 3) * 3 - 1;
    static_assert(kFirstChunkLastIndex == 31997u);
    const uint32_t rgba =
        mpviz::testing::trajectory_carpet_vertex_rgba(r, 0, kFirstChunkLastIndex);
    EXPECT_EQ(rgba & 0x00FFFFFFu, kFirstChunkLastIndex)
        << "first mesh's last vertex (idx " << kFirstChunkLastIndex
        << ") must map to that same source vertex, not one stitched from the wrong triangle "
           "across the chunk seam";

    mpviz::destroy_renderer(r);
}

// ── Slots release when a carpet disappears from the next publish ───────────

TEST(TrajectoryCarpet, SlotReleasedWhenCarpetCountDrops) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_triangles(9, 0xFF0000FFu);
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

// ── Staleness fade is the ONE shared MaterialInstance's alpha uniform ───────

TEST(TrajectoryCarpet, MaterialAlphaFollowsStaleness) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_triangles(9, 0xFF0000FFu);
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
    // 0.7, not 1.0 -- the measured producer opacity (m.color.a, VM-077
    // review fix) is folded into the pushed alpha, not just the staleness
    // ramp, so a fresh carpet is translucent rather than fully opaque.
    EXPECT_FLOAT_EQ(mpviz::testing::trajectory_carpet_material_alpha(r), 0.7f);

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

// ── Z-STACK: the adapter's flattened z=0.0 gets lifted above the opaque
//    ground plane, or the carpet z-fights it into invisibility ───────────────
//
// Found live (VM-077 verification, 2026-09-09): a first pass left the
// carpet at the adapter's flattened z=0.0 verbatim, coplanar with the
// opaque ground -- every mesh/vertex-count assertion above stayed green
// (the geometry IS built correctly), but on the real bag the carpet lost
// the depth test against the ground almost everywhere and was invisible on
// screen. A rendered-pixel check was tried too (a red triangle over the
// default ground) and, verified BOTH ways (with and without the lift),
// could not tell the two states apart at this synthetic scene's scale/
// camera distance -- it passed regardless, so it would not actually have
// caught this regression and is not worth keeping (a test that cannot
// fail is not a check). The CPU-side z assertion below is the one that
// FAILS without the fix (verified) and PASSES with it.

TEST(TrajectoryCarpet, VertexZIsLiftedAboveTheFlattenedZeroTheAdapterSends) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts(3);
    pts[0].position = {0, 0, 0.0};  // exactly the adapter's flatten_z output
    pts[1].position = {1, 0, 0.0};
    pts[2].position = {1, 1, 0.0};
    for (auto& p : pts) p.rgba = 0xFF0000FFu;

    mpviz::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    mpviz::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});

    for (size_t i = 0; i < 3; ++i) {
        const float z = mpviz::testing::trajectory_carpet_vertex_z(r, 0, i);
        EXPECT_GT(z, 0.0f)
            << "vertex " << i << " must be lifted above z==0.0 (the opaque ground plane), "
               "not left at the adapter's flattened value verbatim";
        // Regression guard for the VM-077 review fix (2026-09-09): keeping
        // alerts topmost is still correct even though the carpet is
        // translucent (0.7 alpha) while fresh, not opaque -- a partly
        // see-through carpet sitting above an alert ring would still visually
        // merge with it. Must stay strictly below alert_polygons.cpp's
        // kAlertZLiftM (0.06). A future edit that pushes the carpet back
        // above alerts should fail THIS assertion, not just leave the z>0
        // check above vacuously green.
        EXPECT_LT(z, 0.06f) << "vertex " << i
                             << " carpet lift must stay below alert_polygons' kAlertZLiftM (0.06) "
                                "so alerts remain the topmost overlay";
    }
    mpviz::destroy_renderer(r);
}
