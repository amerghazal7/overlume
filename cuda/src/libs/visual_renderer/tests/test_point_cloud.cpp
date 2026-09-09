// test_point_cloud.cpp — point clouds (Epic 3 Task 6 / VM-035). Same
// "no Filament type" boundary as every other tests/*.cpp -- see
// point_cloud_test_hooks.hpp. No PointCloud2 topic exists in the recorded
// bag or stack (FIXTURE GAP, see the node adapter's own header comment) --
// every scene in this file is hand-built synthetic data.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "point_cloud_test_hooks.hpp"
#include "test_paths.hpp"

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

std::vector<mpviz::PointCloudPoint> make_points(uint32_t n) {
    std::vector<mpviz::PointCloudPoint> pts(n);
    for (uint32_t i = 0; i < n; ++i) {
        pts[i].position = {static_cast<double>(i) * 0.01, 0.0, 0.0};
        pts[i].rgba = 0xFF0000FFu;  // opaque red, a==255 -> real color, not the flat sentinel
    }
    return pts;
}

}  // namespace

// ── Step 2: chunked past the uint16 index ceiling, same shape as every
//    other polyline_chunks() consumer (ribbon/map_elements) ────────────────

TEST(PointCloud, ChunkedUnderTheUint16IndexCeilingLikeEveryPolyline) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // > kMaxPointsPerMesh (32000, polyline.hpp) so this must chunk across
    // more than one mesh -- none truncated.
    constexpr uint32_t kN = 40000;
    std::vector<mpviz::PointCloudPoint> pts = make_points(kN);
    mpviz::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = kN;
    pc.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    mpviz::set_scene(r, s);

    mpviz::CameraPose pose{{0, -20, 20}, {2, 0, 0}, 60.0};
    render_once(r, pose);

    const size_t meshCount = mpviz::testing::point_cloud_mesh_count(r, 0);
    EXPECT_GT(meshCount, 1u) << "a cloud past kMaxPointsPerMesh must split across multiple meshes";
    // polyline_chunks() overlaps consecutive chunks by one point (shared
    // join vertex) -- total vertices is therefore kN plus one per chunk
    // boundary, same accounting ribbon.cpp's own chunk tests use.
    const size_t vertexCount = mpviz::testing::point_cloud_vertex_count(r, 0);
    EXPECT_EQ(vertexCount, static_cast<size_t>(kN) + (meshCount - 1));

    mpviz::destroy_renderer(r);
}

// ── A small, unchunked cloud renders as exactly one mesh ────────────────────

TEST(PointCloud, SmallCloudIsOneMesh) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_points(50);
    mpviz::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = static_cast<uint32_t>(pts.size());
    pc.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0.25, 0, 0}, 60.0});

    EXPECT_EQ(mpviz::testing::point_cloud_mesh_count(r, 0), 1u);
    EXPECT_EQ(mpviz::testing::point_cloud_vertex_count(r, 0), 50u);
    mpviz::destroy_renderer(r);
}

// ── Slots release when a cloud disappears from the next publish ────────────

TEST(PointCloud, SlotReleasedWhenCloudCountDrops) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_points(10);
    mpviz::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = static_cast<uint32_t>(pts.size());
    mpviz::SceneGraph s{};
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    ASSERT_EQ(mpviz::testing::point_cloud_mesh_count(r, 0), 1u);

    mpviz::SceneGraph empty{};
    mpviz::set_scene(r, empty);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_EQ(mpviz::testing::point_cloud_mesh_count(r, 0), 0u)
        << "a slot past the new (lower) point_cloud_count must be torn down, not left dangling";

    mpviz::destroy_renderer(r);
}

// ── Staleness fade is the ONE shared MaterialInstance's alpha uniform ───────

TEST(PointCloud, MaterialAlphaFollowsStaleness) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<mpviz::PointCloudPoint> pts = make_points(10);
    mpviz::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = static_cast<uint32_t>(pts.size());
    pc.last_update_sec = 0.0;
    mpviz::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    mpviz::set_scene(r, s);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_FLOAT_EQ(mpviz::testing::point_cloud_material_alpha(r), 1.0f);

    // Past kStaleFadeTimeoutSec (1.0s, renderer_internal.hpp) with no new
    // publish -- render_frame() re-reads the same last-published scene
    // every call (freeze-frame), so sim_time_sec must be re-published to
    // move the clock forward, same convention every other staleness test
    // in this suite uses.
    mpviz::SceneGraph stale = s;
    stale.sim_time_sec = 5.0;
    mpviz::set_scene(r, stale);
    render_once(r, mpviz::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_FLOAT_EQ(mpviz::testing::point_cloud_material_alpha(r), 0.0f);

    mpviz::destroy_renderer(r);
}
