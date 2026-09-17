// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// test_point_cloud.cpp — point clouds (Epic 3 Task 6 / VM-035). Same
// "no Filament type" boundary as every other tests/*.cpp -- see
// point_cloud_test_hooks.hpp. No PointCloud2 topic exists in the recorded
// bag or stack (FIXTURE GAP, see the node adapter's own header comment) --
// every scene in this file is hand-built synthetic data.
#include "overlume/api.h"
#include "overlume/scene.h"

#include "point_cloud_test_hooks.hpp"
#include "test_paths.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<uint8_t> render_once(overlume::VisualRenderer* r, const overlume::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    return pixels;
}

std::vector<overlume::PointCloudPoint> make_points(uint32_t n) {
    std::vector<overlume::PointCloudPoint> pts(n);
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
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // > kMaxPointsPerMesh (32000, polyline.hpp) so this must chunk across
    // more than one mesh -- none truncated.
    constexpr uint32_t kN = 40000;
    std::vector<overlume::PointCloudPoint> pts = make_points(kN);
    overlume::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = kN;
    pc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{0, -20, 20}, {2, 0, 0}, 60.0};
    render_once(r, pose);

    const size_t meshCount = overlume::testing::point_cloud_mesh_count(r, 0);
    EXPECT_GT(meshCount, 1u) << "a cloud past kMaxPointsPerMesh must split across multiple meshes";
    // polyline_chunks() overlaps consecutive chunks by one point (shared
    // join vertex) -- total vertices is therefore kN plus one per chunk
    // boundary, same accounting ribbon.cpp's own chunk tests use.
    const size_t vertexCount = overlume::testing::point_cloud_vertex_count(r, 0);
    EXPECT_EQ(vertexCount, static_cast<size_t>(kN) + (meshCount - 1));

    overlume::destroy_renderer(r);
}

// ── A small, unchunked cloud renders as exactly one mesh ────────────────────

TEST(PointCloud, SmallCloudIsOneMesh) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_points(50);
    overlume::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = static_cast<uint32_t>(pts.size());
    pc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0.25, 0, 0}, 60.0});

    EXPECT_EQ(overlume::testing::point_cloud_mesh_count(r, 0), 1u);
    EXPECT_EQ(overlume::testing::point_cloud_vertex_count(r, 0), 50u);
    overlume::destroy_renderer(r);
}

// ── Slots release when a cloud disappears from the next publish ────────────

TEST(PointCloud, SlotReleasedWhenCloudCountDrops) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_points(10);
    overlume::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = static_cast<uint32_t>(pts.size());
    overlume::SceneGraph s{};
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    ASSERT_EQ(overlume::testing::point_cloud_mesh_count(r, 0), 1u);

    overlume::SceneGraph empty{};
    overlume::set_scene(r, empty);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_EQ(overlume::testing::point_cloud_mesh_count(r, 0), 0u)
        << "a slot past the new (lower) point_cloud_count must be torn down, not left dangling";

    overlume::destroy_renderer(r);
}

// ── Staleness fade is the ONE shared MaterialInstance's alpha uniform ───────

TEST(PointCloud, MaterialAlphaFollowsStaleness) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<overlume::PointCloudPoint> pts = make_points(10);
    overlume::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = static_cast<uint32_t>(pts.size());
    pc.last_update_sec = 0.0;
    overlume::SceneGraph s{};
    s.sim_time_sec = 0.0;
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    overlume::set_scene(r, s);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_FLOAT_EQ(overlume::testing::point_cloud_material_alpha(r), 1.0f);

    // Past kStaleFadeTimeoutSec (1.0s, renderer_internal.hpp) with no new
    // publish -- render_frame() re-reads the same last-published scene
    // every call (freeze-frame), so sim_time_sec must be re-published to
    // move the clock forward, same convention every other staleness test
    // in this suite uses.
    overlume::SceneGraph stale = s;
    stale.sim_time_sec = 5.0;
    overlume::set_scene(r, stale);
    render_once(r, overlume::CameraPose{{0, -10, 10}, {0, 0, 0}, 60.0});
    EXPECT_FLOAT_EQ(overlume::testing::point_cloud_material_alpha(r), 0.0f);

    overlume::destroy_renderer(r);
}
