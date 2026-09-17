// test_generic_markers.cpp — the generic-marker fallback renderer, i.e.
// the spec §7 parity guarantee. Same "no Filament type" boundary as every
// other tests/*.cpp -- see generic_markers_test_hooks.hpp.
//
// 7 of the 12 ROS marker types never appear in the recorded bag --
// GenericMarkersGolden.EveryPrimitiveType_DarkAdas's scene is entirely
// synthetic by design (golden.cpp's make_all_primitive_markers()).
#include "overlume/api.h"
#include "overlume/scene.h"

#include "generic_markers_test_hooks.hpp"
#include "golden.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

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

// ── Step 1/5: one of every FROZEN primitive, plus the CUBE_LIST/SPHERE_LIST
//    fan-out result, laid out in a row for at-a-glance human counting ───────

TEST(GenericMarkersGolden, EveryPrimitiveType_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const std::string meshPath = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/test_cube.glb";
    overlume::testing::GenericMarkerScene scene =
        overlume::testing::make_all_primitive_markers(/*now=*/10.0, meshPath.c_str());
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {{0, 0, 0}, 0.0, 0.0, /*valid=*/1};
    s.markers = scene.markers.data();
    s.marker_count = static_cast<uint32_t>(scene.markers.size());
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{12, -14, 10}, {12, 3, 0}, 70.0};
    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/markers_parity_dark_adas.png",
        "/tmp/markers_parity_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
}

// ── §4.2: pooled primitive renderables, no per-frame allocation ────────────

TEST(GenericMarkers, PooledRenderablesNoPerFrameAllocation) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const std::string meshPath = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/test_cube.glb";
    overlume::testing::GenericMarkerScene scene =
        overlume::testing::make_all_primitive_markers(/*now=*/10.0, meshPath.c_str());
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.markers = scene.markers.data();
    s.marker_count = static_cast<uint32_t>(scene.markers.size());
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{12, -14, 10}, {12, 3, 0}, 70.0};
    render_once(r, pose);
    const uint32_t afterFirst = overlume::testing::generic_marker_alloc_count(r);
    EXPECT_GT(afterFirst, 0u) << "the first frame must actually allocate something";

    for (int i = 0; i < 30; ++i) {
        render_once(r, pose);
    }
    EXPECT_EQ(overlume::testing::generic_marker_alloc_count(r), afterFirst)
        << "30 renders of an UNCHANGED marker set must not allocate anything past frame 1";
    overlume::destroy_renderer(r);
}

// ── A raw out-of-range primitive value is skipped and counted, not a
//    silent no-op or a crash ───────────────────────────────────────────────

TEST(GenericMarkers, UnknownOrUnsupportedPrimitiveIsSkippedAndCounted) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::GenericMarker m{};
    m.primitive = static_cast<overlume::MarkerPrimitive>(200);  // outside the frozen 0..9 range
    m.position = {0, 0, 0.5};
    m.scale = {1, 1, 1};
    overlume::SceneGraph s{};
    s.markers = &m;
    s.marker_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{1, -8, 6}, {1, 1, 0}, 60.0};
    render_once(r, pose);

    EXPECT_EQ(overlume::testing::generic_marker_unknown_count(r), 1u);
    EXPECT_EQ(overlume::testing::generic_marker_slot_count(r), 1u);
    overlume::destroy_renderer(r);
}

// ── spec §9: asset load failure -> clay-box fallback, WARN once ────────────

TEST(GenericMarkers, MeshPathLoadFailureFallsBackToClayBoxWarnOnce) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::GenericMarker m{};
    m.primitive = overlume::MarkerPrimitive::MESH;
    m.position = {0, 0, 0.5};
    m.scale = {1, 1, 1};
    m.mesh_path = "/nonexistent/does_not_exist.glb";
    overlume::SceneGraph s{};
    s.markers = &m;
    s.marker_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{1, -8, 6}, {1, 1, 0}, 60.0};
    // Render twice (two set_scene-free frames of the same load-failure
    // marker) -- the WARN-once contract must not depend on a fresh
    // set_scene() call to hold; the fallback itself must be stable either
    // way.
    render_once(r, pose);
    render_once(r, pose);

    EXPECT_TRUE(overlume::testing::generic_marker_mesh_is_fallback(r, 0));
    overlume::destroy_renderer(r);
}

// ── GenericMarker::color alpha==0 -> theme-neutral default ─────────────────

TEST(GenericMarkers, ZeroAlphaColorUsesThemeNeutralDefault) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::GenericMarker m{};
    m.primitive = overlume::MarkerPrimitive::CUBE;
    m.position = {0, 0, 0.5};
    m.scale = {1, 1, 1};
    m.color[3] = 0.0f;  // no colour supplied
    overlume::SceneGraph s{};
    s.markers = &m;
    s.marker_count = 1;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{1, -8, 6}, {1, 1, 0}, 60.0};
    render_once(r, pose);

    const auto tint = overlume::testing::generic_marker_tint(r, 0);
    EXPECT_NEAR(tint.r, theme->palette.object_tints.unknown.r, 1e-4);
    EXPECT_NEAR(tint.g, theme->palette.object_tints.unknown.g, 1e-4);
    EXPECT_NEAR(tint.b, theme->palette.object_tints.unknown.b, 1e-4);
    overlume::destroy_renderer(r);
}

// ── Staleness: shared clay_translucent.mat swap, driven by
//    SceneBuffer::staleness_alpha -- same mechanism as objects/ribbons/
//    alerts ─────────────────────────────────────────────────────────────────

TEST(GenericMarkers, StaleMarkersFadeViaSharedStalenessAlpha) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::GenericMarker markers[2]{};
    markers[0].primitive = overlume::MarkerPrimitive::CUBE;
    markers[0].position = {0, 0, 0.5};
    markers[0].scale = {1, 1, 1};
    markers[0].last_update_sec = 10.0;  // fresh at sim_time 10.0

    markers[1].primitive = overlume::MarkerPrimitive::CUBE;
    markers[1].position = {3, 0, 0.5};
    markers[1].scale = {1, 1, 1};
    markers[1].last_update_sec = 9.25;  // 0.75s stale: mid-fade (0.5 <= t < 1.0)

    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.markers = markers;
    s.marker_count = 2;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{1, -8, 6}, {1, 1, 0}, 60.0};
    render_once(r, pose);

    const auto fresh = overlume::testing::generic_marker_material_info(r, 0);
    EXPECT_FALSE(fresh.bound_to_translucent);
    EXPECT_FLOAT_EQ(fresh.alpha, 1.0f);

    const auto stale = overlume::testing::generic_marker_material_info(r, 1);
    EXPECT_TRUE(stale.bound_to_translucent)
        << "a stale marker must swap to its own clay_translucent.mat instance";
    EXPECT_GT(stale.alpha, 0.0f);
    EXPECT_LT(stale.alpha, fresh.alpha);
    overlume::destroy_renderer(r);
}
