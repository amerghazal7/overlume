// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "scene_buffer.hpp"  // -I src, internal header

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

TEST(SceneBuffer, StageThenActive_ReflectsLastPublishedScene) {
    overlume::detail::SceneBuffer buf;
    overlume::SceneGraph g{};
    g.sim_time_sec = 1.0;
    g.ego = {{1, 2, 3}, 0.5, 4.2, /*valid=*/1};
    buf.publish(g);
    const overlume::SceneGraph& active = buf.active();
    EXPECT_EQ(active.ego.valid, 1);
    EXPECT_DOUBLE_EQ(active.ego.position.x, 1.0);
}

TEST(SceneBuffer, DeepCopy_SurvivesCallerBufferReuse) {
    overlume::detail::SceneBuffer buf;
    std::vector<overlume::TrackedObject> objs(1);
    objs[0].id = 7;
    overlume::SceneGraph g{};
    g.objects = objs.data();
    g.object_count = 1;
    buf.publish(g);
    objs[0].id = 999;  // caller mutates its own buffer after publish() returns
    EXPECT_EQ(buf.active().objects[0].id, 7u);  // must have been deep-copied
}

TEST(SceneBuffer, DeepCopy_SurvivesCallerNestedBufferReuse) {
    // The shallow-copy trap: objs[0].id above lives directly in the
    // TrackedObject struct, so even a flat memcpy-style copy passes the test
    // above. predicted_path/label are pointers INTO caller-owned storage the
    // struct doesn't own — this is the case that actually exercises "deep".
    overlume::detail::SceneBuffer buf;
    std::vector<overlume::Vec3> path = {{1, 1, 0}, {2, 2, 0}};
    std::string label = "car-42";
    std::vector<overlume::TrackedObject> objs(1);
    objs[0].id = 7;
    objs[0].predicted_path = path.data();
    objs[0].predicted_path_count = static_cast<uint32_t>(path.size());
    objs[0].label = label.c_str();
    overlume::SceneGraph g{};
    g.objects = objs.data();
    g.object_count = 1;
    buf.publish(g);

    // Caller frees/overwrites its nested buffers after publish() returns —
    // set_scene's frozen contract requires the copy to have taken its own
    // storage for these, not just for the flat TrackedObject array.
    path.assign(2, overlume::Vec3{-9, -9, -9});
    label.assign("OVERWRITTEN");

    const overlume::TrackedObject& active_obj = buf.active().objects[0];
    ASSERT_EQ(active_obj.predicted_path_count, 2u);
    EXPECT_DOUBLE_EQ(active_obj.predicted_path[0].x, 1.0);
    EXPECT_DOUBLE_EQ(active_obj.predicted_path[1].x, 2.0);
    ASSERT_NE(active_obj.label, nullptr);
    EXPECT_STREQ(active_obj.label, "car-42");
}

TEST(SceneBuffer, NoNewPublish_KeepsPreviousActiveScene) {
    overlume::detail::SceneBuffer buf;
    overlume::SceneGraph g{};
    g.sim_time_sec = 5.0;
    buf.publish(g);
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);  // freeze-frame: reading
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);  // again changes nothing
}

TEST(StalenessAlpha, FreshIsFullyOpaque) {
    EXPECT_FLOAT_EQ(overlume::detail::SceneBuffer::staleness_alpha(10.0, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, BeforeFadeStartIsFullyOpaque) {
    EXPECT_FLOAT_EQ(overlume::detail::SceneBuffer::staleness_alpha(10.4, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, MidFadeIsInterpolated) {
    // age=1.25s, fade_start=0.5s, timeout=2.0s -> 50% through the fade window
    EXPECT_NEAR(overlume::detail::SceneBuffer::staleness_alpha(11.25, 10.0, 0.5, 2.0), 0.5f, 1e-6f);
}
TEST(StalenessAlpha, PastTimeoutIsFullyFaded) {
    EXPECT_FLOAT_EQ(overlume::detail::SceneBuffer::staleness_alpha(13.0, 10.0, 0.5, 2.0), 0.0f);
}

static_assert(overlume::kSceneVersion == 6,
              "bump this alongside every additive scene.h change, and update the "
              "node-side test_scene_layout.cpp mirror");

// POD-layout snapshot test: guards scene.h's ADDITIVE-ONLY contract (ADR-0004)
// — catches an accidental member reorder/resize in review, not just a
// missing-field build error. Numbers below were read off the actual compiler
// output on this toolchain (clang/libc++, LP64), not hand-guessed. "layout
// frozen" below is ADR-0004-superseded wording (appending is expected and
// covered by kSceneVersion, not frozen shut) — kept verbatim on pre-existing
// assert messages so a diff against history stays legible; new asserts use
// ADR-0004 phrasing directly.
//
// Every member of every struct below has its own offsetof assert, not just
// sizeof: a same-size member reorder (e.g. swapping two same-width fields)
// would pass a sizeof-only check silently, and the ROS node links a
// PREBUILT liboverlume.a archive against this header, so that would
// be a silent ABI break, not a compile error at the call site. sizeof stays
// too, as the "no trailing padding grew" check offsetof alone doesn't give
// you.
static_assert(sizeof(overlume::Vec3) == 24, "Vec3 layout frozen");
static_assert(offsetof(overlume::Vec3, x) == 0, "Vec3 layout frozen");
static_assert(offsetof(overlume::Vec3, y) == 8, "Vec3 layout frozen");
static_assert(offsetof(overlume::Vec3, z) == 16, "Vec3 layout frozen");

static_assert(sizeof(overlume::EgoState) == 48, "EgoState layout frozen — see Interfaces block");
static_assert(offsetof(overlume::EgoState, position) == 0, "EgoState layout frozen");
static_assert(offsetof(overlume::EgoState, heading_rad) == 24, "EgoState layout frozen");
static_assert(offsetof(overlume::EgoState, speed_mps) == 32, "EgoState layout frozen");
static_assert(offsetof(overlume::EgoState, valid) == 40, "EgoState layout frozen");

static_assert(sizeof(overlume::TrackedObject) == 120, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, id) == 0, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, cls) == 4, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, position) == 8, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, heading_rad) == 32, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, dimensions) == 40, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, velocity) == 64, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, predicted_path) == 88,
              "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, predicted_path_count) == 96,
              "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, label) == 104, "TrackedObject layout frozen");
static_assert(offsetof(overlume::TrackedObject, last_update_sec) == 112,
              "TrackedObject layout frozen");

static_assert(sizeof(overlume::PathRibbon) == 32, "PathRibbon layout frozen");
static_assert(offsetof(overlume::PathRibbon, role) == 0, "PathRibbon layout frozen");
static_assert(offsetof(overlume::PathRibbon, points) == 8, "PathRibbon layout frozen");
static_assert(offsetof(overlume::PathRibbon, point_count) == 16, "PathRibbon layout frozen");
static_assert(offsetof(overlume::PathRibbon, last_update_sec) == 24, "PathRibbon layout frozen");

// kind/lane_id/last_update_sec appended after is_polygon (ADR-0004) -- 16
// -> 32 bytes. "layout frozen" reworded to "ADR-0004 additive": appending
// is expected and covered by kSceneVersion, not frozen shut.
static_assert(sizeof(overlume::MapElement) == 32, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(overlume::MapElement, points) == 0, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(overlume::MapElement, point_count) == 8,
              "MapElement layout, ADR-0004 additive");
static_assert(offsetof(overlume::MapElement, is_polygon) == 12,
              "MapElement layout, ADR-0004 additive");
static_assert(offsetof(overlume::MapElement, kind) == 13, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(overlume::MapElement, lane_id) == 16,
              "MapElement layout, ADR-0004 additive");
static_assert(offsetof(overlume::MapElement, last_update_sec) == 24,
              "MapElement layout, ADR-0004 additive");

static_assert(sizeof(overlume::GroundGridLayer) == 64, "GroundGridLayer layout frozen");
static_assert(offsetof(overlume::GroundGridLayer, kind) == 0, "GroundGridLayer layout frozen");
static_assert(offsetof(overlume::GroundGridLayer, origin) == 8, "GroundGridLayer layout frozen");
static_assert(offsetof(overlume::GroundGridLayer, resolution_m) == 32,
              "GroundGridLayer layout frozen");
static_assert(offsetof(overlume::GroundGridLayer, width_cells) == 40,
              "GroundGridLayer layout frozen");
static_assert(offsetof(overlume::GroundGridLayer, height_cells) == 44,
              "GroundGridLayer layout frozen");
static_assert(offsetof(overlume::GroundGridLayer, cells) == 48, "GroundGridLayer layout frozen");
static_assert(offsetof(overlume::GroundGridLayer, last_update_sec) == 56,
              "GroundGridLayer layout frozen");

static_assert(sizeof(overlume::AlertPolygon) == 24, "AlertPolygon layout frozen");
static_assert(offsetof(overlume::AlertPolygon, points) == 0, "AlertPolygon layout frozen");
static_assert(offsetof(overlume::AlertPolygon, point_count) == 8, "AlertPolygon layout frozen");
static_assert(offsetof(overlume::AlertPolygon, severity) == 12, "AlertPolygon layout frozen");
static_assert(offsetof(overlume::AlertPolygon, last_update_sec) == 16,
              "AlertPolygon layout frozen");

static_assert(sizeof(overlume::GenericMarker) == 120, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, primitive) == 0, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, position) == 8, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, heading_rad) == 32, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, scale) == 40, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, points) == 64, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, point_count) == 72, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, text) == 80, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, mesh_path) == 88, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, color) == 96, "GenericMarker layout frozen");
static_assert(offsetof(overlume::GenericMarker, last_update_sec) == 112,
              "GenericMarker layout frozen");

static_assert(sizeof(overlume::AlertChip) == 32, "AlertChip layout frozen");
static_assert(offsetof(overlume::AlertChip, text) == 0, "AlertChip layout frozen");
static_assert(offsetof(overlume::AlertChip, anchor) == 8, "AlertChip layout frozen");

static_assert(sizeof(overlume::Hud) == 32, "Hud layout frozen");
static_assert(offsetof(overlume::Hud, speed_mps) == 0, "Hud layout frozen");
static_assert(offsetof(overlume::Hud, active_mode) == 8, "Hud layout frozen");
static_assert(offsetof(overlume::Hud, chips) == 16, "Hud layout frozen");
static_assert(offsetof(overlume::Hud, chip_count) == 24, "Hud layout frozen");

// PointCloudPoint/PointCloud, appended Epic 3 Task 6 (VM-035, ADR-0004).
static_assert(sizeof(overlume::PointCloudPoint) == 32, "PointCloudPoint layout, ADR-0004 additive");
static_assert(offsetof(overlume::PointCloudPoint, position) == 0,
              "PointCloudPoint layout, ADR-0004 additive");
static_assert(offsetof(overlume::PointCloudPoint, rgba) == 24,
              "PointCloudPoint layout, ADR-0004 additive");

static_assert(sizeof(overlume::PointCloud) == 24, "PointCloud layout, ADR-0004 additive");
static_assert(offsetof(overlume::PointCloud, points) == 0, "PointCloud layout, ADR-0004 additive");
static_assert(offsetof(overlume::PointCloud, point_count) == 8,
              "PointCloud layout, ADR-0004 additive");
static_assert(offsetof(overlume::PointCloud, last_update_sec) == 16,
              "PointCloud layout, ADR-0004 additive");

static_assert(sizeof(overlume::TrajectoryCarpet) == 24,
              "TrajectoryCarpet layout, ADR-0004 additive");
static_assert(offsetof(overlume::TrajectoryCarpet, points) == 0,
              "TrajectoryCarpet layout, ADR-0004 additive");
static_assert(offsetof(overlume::TrajectoryCarpet, point_count) == 8,
              "TrajectoryCarpet layout, ADR-0004 additive");
static_assert(offsetof(overlume::TrajectoryCarpet, last_update_sec) == 16,
              "TrajectoryCarpet layout, ADR-0004 additive");

static_assert(sizeof(overlume::SceneGraph) == 216, "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, sim_time_sec) == 0,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, ego) == 8, "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, objects) == 56,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, object_count) == 64,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, paths) == 72, "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, path_count) == 80,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, map_elements) == 88,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, map_element_count) == 96,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, grids) == 104, "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, grid_count) == 112,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, alerts) == 120,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, alert_count) == 128,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, markers) == 136,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, marker_count) == 144,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, hud) == 152, "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, point_clouds) == 184,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, point_cloud_count) == 192,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, trajectory_carpets) == 200,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, trajectory_carpet_count) == 208,
              "SceneGraph layout, ADR-0004 additive");

// GeoAnchor, appended VM-050 (Epic 4 Task 1, ADR-0004) -- kSceneVersion
// 3 -> 4. NOT a SceneGraph field (Decision 1) -- standalone POD, three
// doubles, no padding.
static_assert(sizeof(overlume::GeoAnchor) == 24, "GeoAnchor layout, ADR-0004 additive");
static_assert(offsetof(overlume::GeoAnchor, origin_lat_deg) == 0,
              "GeoAnchor layout, ADR-0004 additive");
static_assert(offsetof(overlume::GeoAnchor, origin_lon_deg) == 8,
              "GeoAnchor layout, ADR-0004 additive");
static_assert(offsetof(overlume::GeoAnchor, heading_rad) == 16,
              "GeoAnchor layout, ADR-0004 additive");

// CameraExtrinsics/CameraIntrinsics/BowlConfig, appended VM-090 (unified-
// engine migration Task 1, ADR-0005) -- kSceneVersion 4 -> 5. None are
// SceneGraph fields (same "standalone POD, not deep-copied per-tick" shape
// as GeoAnchor above).
static_assert(sizeof(overlume::CameraExtrinsics) == 96,
              "CameraExtrinsics layout, ADR-0004 additive");
static_assert(offsetof(overlume::CameraExtrinsics, R) == 0,
              "CameraExtrinsics layout, ADR-0004 additive");
static_assert(offsetof(overlume::CameraExtrinsics, t) == 72,
              "CameraExtrinsics layout, ADR-0004 additive");

static_assert(sizeof(overlume::CameraIntrinsics) == 72,
              "CameraIntrinsics layout, ADR-0004 additive");
static_assert(offsetof(overlume::CameraIntrinsics, fx) == 0,
              "CameraIntrinsics layout, ADR-0004 additive");
static_assert(offsetof(overlume::CameraIntrinsics, fy) == 8,
              "CameraIntrinsics layout, ADR-0004 additive");
static_assert(offsetof(overlume::CameraIntrinsics, cx) == 16,
              "CameraIntrinsics layout, ADR-0004 additive");
static_assert(offsetof(overlume::CameraIntrinsics, cy) == 24,
              "CameraIntrinsics layout, ADR-0004 additive");
static_assert(offsetof(overlume::CameraIntrinsics, dist) == 32,
              "CameraIntrinsics layout, ADR-0004 additive");

static_assert(sizeof(overlume::BowlConfig) == 96,
              "BowlConfig layout, ADR-0004 additive (review round 1: +exposure_compensation)");
static_assert(offsetof(overlume::BowlConfig, camera_count) == 0,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, extrinsics) == 8,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, intrinsics) == 16,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, cam_width) == 24,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, cam_height) == 32,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, bowl_R0) == 40,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, bowl_k) == 48, "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, bowl_Rmax) == 56,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, feather_margin) == 64,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, fill_blind_zone) == 72,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, exposure_match) == 73,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, sky_color) == 76,
              "BowlConfig layout, ADR-0004 additive");
static_assert(offsetof(overlume::BowlConfig, exposure_compensation) == 88,
              "BowlConfig layout, ADR-0004 additive (review round 1)");

// EnvironmentSourceState, appended VM-063 (Epic 6 Task 4, ADR-0004) --
// kSceneVersion 5 -> 6. enum class uint8_t, no offsetof members to guard
// (same shape as ObjectClass/MapKind/PathRole above).
static_assert(sizeof(overlume::EnvironmentSourceState) == 1,
              "EnvironmentSourceState layout, ADR-0004 additive");

// RenderConfig (api.h) — also crosses the prebuilt-archive ABI boundary.
static_assert(sizeof(overlume::RenderConfig) == 32, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, width) == 0, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, height) == 4, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, quality) == 8, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, theme_assets_dir) == 16,
              "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, initial_theme) == 24, "RenderConfig layout frozen");

// CameraPose/FrameView (api.h) — gate-verify follow-up: the other two PODs
// that cross the prebuilt-archive ABI boundary (render_frame takes both);
// adjacent same-width members (eye/target, width/height) would swap with no
// sizeof change, so offsetof coverage is the only guard.
static_assert(sizeof(overlume::CameraPose) == 56, "CameraPose layout frozen");
static_assert(offsetof(overlume::CameraPose, eye) == 0, "CameraPose layout frozen");
static_assert(offsetof(overlume::CameraPose, target) == 24, "CameraPose layout frozen");
static_assert(offsetof(overlume::CameraPose, vfov_deg) == 48, "CameraPose layout frozen");
static_assert(sizeof(overlume::FrameView) == 16, "FrameView layout frozen");
static_assert(offsetof(overlume::FrameView, rgb) == 0, "FrameView layout frozen");
static_assert(offsetof(overlume::FrameView, width) == 8, "FrameView layout frozen");
static_assert(offsetof(overlume::FrameView, height) == 12, "FrameView layout frozen");

// MapElement's kind/lane_id/last_update_sec fields survive the deep-copy
// in SceneBuffer::assign() -- no owned-pointer new field means no new
// logic in assign() itself, just the existing memberwise struct copy.
TEST(SceneBufferMapElement, KindLaneIdLastUpdateSecSurviveAssign) {
    overlume::detail::SceneBuffer buf;
    overlume::Vec3 pts[2] = {{0, 0, 0}, {1, 0, 0}};
    overlume::MapElement e{};
    e.points = pts;
    e.point_count = 2;
    e.is_polygon = 0;
    e.kind = overlume::MapKind::CENTERLINE;
    e.lane_id = 934;
    e.last_update_sec = 12.5;
    overlume::SceneGraph s{};
    s.map_elements = &e;
    s.map_element_count = 1;
    buf.publish(s);

    e.kind = overlume::MapKind::OTHER;  // caller mutates its own buffer after publish() returns
    e.lane_id = 0;
    e.last_update_sec = 0.0;

    const overlume::MapElement& active = buf.active().map_elements[0];
    EXPECT_EQ(active.kind, overlume::MapKind::CENTERLINE);
    EXPECT_EQ(active.lane_id, 934u);
    EXPECT_DOUBLE_EQ(active.last_update_sec, 12.5);
}

// PointCloud::points is a caller-owned raw pointer, exactly like
// MapElement::points -- assign()'s `view = src` copies that pointer
// verbatim, so without the point_clouds deep-copy block, active().
// point_clouds[i].points would dangle the instant this test's own `pts`
// array is overwritten below (Task 6 Step 1).
TEST(SceneBufferPointCloud, PointCloudPointsSurviveAssignAfterSourceBufferDies) {
    overlume::detail::SceneBuffer buf;
    std::vector<overlume::PointCloudPoint> pts = {{{0, 0, 0}, 0xFF0000FFu},
                                                  {{1, 0, 0}, 0xFF00FF00u}};
    overlume::PointCloud pc{};
    pc.points = pts.data();
    pc.point_count = 2;
    pc.last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.point_clouds = &pc;
    s.point_cloud_count = 1;
    buf.publish(s);

    std::fill(pts.begin(), pts.end(), overlume::PointCloudPoint{});  // overwrite caller's array

    ASSERT_EQ(buf.active().point_cloud_count, 1u);
    const overlume::PointCloud& active = buf.active().point_clouds[0];
    ASSERT_EQ(active.point_count, 2u);
    EXPECT_DOUBLE_EQ(active.points[0].position.x, 0.0);
    EXPECT_EQ(active.points[0].rgba, 0xFF0000FFu);
    EXPECT_DOUBLE_EQ(active.points[1].position.x, 1.0);
    EXPECT_EQ(active.points[1].rgba, 0xFF00FF00u);
    EXPECT_DOUBLE_EQ(active.last_update_sec, 1.0);
}

// TrajectoryCarpet::points is a caller-owned raw pointer, exactly like
// PointCloud::points -- assign()'s `view = src` copies that pointer
// verbatim, so without the trajectory_carpets deep-copy block, active().
// trajectory_carpets[i].points would dangle the instant this test's own
// `pts` array is overwritten below (VM-077 Task 2 Step 0).
TEST(SceneBufferTrajectoryCarpet, TrajectoryCarpetPointsSurviveAssignAfterSourceBufferDies) {
    overlume::detail::SceneBuffer buf;
    std::vector<overlume::PointCloudPoint> pts = {
        {{0, 0, 0}, 0xFF0000FFu}, {{1, 0, 0}, 0xFF00FF00u}, {{1, 1, 0}, 0x00FF00FFu}};
    overlume::TrajectoryCarpet tc{};
    tc.points = pts.data();
    tc.point_count = 3;
    tc.last_update_sec = 1.0;
    overlume::SceneGraph s{};
    s.trajectory_carpets = &tc;
    s.trajectory_carpet_count = 1;
    buf.publish(s);

    std::fill(pts.begin(), pts.end(), overlume::PointCloudPoint{});  // overwrite caller's array

    ASSERT_EQ(buf.active().trajectory_carpet_count, 1u);
    const overlume::TrajectoryCarpet& active = buf.active().trajectory_carpets[0];
    ASSERT_EQ(active.point_count, 3u);
    EXPECT_DOUBLE_EQ(active.points[0].position.x, 0.0);
    EXPECT_EQ(active.points[0].rgba, 0xFF0000FFu);
    EXPECT_DOUBLE_EQ(active.points[1].position.x, 1.0);
    EXPECT_EQ(active.points[1].rgba, 0xFF00FF00u);
    EXPECT_DOUBLE_EQ(active.points[2].position.y, 1.0);
    EXPECT_EQ(active.points[2].rgba, 0x00FF00FFu);
    EXPECT_DOUBLE_EQ(active.last_update_sec, 1.0);
}
