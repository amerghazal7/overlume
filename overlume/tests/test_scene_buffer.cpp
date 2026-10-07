// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "scene_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

TEST(SceneBuffer, StageThenActive_ReflectsLastPublishedScene) {
    overlume::detail::SceneBuffer buf;
    overlume::SceneGraph g{};
    g.sim_time_sec = 1.0;
    g.ego = {{1, 2, 3}, 0.5, 4.2, 1};
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
    objs[0].id = 999;
    EXPECT_EQ(buf.active().objects[0].id, 7u);
}

TEST(SceneBuffer, DeepCopy_SurvivesCallerNestedBufferReuse) {
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
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);
}

TEST(SceneBuffer, HeightGrid_DeepCopySurvivesSourceMutation) {
    overlume::detail::SceneBuffer buf;
    std::vector<float> heights = {0.0f, 1.0f, std::nanf(""), 3.5f, -2.0f, 0.25f};
    overlume::HeightGridLayer layer{};
    layer.origin = {1.0, 2.0, 3.0};
    layer.yaw_rad = 0.5;
    layer.resolution_m = 0.25;
    layer.width_cells = 3;
    layer.height_cells = 2;
    layer.heights_m = heights.data();
    layer.last_update_sec = 7.0;
    std::vector<overlume::HeightGridLayer> layers = {layer};

    overlume::SceneGraph g{};
    g.height_grids = layers.data();
    g.height_grid_count = 1;
    buf.publish(g);

    // Mutate and free-reuse the caller's buffers: the published copy must not move.
    std::fill(heights.begin(), heights.end(), 99.0f);
    layers[0].width_cells = 1;
    layers[0].yaw_rad = 9.0;

    const overlume::SceneGraph& active = buf.active();
    ASSERT_EQ(active.height_grid_count, 1u);
    const overlume::HeightGridLayer& a = active.height_grids[0];
    ASSERT_NE(a.heights_m, nullptr);
    EXPECT_NE(a.heights_m, heights.data());
    EXPECT_EQ(a.width_cells, 3u);
    EXPECT_EQ(a.height_cells, 2u);
    EXPECT_DOUBLE_EQ(a.yaw_rad, 0.5);
    EXPECT_DOUBLE_EQ(a.resolution_m, 0.25);
    EXPECT_DOUBLE_EQ(a.origin.z, 3.0);
    EXPECT_DOUBLE_EQ(a.last_update_sec, 7.0);
    EXPECT_FLOAT_EQ(a.heights_m[0], 0.0f);
    EXPECT_FLOAT_EQ(a.heights_m[1], 1.0f);
    EXPECT_TRUE(std::isnan(a.heights_m[2]));  // unknown cells survive the copy
    EXPECT_FLOAT_EQ(a.heights_m[3], 3.5f);
    EXPECT_FLOAT_EQ(a.heights_m[4], -2.0f);
    EXPECT_FLOAT_EQ(a.heights_m[5], 0.25f);
}

TEST(SceneBuffer, HeightGrid_NullHeightsOrZeroDimsGiveEmptyCopy) {
    overlume::detail::SceneBuffer buf;
    float one = 1.0f;
    overlume::HeightGridLayer null_heights{};
    null_heights.width_cells = 4;
    null_heights.height_cells = 4;  // heights_m == nullptr
    overlume::HeightGridLayer zero_width{};
    zero_width.width_cells = 0;
    zero_width.height_cells = 4;
    zero_width.heights_m = &one;
    overlume::HeightGridLayer zero_height{};
    zero_height.width_cells = 4;
    zero_height.height_cells = 0;
    zero_height.heights_m = &one;
    std::vector<overlume::HeightGridLayer> layers = {null_heights, zero_width, zero_height};

    overlume::SceneGraph g{};
    g.height_grids = layers.data();
    g.height_grid_count = static_cast<uint32_t>(layers.size());
    buf.publish(g);

    const overlume::SceneGraph& active = buf.active();
    ASSERT_EQ(active.height_grid_count, 3u);
    for (uint32_t i = 0; i < 3; ++i) EXPECT_EQ(active.height_grids[i].heights_m, nullptr) << i;
}

TEST(SceneBuffer, HeightGrid_RepublishReplacesAndClears) {
    overlume::detail::SceneBuffer buf;
    std::vector<float> heights(4, 1.0f);
    overlume::HeightGridLayer layer{};
    layer.width_cells = 2;
    layer.height_cells = 2;
    layer.heights_m = heights.data();
    overlume::SceneGraph g{};
    g.height_grids = &layer;
    g.height_grid_count = 1;
    buf.publish(g);  // first buffer
    heights.assign(4, 2.0f);
    buf.publish(g);  // second buffer
    ASSERT_NE(buf.active().height_grids[0].heights_m, nullptr);
    EXPECT_FLOAT_EQ(buf.active().height_grids[0].heights_m[0], 2.0f);

    // Third publish reuses the first buffer with a null-heights layer: the view must
    // report a null pointer and count 1 (slot reuse must not resurrect the earlier copy).
    overlume::HeightGridLayer bare{};
    bare.width_cells = 2;
    bare.height_cells = 2;  // heights_m == nullptr
    overlume::SceneGraph g3{};
    g3.height_grids = &bare;
    g3.height_grid_count = 1;
    buf.publish(g3);
    ASSERT_EQ(buf.active().height_grid_count, 1u);
    EXPECT_EQ(buf.active().height_grids[0].heights_m, nullptr);

    // And an empty scene reuses the second buffer: the count must read zero.
    overlume::SceneGraph empty{};
    buf.publish(empty);
    EXPECT_EQ(buf.active().height_grid_count, 0u);
}

TEST(StalenessAlpha, FreshIsFullyOpaque) {
    EXPECT_FLOAT_EQ(overlume::detail::SceneBuffer::staleness_alpha(10.0, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, BeforeFadeStartIsFullyOpaque) {
    EXPECT_FLOAT_EQ(overlume::detail::SceneBuffer::staleness_alpha(10.4, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, MidFadeIsInterpolated) {
    EXPECT_NEAR(overlume::detail::SceneBuffer::staleness_alpha(11.25, 10.0, 0.5, 2.0), 0.5f, 1e-6f);
}
TEST(StalenessAlpha, PastTimeoutIsFullyFaded) {
    EXPECT_FLOAT_EQ(overlume::detail::SceneBuffer::staleness_alpha(13.0, 10.0, 0.5, 2.0), 0.0f);
}

static_assert(overlume::kSceneVersion == 9,
              "bump this alongside every additive scene.h change, and update the "
              "node-side test_scene_layout.cpp mirror");

// The frozen layouts are the LP64 (ROS node) ABI; 32-bit Android ABIs (armeabi-v7a, x86) have
// 4-byte pointers and 4-byte-aligned doubles (x86), so the offsets differ there by design.
#if INTPTR_MAX == INT64_MAX
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

static_assert(sizeof(overlume::GroundGridLayer) == 72, "GroundGridLayer layout frozen");
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
static_assert(offsetof(overlume::GroundGridLayer, yaw_rad) == 64, "GroundGridLayer layout frozen");

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

static_assert(sizeof(overlume::HeightGridLayer) == 64, "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, origin) == 0,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, yaw_rad) == 24,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, resolution_m) == 32,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, width_cells) == 40,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, height_cells) == 44,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, heights_m) == 48,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, last_update_sec) == 56,
              "HeightGridLayer layout, ADR-0004 additive");

static_assert(sizeof(overlume::SceneGraph) == 232, "SceneGraph layout, ADR-0004 additive");
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
static_assert(offsetof(overlume::SceneGraph, height_grids) == 216,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, height_grid_count) == 224,
              "SceneGraph layout, ADR-0004 additive");

static_assert(sizeof(overlume::GeoAnchor) == 32, "GeoAnchor layout, ADR-0004 additive");
static_assert(offsetof(overlume::GeoAnchor, origin_lat_deg) == 0,
              "GeoAnchor layout, ADR-0004 additive");
static_assert(offsetof(overlume::GeoAnchor, origin_lon_deg) == 8,
              "GeoAnchor layout, ADR-0004 additive");
static_assert(offsetof(overlume::GeoAnchor, heading_rad) == 16,
              "GeoAnchor layout, ADR-0004 additive");
static_assert(offsetof(overlume::GeoAnchor, origin_height_m) == 24,
              "GeoAnchor layout, ADR-0004 additive");

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

static_assert(sizeof(overlume::EnvironmentSourceState) == 1,
              "EnvironmentSourceState layout, ADR-0004 additive");

static_assert(sizeof(overlume::RenderConfig) == 32, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, width) == 0, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, height) == 4, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, quality) == 8, "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, theme_assets_dir) == 16,
              "RenderConfig layout frozen");
static_assert(offsetof(overlume::RenderConfig, initial_theme) == 24, "RenderConfig layout frozen");

static_assert(sizeof(overlume::CameraPose) == 56, "CameraPose layout frozen");
static_assert(offsetof(overlume::CameraPose, eye) == 0, "CameraPose layout frozen");
static_assert(offsetof(overlume::CameraPose, target) == 24, "CameraPose layout frozen");
static_assert(offsetof(overlume::CameraPose, vfov_deg) == 48, "CameraPose layout frozen");
static_assert(sizeof(overlume::FrameView) == 16, "FrameView layout frozen");
static_assert(offsetof(overlume::FrameView, rgb) == 0, "FrameView layout frozen");
static_assert(offsetof(overlume::FrameView, width) == 8, "FrameView layout frozen");
static_assert(offsetof(overlume::FrameView, height) == 12, "FrameView layout frozen");
#endif  // LP64

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

    e.kind = overlume::MapKind::OTHER;
    e.lane_id = 0;
    e.last_update_sec = 0.0;

    const overlume::MapElement& active = buf.active().map_elements[0];
    EXPECT_EQ(active.kind, overlume::MapKind::CENTERLINE);
    EXPECT_EQ(active.lane_id, 934u);
    EXPECT_DOUBLE_EQ(active.last_update_sec, 12.5);
}

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

    std::fill(pts.begin(), pts.end(), overlume::PointCloudPoint{});

    ASSERT_EQ(buf.active().point_cloud_count, 1u);
    const overlume::PointCloud& active = buf.active().point_clouds[0];
    ASSERT_EQ(active.point_count, 2u);
    EXPECT_DOUBLE_EQ(active.points[0].position.x, 0.0);
    EXPECT_EQ(active.points[0].rgba, 0xFF0000FFu);
    EXPECT_DOUBLE_EQ(active.points[1].position.x, 1.0);
    EXPECT_EQ(active.points[1].rgba, 0xFF00FF00u);
    EXPECT_DOUBLE_EQ(active.last_update_sec, 1.0);
}

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

    std::fill(pts.begin(), pts.end(), overlume::PointCloudPoint{});

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
