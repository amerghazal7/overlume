/** @file test_scene_layout.cpp
 *  @brief Node-side (gcc/libstdc++) mirror of tests/test_scene_buffer.cpp's
 *  scene.h layout static_asserts. ADR-0004
 *  (docs/adr/0004-scene-interface-versioning.md) requires kSceneVersion to
 *  be bumped alongside every additive scene.h change, guarded by
 *  sizeof/offsetof static_asserts on both toolchains --
 *  the library's own test_scene_buffer.cpp only compiles clang/libc++; this
 *  file is the gcc/libstdc++ half, so a layout mismatch between the two
 *  toolchains fails THIS build too, not only the library's, instead of
 *  silently reading garbage across the prebuilt-archive ABI boundary the
 *  next time the node links a stale libvisual_renderer.a.
 *
 *  Numbers here are NOT re-derived independently -- they must stay in
 *  lockstep with tests/test_scene_buffer.cpp in overlume;
 *  a change to one without the other is exactly the drift this file exists
 *  to catch.
 */
#include "visual_renderer/scene.h"

#include <cstddef>

#include <gtest/gtest.h>

static_assert(mpviz::kSceneVersion == 6, "node/library scene.h version drifted");

static_assert(sizeof(mpviz::MapElement) == 32, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, points) == 0, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, point_count) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, is_polygon) == 12,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, kind) == 13, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, lane_id) == 16, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, last_update_sec) == 24,
              "node/library scene.h version drifted");

// PointCloudPoint/PointCloud + SceneGraph::point_clouds/point_cloud_count,
// appended Epic 3 Task 6 (VM-035, ADR-0004) -- kSceneVersion 1 -> 2.
static_assert(sizeof(mpviz::PointCloudPoint) == 32, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::PointCloudPoint, position) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::PointCloudPoint, rgba) == 24,
              "node/library scene.h version drifted");

static_assert(sizeof(mpviz::PointCloud) == 24, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::PointCloud, points) == 0, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::PointCloud, point_count) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::PointCloud, last_update_sec) == 16,
              "node/library scene.h version drifted");

static_assert(sizeof(mpviz::SceneGraph) == 216, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::SceneGraph, point_clouds) == 184,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::SceneGraph, point_cloud_count) == 192,
              "node/library scene.h version drifted");

// TrajectoryCarpet + SceneGraph::trajectory_carpets/trajectory_carpet_count,
// appended VM-077 (ADR-0004) -- kSceneVersion 2 -> 3.
static_assert(sizeof(mpviz::TrajectoryCarpet) == 24, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::TrajectoryCarpet, points) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::TrajectoryCarpet, point_count) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::TrajectoryCarpet, last_update_sec) == 16,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::SceneGraph, trajectory_carpets) == 200,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::SceneGraph, trajectory_carpet_count) == 208,
              "node/library scene.h version drifted");

// GeoAnchor, appended VM-050 (Epic 4 Task 1, ADR-0004) -- kSceneVersion
// 3 -> 4. NOT a SceneGraph field (Decision 1) -- standalone POD.
static_assert(sizeof(mpviz::GeoAnchor) == 24, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::GeoAnchor, origin_lat_deg) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::GeoAnchor, origin_lon_deg) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::GeoAnchor, heading_rad) == 16,
              "node/library scene.h version drifted");

// CameraExtrinsics/CameraIntrinsics/BowlConfig, appended VM-090 (unified-
// engine migration Task 1, ADR-0005) -- kSceneVersion 4 -> 5. Not
// SceneGraph fields, same reasoning as GeoAnchor above.
static_assert(sizeof(mpviz::CameraExtrinsics) == 96, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::CameraExtrinsics, R) == 0, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::CameraExtrinsics, t) == 72, "node/library scene.h version drifted");

static_assert(sizeof(mpviz::CameraIntrinsics) == 72, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::CameraIntrinsics, fx) == 0, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::CameraIntrinsics, fy) == 8, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::CameraIntrinsics, cx) == 16, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::CameraIntrinsics, cy) == 24, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::CameraIntrinsics, dist) == 32,
              "node/library scene.h version drifted");

static_assert(sizeof(mpviz::BowlConfig) == 96, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, camera_count) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, extrinsics) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, intrinsics) == 16,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, cam_width) == 24,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, cam_height) == 32,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, bowl_R0) == 40,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, bowl_k) == 48,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, bowl_Rmax) == 56,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, feather_margin) == 64,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, fill_blind_zone) == 72,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, exposure_match) == 73,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, sky_color) == 76,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::BowlConfig, exposure_compensation) == 88,
              "node/library scene.h version drifted");

// EnvironmentSourceState, appended VM-063 (Epic 6 Task 4, ADR-0004) --
// kSceneVersion 5 -> 6.
static_assert(sizeof(mpviz::EnvironmentSourceState) == 1,
              "node/library scene.h version drifted");

// static_asserts above do the real work; this TEST body only exists so
// ament_add_gtest/ctest has something runnable to report.
TEST(SceneLayout, Placeholder) {}
