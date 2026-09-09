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
 *  lockstep with tests/test_scene_buffer.cpp in cuda/src/libs/visual_renderer;
 *  a change to one without the other is exactly the drift this file exists
 *  to catch.
 */
#include "visual_renderer/scene.h"

#include <cstddef>

#include <gtest/gtest.h>

static_assert(mpviz::kSceneVersion == 2, "node/library scene.h version drifted");

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

static_assert(sizeof(mpviz::SceneGraph) == 200, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::SceneGraph, point_clouds) == 184,
              "node/library scene.h version drifted");
static_assert(offsetof(mpviz::SceneGraph, point_cloud_count) == 192,
              "node/library scene.h version drifted");

// static_asserts above do the real work; this TEST body only exists so
// ament_add_gtest/ctest has something runnable to report.
TEST(SceneLayout, Placeholder) {}
