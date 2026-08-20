#include "scene_buffer.hpp"   // -I src, internal header

#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

TEST(SceneBuffer, StageThenActive_ReflectsLastPublishedScene) {
    mpviz::detail::SceneBuffer buf;
    mpviz::SceneGraph g{};
    g.sim_time_sec = 1.0;
    g.ego = {{1, 2, 3}, 0.5, 4.2, /*valid=*/1};
    buf.publish(g);
    const mpviz::SceneGraph& active = buf.active();
    EXPECT_EQ(active.ego.valid, 1);
    EXPECT_DOUBLE_EQ(active.ego.position.x, 1.0);
}

TEST(SceneBuffer, DeepCopy_SurvivesCallerBufferReuse) {
    mpviz::detail::SceneBuffer buf;
    std::vector<mpviz::TrackedObject> objs(1);
    objs[0].id = 7;
    mpviz::SceneGraph g{};
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
    mpviz::detail::SceneBuffer buf;
    std::vector<mpviz::Vec3> path = {{1, 1, 0}, {2, 2, 0}};
    std::string label = "car-42";
    std::vector<mpviz::TrackedObject> objs(1);
    objs[0].id = 7;
    objs[0].predicted_path = path.data();
    objs[0].predicted_path_count = static_cast<uint32_t>(path.size());
    objs[0].label = label.c_str();
    mpviz::SceneGraph g{};
    g.objects = objs.data();
    g.object_count = 1;
    buf.publish(g);

    // Caller frees/overwrites its nested buffers after publish() returns —
    // set_scene's frozen contract requires the copy to have taken its own
    // storage for these, not just for the flat TrackedObject array.
    path.assign(2, mpviz::Vec3{-9, -9, -9});
    label.assign("OVERWRITTEN");

    const mpviz::TrackedObject& active_obj = buf.active().objects[0];
    ASSERT_EQ(active_obj.predicted_path_count, 2u);
    EXPECT_DOUBLE_EQ(active_obj.predicted_path[0].x, 1.0);
    EXPECT_DOUBLE_EQ(active_obj.predicted_path[1].x, 2.0);
    ASSERT_NE(active_obj.label, nullptr);
    EXPECT_STREQ(active_obj.label, "car-42");
}

TEST(SceneBuffer, NoNewPublish_KeepsPreviousActiveScene) {
    mpviz::detail::SceneBuffer buf;
    mpviz::SceneGraph g{};
    g.sim_time_sec = 5.0;
    buf.publish(g);
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);  // freeze-frame: reading
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);  // again changes nothing
}

TEST(StalenessAlpha, FreshIsFullyOpaque) {
    EXPECT_FLOAT_EQ(mpviz::detail::SceneBuffer::staleness_alpha(10.0, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, BeforeFadeStartIsFullyOpaque) {
    EXPECT_FLOAT_EQ(mpviz::detail::SceneBuffer::staleness_alpha(10.4, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, MidFadeIsInterpolated) {
    // age=1.25s, fade_start=0.5s, timeout=2.0s -> 50% through the fade window
    EXPECT_NEAR(mpviz::detail::SceneBuffer::staleness_alpha(11.25, 10.0, 0.5, 2.0), 0.5f, 1e-6f);
}
TEST(StalenessAlpha, PastTimeoutIsFullyFaded) {
    EXPECT_FLOAT_EQ(mpviz::detail::SceneBuffer::staleness_alpha(13.0, 10.0, 0.5, 2.0), 0.0f);
}

// POD-layout snapshot test: guards the scene.h freeze itself — catches an
// accidental member reorder/resize in review, not just a missing-field build
// error. Numbers below were read off the actual compiler output on this
// toolchain (clang/libc++, LP64), not hand-guessed.
//
// Epic1 Task2/3 review gate (MAJOR 2): the original version of this block
// only sizeof-checked TrackedObject/PathRibbon/MapElement/GroundGridLayer/
// AlertPolygon/GenericMarker/AlertChip/Hud (no offsetof for any of their
// members) and left 9 of SceneGraph's 15 members' offsets unchecked — a
// same-size member reorder inside any of those structs (e.g. swapping two
// same-width fields) would pass every sizeof-only assert here silently, and
// the ROS node links a PREBUILT libvisual_renderer.a archive against this
// header, so that would be a silent ABI break, not a compile error at the
// call site. Every member of every struct below now has its own offsetof
// assert; sizeof stays too as the "no trailing padding grew" check offsetof
// alone doesn't give you.
static_assert(sizeof(mpviz::Vec3) == 24, "Vec3 layout frozen");
static_assert(offsetof(mpviz::Vec3, x) == 0, "Vec3 layout frozen");
static_assert(offsetof(mpviz::Vec3, y) == 8, "Vec3 layout frozen");
static_assert(offsetof(mpviz::Vec3, z) == 16, "Vec3 layout frozen");

static_assert(sizeof(mpviz::EgoState) == 48, "EgoState layout frozen — see Interfaces block");
static_assert(offsetof(mpviz::EgoState, position) == 0, "EgoState layout frozen");
static_assert(offsetof(mpviz::EgoState, heading_rad) == 24, "EgoState layout frozen");
static_assert(offsetof(mpviz::EgoState, speed_mps) == 32, "EgoState layout frozen");
static_assert(offsetof(mpviz::EgoState, valid) == 40, "EgoState layout frozen");

static_assert(sizeof(mpviz::TrackedObject) == 120, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, id) == 0, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, cls) == 4, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, position) == 8, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, heading_rad) == 32, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, dimensions) == 40, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, velocity) == 64, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, predicted_path) == 88, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, predicted_path_count) == 96,
              "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, label) == 104, "TrackedObject layout frozen");
static_assert(offsetof(mpviz::TrackedObject, last_update_sec) == 112,
              "TrackedObject layout frozen");

static_assert(sizeof(mpviz::PathRibbon) == 32, "PathRibbon layout frozen");
static_assert(offsetof(mpviz::PathRibbon, role) == 0, "PathRibbon layout frozen");
static_assert(offsetof(mpviz::PathRibbon, points) == 8, "PathRibbon layout frozen");
static_assert(offsetof(mpviz::PathRibbon, point_count) == 16, "PathRibbon layout frozen");
static_assert(offsetof(mpviz::PathRibbon, last_update_sec) == 24, "PathRibbon layout frozen");

static_assert(sizeof(mpviz::MapElement) == 16, "MapElement layout frozen");
static_assert(offsetof(mpviz::MapElement, points) == 0, "MapElement layout frozen");
static_assert(offsetof(mpviz::MapElement, point_count) == 8, "MapElement layout frozen");
static_assert(offsetof(mpviz::MapElement, is_polygon) == 12, "MapElement layout frozen");

static_assert(sizeof(mpviz::GroundGridLayer) == 64, "GroundGridLayer layout frozen");
static_assert(offsetof(mpviz::GroundGridLayer, kind) == 0, "GroundGridLayer layout frozen");
static_assert(offsetof(mpviz::GroundGridLayer, origin) == 8, "GroundGridLayer layout frozen");
static_assert(offsetof(mpviz::GroundGridLayer, resolution_m) == 32,
              "GroundGridLayer layout frozen");
static_assert(offsetof(mpviz::GroundGridLayer, width_cells) == 40,
              "GroundGridLayer layout frozen");
static_assert(offsetof(mpviz::GroundGridLayer, height_cells) == 44,
              "GroundGridLayer layout frozen");
static_assert(offsetof(mpviz::GroundGridLayer, cells) == 48, "GroundGridLayer layout frozen");
static_assert(offsetof(mpviz::GroundGridLayer, last_update_sec) == 56,
              "GroundGridLayer layout frozen");

static_assert(sizeof(mpviz::AlertPolygon) == 24, "AlertPolygon layout frozen");
static_assert(offsetof(mpviz::AlertPolygon, points) == 0, "AlertPolygon layout frozen");
static_assert(offsetof(mpviz::AlertPolygon, point_count) == 8, "AlertPolygon layout frozen");
static_assert(offsetof(mpviz::AlertPolygon, severity) == 12, "AlertPolygon layout frozen");
static_assert(offsetof(mpviz::AlertPolygon, last_update_sec) == 16, "AlertPolygon layout frozen");

static_assert(sizeof(mpviz::GenericMarker) == 120, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, primitive) == 0, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, position) == 8, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, heading_rad) == 32, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, scale) == 40, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, points) == 64, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, point_count) == 72, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, text) == 80, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, mesh_path) == 88, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, color) == 96, "GenericMarker layout frozen");
static_assert(offsetof(mpviz::GenericMarker, last_update_sec) == 112,
              "GenericMarker layout frozen");

static_assert(sizeof(mpviz::AlertChip) == 32, "AlertChip layout frozen");
static_assert(offsetof(mpviz::AlertChip, text) == 0, "AlertChip layout frozen");
static_assert(offsetof(mpviz::AlertChip, anchor) == 8, "AlertChip layout frozen");

static_assert(sizeof(mpviz::Hud) == 32, "Hud layout frozen");
static_assert(offsetof(mpviz::Hud, speed_mps) == 0, "Hud layout frozen");
static_assert(offsetof(mpviz::Hud, active_mode) == 8, "Hud layout frozen");
static_assert(offsetof(mpviz::Hud, chips) == 16, "Hud layout frozen");
static_assert(offsetof(mpviz::Hud, chip_count) == 24, "Hud layout frozen");

static_assert(sizeof(mpviz::SceneGraph) == 184, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, sim_time_sec) == 0, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, ego) == 8, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, objects) == 56, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, object_count) == 64, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, paths) == 72, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, path_count) == 80, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, map_elements) == 88, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, map_element_count) == 96, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, grids) == 104, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, grid_count) == 112, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, alerts) == 120, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, alert_count) == 128, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, markers) == 136, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, marker_count) == 144, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, hud) == 152, "SceneGraph layout frozen");

// RenderConfig (api.h) — same review finding: not covered at all before.
static_assert(sizeof(mpviz::RenderConfig) == 32, "RenderConfig layout frozen");
static_assert(offsetof(mpviz::RenderConfig, width) == 0, "RenderConfig layout frozen");
static_assert(offsetof(mpviz::RenderConfig, height) == 4, "RenderConfig layout frozen");
static_assert(offsetof(mpviz::RenderConfig, quality) == 8, "RenderConfig layout frozen");
static_assert(offsetof(mpviz::RenderConfig, theme_assets_dir) == 16, "RenderConfig layout frozen");
static_assert(offsetof(mpviz::RenderConfig, initial_theme) == 24, "RenderConfig layout frozen");

// CameraPose/FrameView (api.h) — gate-verify follow-up: the other two PODs
// that cross the prebuilt-archive ABI boundary (render_frame takes both);
// adjacent same-width members (eye/target, width/height) would swap with no
// sizeof change, so offsetof coverage is the only guard.
static_assert(sizeof(mpviz::CameraPose) == 56, "CameraPose layout frozen");
static_assert(offsetof(mpviz::CameraPose, eye) == 0, "CameraPose layout frozen");
static_assert(offsetof(mpviz::CameraPose, target) == 24, "CameraPose layout frozen");
static_assert(offsetof(mpviz::CameraPose, vfov_deg) == 48, "CameraPose layout frozen");
static_assert(sizeof(mpviz::FrameView) == 16, "FrameView layout frozen");
static_assert(offsetof(mpviz::FrameView, rgb) == 0, "FrameView layout frozen");
static_assert(offsetof(mpviz::FrameView, width) == 8, "FrameView layout frozen");
static_assert(offsetof(mpviz::FrameView, height) == 12, "FrameView layout frozen");
