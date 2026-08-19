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
static_assert(sizeof(mpviz::Vec3) == 24, "Vec3 layout frozen");
static_assert(sizeof(mpviz::EgoState) == 48, "EgoState layout frozen — see Interfaces block");
static_assert(offsetof(mpviz::EgoState, position) == 0, "EgoState layout frozen");
static_assert(offsetof(mpviz::EgoState, heading_rad) == 24, "EgoState layout frozen");
static_assert(offsetof(mpviz::EgoState, speed_mps) == 32, "EgoState layout frozen");
static_assert(offsetof(mpviz::EgoState, valid) == 40, "EgoState layout frozen");

static_assert(sizeof(mpviz::TrackedObject) == 120, "TrackedObject layout frozen");
static_assert(sizeof(mpviz::PathRibbon) == 32, "PathRibbon layout frozen");
static_assert(sizeof(mpviz::MapElement) == 16, "MapElement layout frozen");
static_assert(sizeof(mpviz::GroundGridLayer) == 64, "GroundGridLayer layout frozen");
static_assert(sizeof(mpviz::AlertPolygon) == 24, "AlertPolygon layout frozen");
static_assert(sizeof(mpviz::GenericMarker) == 120, "GenericMarker layout frozen");
static_assert(sizeof(mpviz::AlertChip) == 32, "AlertChip layout frozen");
static_assert(sizeof(mpviz::Hud) == 32, "Hud layout frozen");

static_assert(offsetof(mpviz::SceneGraph, sim_time_sec) == 0, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, ego) == 8, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, objects) == 56, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, object_count) == 64, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, paths) == 72, "SceneGraph layout frozen");
static_assert(offsetof(mpviz::SceneGraph, hud) == 152, "SceneGraph layout frozen");
static_assert(sizeof(mpviz::SceneGraph) == 184, "SceneGraph layout frozen");
