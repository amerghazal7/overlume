// test_objects.cpp — clay object rendering. Same "no Filament type"
// boundary as every other tests/*.cpp — see objects_test_hooks.hpp /
// map_elements_test_hooks.hpp / ego_test_hooks.hpp.
#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "objects_test_hooks.hpp"
#include "test_paths.hpp"
#include "theme.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

const overlume::CameraPose kPose{{-14, -14, 8}, {0, 0, 0}, 60.0};

void render_once(overlume::VisualRenderer* r, const overlume::CameraPose& pose) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
}

overlume::TrackedObject make_car(uint32_t id, double x) {
    overlume::TrackedObject o{};
    o.id = id;
    o.cls = overlume::ObjectClass::CAR;
    o.position = {x, 0.0, 0.0};
    o.dimensions = {4.5, 1.8, 1.5};
    o.last_update_sec = 10.0;
    return o;
}

}  // namespace

// ── Step 2: the mixed-class golden (no committed golden yet -- SSIM 0,
// left red for human promotion; /tmp/objects_mixed_dark_adas_actual.png is
// the viewable PNG for that review) ─────────────────────────────────────

TEST(ObjectsGolden, MixedClassScene_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::set_object_model_dir(r, OVERLUME_TEST_DATA_DIR "/assets/models");  // 0 is fine

    overlume::testing::ObjectScene objs = overlume::testing::make_mixed_class_objects(/*now=*/10.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {{0, 0, 0}, 0, 0, 1};
    s.objects = objs.objects.data();
    s.object_count = static_cast<uint32_t>(objs.objects.size());
    overlume::set_scene(r, s);

    double ssim = overlume::testing::render_and_compare(
        r, kPose, OVERLUME_TEST_DATA_DIR "/tests/goldens/objects_mixed_dark_adas.png",
        "/tmp/objects_mixed_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    overlume::destroy_renderer(r);
}

// ── Staleness fade: shared clay_translucent.mat swap, CPU-mirrored alpha ──

TEST(Objects, StaleObjectFadesViaSharedStalenessAlpha) {
    // objects_overrange_opacity clamps to EXACTLY 1.0 -- the fresh-opaque
    // invariant this test pins holds only at opacity 1.0 (the shipped
    // themes author 0.5 since 2026-09-11, under which fresh objects bind
    // translucent by design -- covered by the HalfOpacity tests).
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "objects_overrange_opacity"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::TrackedObject objs[2]{};
    objs[0] = make_car(1, 0.0);   // fresh
    objs[1].id = 2;
    objs[1].cls = overlume::ObjectClass::TRUCK_VAN;
    objs[1].position = {10.0, 0.0, 0.0};
    objs[1].dimensions = {5.5, 2.0, 2.2};
    objs[1].last_update_sec = 10.0 - 0.75;  // 0.75s behind -> alpha ~0.5

    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.objects = objs;
    s.object_count = 2;
    overlume::set_scene(r, s);
    render_once(r, kPose);

    const auto fresh = overlume::testing::object_material_info(r, 1);
    EXPECT_FALSE(fresh.bound_to_translucent) << "a FRESH object must stay on the opaque shared "
                                                 "template, not get a per-entity instance";
    EXPECT_NEAR(fresh.alpha, 1.0f, 1e-4);

    const auto stale = overlume::testing::object_material_info(r, 2);
    EXPECT_TRUE(stale.bound_to_translucent)
        << "a stale object's renderable must be bound to clay_translucent.mat, not clay.mat";
    EXPECT_NEAR(stale.alpha, 0.5f, 0.02f);

    overlume::destroy_renderer(r);
}

// ── objects.opacity theme token (VM-078) ──────────────────────────────────
// Reuses the exact staleness-swap machinery above -- no parallel path. At
// the shipped themes' default 1.0, alpha == staleness_alpha exactly (proven
// by StaleObjectFadesViaSharedStalenessAlpha above, byte-identical to
// before this token existed -- also why the existing goldens, unchanged by
// this feature, stay pixel-identical). Below 1.0 (objects_half_opacity.yaml
// fixture, VM-078's own theme.cpp test fixture), a FRESH object must now
// also bind translucent, at alpha == opacity.

TEST(Objects, HalfOpacityBindsTranslucentEvenWhileFresh) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "objects_half_opacity"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::TrackedObject obj = make_car(1, 0.0);  // fresh: last_update_sec == sim_time_sec
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.objects = &obj;
    s.object_count = 1;
    overlume::set_scene(r, s);
    render_once(r, kPose);

    const auto info = overlume::testing::object_material_info(r, 1);
    EXPECT_TRUE(info.bound_to_translucent)
        << "at objects.opacity 0.5 even a FRESH object must ride the translucent swap, "
           "not stay on the fully-opaque shared template";
    EXPECT_NEAR(info.alpha, 0.5f, 1e-4f);

    overlume::destroy_renderer(r);
}

TEST(Objects, HalfOpacityStalenessRampsDownFromTheOpacityCeilingNeverAboveIt) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "objects_half_opacity"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::TrackedObject obj = make_car(1, 0.0);
    obj.last_update_sec = 10.0 - 0.75;  // same offset as StaleObjectFadesViaSharedStalenessAlpha
                                        // above -> staleness_alpha ~0.5
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.objects = &obj;
    s.object_count = 1;
    overlume::set_scene(r, s);
    render_once(r, kPose);

    const auto info = overlume::testing::object_material_info(r, 1);
    EXPECT_TRUE(info.bound_to_translucent);
    // alpha = opacity(0.5) * staleness_alpha(~0.5) ~= 0.25 -- strictly below
    // the 0.5 opacity ceiling, proving the fade still ramps DOWN from it
    // rather than the opacity floor being clamped away.
    EXPECT_NEAR(info.alpha, 0.25f, 0.02f);
    EXPECT_LT(info.alpha, 0.5f);

    overlume::destroy_renderer(r);
}

// ── Dimensions drive scale, never the class model's own native size ──────

TEST(Objects, DimensionsDriveScaleNotTheClassModel) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::set_object_model_dir(r, OVERLUME_TEST_DATA_DIR "/assets/models");

    overlume::TrackedObject objs[3]{};
    objs[0] = make_car(1, 0.0);
    objs[0].dimensions = {4.5, 1.8, 1.5};
    objs[1] = make_car(2, 10.0);
    objs[1].dimensions = {12.0, 2.5, 3.2};  // same class, very different bbox
    objs[2].id = 3;
    objs[2].cls = overlume::ObjectClass::UNKNOWN;  // always the procedural box
    objs[2].position = {-10.0, 0.0, 0.0};
    objs[2].dimensions = {2.0, 2.0, 2.0};
    objs[2].last_update_sec = 10.0;

    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.objects = objs;
    s.object_count = 3;
    overlume::set_scene(r, s);
    render_once(r, kPose);

    // Both CAR objects share the SAME loaded class model; the normalize
    // pipeline guarantees every model's own X/Y footprint is exactly 1x1
    // (scripts/normalize_models.py), so the APPLIED scale's X/Y always
    // equal the raw perception dims regardless of which glb backs the
    // class. This hook reads the APPLIED TransformManager scale, never
    // RenderableManager's AABB -- add_mesh() hard-codes that to the same
    // 40x40x2 box for every renderable, see renderer.cpp.
    const auto small = overlume::testing::object_transform_scale(r, 1);
    const auto big = overlume::testing::object_transform_scale(r, 2);
    EXPECT_NEAR(small.x, 4.5, 1e-4);
    EXPECT_NEAR(small.y, 1.8, 1e-4);
    EXPECT_NEAR(big.x, 12.0, 1e-4);
    EXPECT_NEAR(big.y, 2.5, 1e-4);
    EXPECT_GT(big.x, small.x);

    // Procedural box path (UNKNOWN): build_unit_box() is always a unit
    // cube, so its unit footprint is (1,1,1) BY CONSTRUCTION and scale ==
    // dims exactly on every axis.
    const auto boxScale = overlume::testing::object_transform_scale(r, 3);
    EXPECT_NEAR(boxScale.x, 2.0, 1e-4);
    EXPECT_NEAR(boxScale.y, 2.0, 1e-4);
    EXPECT_NEAR(boxScale.z, 2.0, 1e-4);

    overlume::destroy_renderer(r);
}

// ── Vanished track: removed from the scene AND recycled, not destroyed ───

TEST(Objects, ObjectDisappearingIsRemovedFromTheSceneAndRecycled) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::set_object_model_dir(r, OVERLUME_TEST_DATA_DIR "/assets/models");

    std::vector<overlume::TrackedObject> five = {make_car(1, 0), make_car(2, 10), make_car(3, 20),
                                               make_car(4, 30), make_car(5, 40)};
    overlume::SceneGraph s5{};
    s5.sim_time_sec = 10.0;
    s5.objects = five.data();
    s5.object_count = static_cast<uint32_t>(five.size());
    overlume::set_scene(r, s5);
    render_once(r, kPose);

    ASSERT_TRUE(overlume::testing::object_in_scene(r, 2));
    ASSERT_TRUE(overlume::testing::object_in_scene(r, 4));
    const uint64_t id2Identity = overlume::testing::object_entity_identity(r, 2);
    const uint64_t id4Identity = overlume::testing::object_entity_identity(r, 4);

    // Objects 2 and 4 vanish.
    std::vector<overlume::TrackedObject> three = {five[0], five[2], five[4]};  // ids 1, 3, 5
    overlume::SceneGraph s3{};
    s3.sim_time_sec = 10.0;
    s3.objects = three.data();
    s3.object_count = static_cast<uint32_t>(three.size());
    overlume::set_scene(r, s3);
    render_once(r, kPose);

    EXPECT_FALSE(overlume::testing::object_in_scene(r, 2))
        << "a vanished track's renderable must be REMOVED from the scene";
    EXPECT_FALSE(overlume::testing::object_in_scene(r, 4));
    EXPECT_TRUE(overlume::testing::object_in_scene(r, 1));
    EXPECT_TRUE(overlume::testing::object_in_scene(r, 3));
    EXPECT_TRUE(overlume::testing::object_in_scene(r, 5));

    // A 6th object arrives: its instance must be one of the two just freed
    // (the free list is real, not a comment) — gltfio has no
    // destroyInstance(), so recycling, not destruction, is the only shape
    // available.
    std::vector<overlume::TrackedObject> four = {five[0], five[2], five[4], make_car(6, 50)};
    overlume::SceneGraph s4{};
    s4.sim_time_sec = 10.0;
    s4.objects = four.data();
    s4.object_count = static_cast<uint32_t>(four.size());
    overlume::set_scene(r, s4);
    render_once(r, kPose);

    ASSERT_TRUE(overlume::testing::object_in_scene(r, 6));
    const uint64_t id6Identity = overlume::testing::object_entity_identity(r, 6);
    EXPECT_TRUE(id6Identity == id2Identity || id6Identity == id4Identity)
        << "object 6 did not reuse either freed instance -- the free list is not being reused";

    overlume::destroy_renderer(r);
}

// ── Object class materials are themed on first data, no set_theme() ──────
// needed (Step 8a's exemplar, copied — see map_elements.cpp's own version)

TEST(Objects, ObjectMaterialIsThemedOnFirstDataWithNoTransition) {
    const auto theme = overlume::detail::load_theme(kThemeDir, "dark_adas");
    ASSERT_TRUE(theme.has_value());

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // First-ever object data. Nothing calls set_theme().
    overlume::TrackedObject obj = make_car(1, 0.0);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.objects = &obj;
    s.object_count = 1;
    overlume::set_scene(r, s);
    render_once(r, kPose);

    const auto carTint = overlume::testing::object_class_tint(r, overlume::ObjectClass::CAR);
    EXPECT_NEAR(carTint.r, theme->palette.object_tints.car.r, 1e-4);
    EXPECT_NEAR(carTint.g, theme->palette.object_tints.car.g, 1e-4);
    EXPECT_NEAR(carTint.b, theme->palette.object_tints.car.b, 1e-4);
    overlume::destroy_renderer(r);
}

// ── Step 5: the 50-object timing AC ───────────────────────────────────────

TEST(Objects, FiftyObjectsSceneUpdateUnderTwoMilliseconds) {
    overlume::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::set_object_model_dir(r, OVERLUME_TEST_DATA_DIR "/assets/models");

    constexpr int kObjectCount = 50;
    const overlume::ObjectClass kClasses[] = {
        overlume::ObjectClass::CAR,        overlume::ObjectClass::TRUCK_VAN, overlume::ObjectClass::BUS,
        overlume::ObjectClass::PEDESTRIAN, overlume::ObjectClass::CYCLIST,   overlume::ObjectClass::UNKNOWN,
    };
    std::vector<overlume::TrackedObject> objects(kObjectCount);
    for (int i = 0; i < kObjectCount; ++i) {
        overlume::TrackedObject& o = objects[i];
        o.id = static_cast<uint32_t>(i + 1);
        o.cls = kClasses[i % 6];
        o.position = {static_cast<double>((i % 10) * 4 - 18), static_cast<double>((i / 10) * 4 - 8), 0.0};
        o.heading_rad = 0.1 * i;
        o.dimensions = {4.0, 1.8, 1.5};
        o.velocity = (i % 2 == 0) ? overlume::Vec3{2.0, 0.0, 0.0} : overlume::Vec3{0.0, 0.0, 0.0};
        o.last_update_sec = 10.0;
    }
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.objects = objects.data();
    s.object_count = static_cast<uint32_t>(objects.size());
    overlume::set_scene(r, s);

    const overlume::CameraPose pose{{0, -30, 20}, {0, 0, 0}, 60.0};
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};

    constexpr int kIterations = 100;
    std::vector<double> msPerIter(kIterations);
    for (int i = 0; i < kIterations; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        ASSERT_TRUE(overlume::render_frame(r, pose, view));
        const auto t1 = std::chrono::steady_clock::now();
        msPerIter[i] = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    std::sort(msPerIter.begin(), msPerIter.end());
    const double median = msPerIter[kIterations / 2];
    // Recorded in the plan's Epic 2 results block along with the machine
    // this ran on (dev RTX 3090, possible CARLA contention) -- a regression
    // tripwire, not a robot-hardware claim (Epic 0 Task 6 is unmeasured).
    std::fprintf(stderr, "[test_objects] 50-object render_frame median: %.3f ms\n", median);
    EXPECT_LT(median, 2.0);

    overlume::destroy_renderer(r);
}

// ── Predicted-path ribbon teardown must not double-destroy: destroy_mesh()
// must null the Mesh it tears down, or a persistent track's predicted_path
// vanishing (while a second object stays live) re-enters destroy_mesh() on
// every subsequent frame, double-freeing Filament resources and recycling
// the entity id into a live object. Repro: path present -> path gone (2nd
// object present) -> repeat identical scene twice more.

TEST(Objects, VanishingPredictedPathDoesNotDoubleDestroyRibbon) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::set_object_model_dir(r, OVERLUME_TEST_DATA_DIR "/assets/models");

    const overlume::Vec3 path[4] = {{0, 0, 0}, {5, 0, 0}, {10, 1, 0}, {15, 2, 0}};

    overlume::TrackedObject withPath = make_car(1, 0.0);
    withPath.predicted_path = path;
    withPath.predicted_path_count = 4;

    overlume::SceneGraph s1{};
    s1.sim_time_sec = 10.0;
    s1.objects = &withPath;
    s1.object_count = 1;
    overlume::set_scene(r, s1);
    render_once(r, kPose);
    ASSERT_TRUE(overlume::testing::object_in_scene(r, 1));

    // Path vanishes on the SAME track, while a second object arrives.
    overlume::TrackedObject noPath = make_car(1, 0.0);  // predicted_path == nullptr, count == 0
    overlume::TrackedObject second = make_car(2, 20.0);
    std::vector<overlume::TrackedObject> pair = {noPath, second};

    overlume::SceneGraph s2{};
    s2.sim_time_sec = 10.0;
    s2.objects = pair.data();
    s2.object_count = static_cast<uint32_t>(pair.size());
    overlume::set_scene(r, s2);

    // Render the identical no-path pair three more times: this is the
    // steady-state re-entry the finding describes ("EVERY subsequent
    // frame"). Must not throw/crash and object 2 must stay live throughout.
    for (int i = 0; i < 3; ++i) {
        overlume::set_scene(r, s2);
        render_once(r, kPose);
        EXPECT_TRUE(overlume::testing::object_in_scene(r, 2))
            << "iteration " << i << ": recycled entity id must not have killed object 2's live entity";
    }

    overlume::destroy_renderer(r);
}
