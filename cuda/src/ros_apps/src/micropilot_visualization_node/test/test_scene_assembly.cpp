/** @file test_scene_assembly.cpp
 *  @brief SceneAssembly tests.
 */
#include "micropilot_visualization_node/scene_assembly.hpp"

#include <gtest/gtest.h>

using micropilot::visualization_app::apply_layer_gates;
using micropilot::visualization_app::LayerFlags;
using micropilot::visualization_app::SceneAssembly;

namespace
{

mpviz::MapElement MakeElement()
{
    mpviz::MapElement e{};
    e.points = nullptr;
    e.point_count = 0;
    e.is_polygon = 0;
    return e;
}

}  // namespace

TEST(SceneAssembly, TwoAdaptersOnOneCategoryBothSurvive)
{
    // the finding that motivated SceneAssembly: fill() two hd_map adapters
    // (3 + 5 elements) -> map_element_count == 8, not 5.
    SceneAssembly asm_;
    for (int i = 0; i < 3; ++i) asm_.map_elements.push_back(MakeElement());  // adapter A's fill()
    for (int i = 0; i < 5; ++i) asm_.map_elements.push_back(MakeElement());  // adapter B's fill()

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.map_element_count, 8u);
    EXPECT_EQ(scene.map_elements, asm_.map_elements.data());
}

TEST(SceneAssembly, ClearBetweenTicksDoesNotAccumulate)
{
    SceneAssembly asm_;
    for (int i = 0; i < 3; ++i) asm_.map_elements.push_back(MakeElement());
    mpviz::SceneGraph scene1{};
    asm_.point_at(scene1);
    EXPECT_EQ(scene1.map_element_count, 3u);

    asm_.clear();
    for (int i = 0; i < 2; ++i) asm_.map_elements.push_back(MakeElement());
    mpviz::SceneGraph scene2{};
    asm_.point_at(scene2);
    EXPECT_EQ(scene2.map_element_count, 2u);  // not 3+2 == 5
}

TEST(SceneAssembly, ClearEmptiesEveryCategoryNotJustMapElements)
{
    SceneAssembly asm_;
    asm_.map_elements.push_back(MakeElement());
    asm_.objects.push_back(mpviz::TrackedObject{});
    asm_.paths.push_back(mpviz::PathRibbon{});
    asm_.grids.push_back(mpviz::GroundGridLayer{});
    asm_.alerts.push_back(mpviz::AlertPolygon{});
    asm_.markers.push_back(mpviz::GenericMarker{});
    asm_.point_clouds.push_back(mpviz::PointCloud{});
    asm_.trajectory_carpets.push_back(mpviz::TrajectoryCarpet{});

    asm_.clear();

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.map_element_count, 0u);
    EXPECT_EQ(scene.object_count, 0u);
    EXPECT_EQ(scene.path_count, 0u);
    EXPECT_EQ(scene.grid_count, 0u);
    EXPECT_EQ(scene.alert_count, 0u);
    EXPECT_EQ(scene.marker_count, 0u);
    EXPECT_EQ(scene.point_cloud_count, 0u);
    EXPECT_EQ(scene.trajectory_carpet_count, 0u);
}

TEST(SceneAssembly, PointCloudRowAppendsIntoScenePointClouds)
{
    // Same shape as TwoAdaptersOnOneCategoryBothSurvive above, for the new
    // category (Task 6 / VM-035).
    SceneAssembly asm_;
    asm_.point_clouds.push_back(mpviz::PointCloud{});
    asm_.point_clouds.push_back(mpviz::PointCloud{});

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 2u);
    EXPECT_EQ(scene.point_clouds, asm_.point_clouds.data());
}

TEST(SceneAssembly, ApplyLayerGatesPointCloudsOffZeroesOnlyPointClouds)
{
    // The finding this covers: visualization_node.cpp's
    // `if (!layer_point_clouds_) scene_asm_.point_clouds.clear();` gate had
    // no test exercising it directly. apply_layer_gates() is the exact
    // function that line now calls, with the same LayerFlags shape.
    SceneAssembly asm_;
    asm_.point_clouds.push_back(mpviz::PointCloud{});
    asm_.objects.push_back(mpviz::TrackedObject{});  // another category, must survive

    LayerFlags flags;  // all true by default
    flags.point_clouds = false;
    apply_layer_gates(asm_, flags);

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 0u);
    EXPECT_EQ(scene.object_count, 1u);
}

TEST(SceneAssembly, ApplyLayerGatesPointCloudsOnLeavesItIntact)
{
    SceneAssembly asm_;
    asm_.point_clouds.push_back(mpviz::PointCloud{});

    LayerFlags flags;  // all true, including point_clouds
    apply_layer_gates(asm_, flags);

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 1u);
}

TEST(SceneAssembly, TrajectoryCarpetRowAppendsIntoSceneTrajectoryCarpets)
{
    // Same shape as PointCloudRowAppendsIntoScenePointClouds above, for the
    // new category (VM-077).
    SceneAssembly asm_;
    asm_.trajectory_carpets.push_back(mpviz::TrajectoryCarpet{});

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.trajectory_carpet_count, 1u);
    EXPECT_EQ(scene.trajectory_carpets, asm_.trajectory_carpets.data());
}

TEST(SceneAssembly, ApplyLayerGatesTrajectoryCarpetOffZeroesOnlyTrajectoryCarpets)
{
    // Task 3 Step 2: closes the gap the epic3 plan's own review fix named
    // for layer_point_clouds -- this plan ships the gate test in the SAME
    // task as the category, no forward-reference window left open.
    SceneAssembly asm_;
    asm_.trajectory_carpets.push_back(mpviz::TrajectoryCarpet{});
    asm_.objects.push_back(mpviz::TrackedObject{});  // another category, must survive

    LayerFlags flags;  // all true by default
    flags.trajectory_carpet = false;
    apply_layer_gates(asm_, flags);

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.trajectory_carpet_count, 0u);
    EXPECT_EQ(scene.object_count, 1u);
}

TEST(SceneAssembly, ApplyLayerGatesTrajectoryCarpetOnLeavesItIntact)
{
    SceneAssembly asm_;
    asm_.trajectory_carpets.push_back(mpviz::TrajectoryCarpet{});

    LayerFlags flags;  // all true, including trajectory_carpet
    apply_layer_gates(asm_, flags);

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.trajectory_carpet_count, 1u);
}

// ── Task 4 (VM-093) per-mode content mask, USER DIRECTIVE 2026-09-11 ───────

using micropilot::visualization_app::compose_layer_gates;
using micropilot::visualization_app::mode_content_mask;
using micropilot::visualization_app::RenderMode;

TEST(SceneAssembly, ModeContentMaskBowlIsAllFalse)
{
    // BOWL = bowl + ego ONLY (Decision resolutions, mode content
    // exclusivity): every SceneAssembly/LayerFlags category is masked off,
    // regardless of the user's own layer_* settings (checked by
    // compose_layer_gates below, not here).
    LayerFlags mask = mode_content_mask(RenderMode::BOWL);
    EXPECT_FALSE(mask.objects);
    EXPECT_FALSE(mask.paths);
    EXPECT_FALSE(mask.map_elements);
    EXPECT_FALSE(mask.grids);
    EXPECT_FALSE(mask.alerts);
    EXPECT_FALSE(mask.markers);
    EXPECT_FALSE(mask.point_clouds);
    EXPECT_FALSE(mask.trajectory_carpet);
}

TEST(SceneAssembly, ModeContentMaskHybridAllowsOnlyPointClouds)
{
    // HYBRID = bowl + camera-colorized lidar + ego ONLY -- lidar rides the
    // point_clouds category (Task 5/VM-094); everything else stays masked
    // off, same as BOWL.
    LayerFlags mask = mode_content_mask(RenderMode::HYBRID);
    EXPECT_FALSE(mask.objects);
    EXPECT_FALSE(mask.paths);
    EXPECT_FALSE(mask.map_elements);
    EXPECT_FALSE(mask.grids);
    EXPECT_FALSE(mask.alerts);
    EXPECT_FALSE(mask.markers);
    EXPECT_TRUE(mask.point_clouds);
    EXPECT_FALSE(mask.trajectory_carpet);
}

TEST(SceneAssembly, ModeContentMaskFreeLookIsAllTrue)
{
    // FREE_LOOK = the full autonomy scene, unmasked -- AND-ing this over the
    // user's own flags in compose_layer_gates() must be a true no-op.
    LayerFlags mask = mode_content_mask(RenderMode::FREE_LOOK);
    EXPECT_TRUE(mask.objects);
    EXPECT_TRUE(mask.paths);
    EXPECT_TRUE(mask.map_elements);
    EXPECT_TRUE(mask.grids);
    EXPECT_TRUE(mask.alerts);
    EXPECT_TRUE(mask.markers);
    EXPECT_TRUE(mask.point_clouds);
    EXPECT_TRUE(mask.trajectory_carpet);
}

TEST(SceneAssembly, ComposeLayerGatesBowlMasksEverythingEvenWhenUserWantsItOn)
{
    // The user has every layer turned ON (their persisted layer_* params) --
    // BOWL must still mask everything off. This is the "AND, never
    // overwrite" contract: composing must not touch `user` itself.
    LayerFlags user;  // all true, default member initializers
    LayerFlags effective = compose_layer_gates(user, mode_content_mask(RenderMode::BOWL));
    EXPECT_FALSE(effective.objects);
    EXPECT_FALSE(effective.paths);
    EXPECT_FALSE(effective.point_clouds);
    // `user` itself is untouched -- compose_layer_gates takes it by const&
    // and returns a new value, never mutates the caller's copy.
    EXPECT_TRUE(user.objects);
    EXPECT_TRUE(user.point_clouds);
}

TEST(SceneAssembly, ComposeLayerGatesNeverTurnsOnWhatTheUserTurnedOff)
{
    // The user turned `objects` off themselves (their own layer_objects
    // param) -- FREE_LOOK's all-true mask must not resurrect it. This is
    // the "switching back to FREE_LOOK restores the user's own settings
    // exactly" half of the directive.
    LayerFlags user;
    user.objects = false;
    LayerFlags effective = compose_layer_gates(user, mode_content_mask(RenderMode::FREE_LOOK));
    EXPECT_FALSE(effective.objects);
    EXPECT_TRUE(effective.paths);  // everything else the user left on stays on
}

TEST(SceneAssembly, ComposeLayerGatesHybridLeavesUsersPointCloudsChoiceUnion)
{
    // HYBRID's mask allows point_clouds through, but composition is still
    // an AND against the user's own setting -- if the user had turned
    // point_clouds off themselves, HYBRID must not turn it back on.
    LayerFlags user;
    user.point_clouds = false;
    LayerFlags effective = compose_layer_gates(user, mode_content_mask(RenderMode::HYBRID));
    EXPECT_FALSE(effective.point_clouds);
}

// ── Bowl visibility / overlay-suppression dispatch (review round 1, 2026-09-11) ──
// Pulled out of visualization_node.cpp's timer_callback() so the actual
// dispatch predicates -- not just the LayerFlags mask above -- are directly
// unit-tested. Before this, nothing failed if `bowl_visible_for_mode()`'s
// predicate were inverted or deleted (test_mode_dispatch.py only checks
// param accept/reject; smoke_test.py only checks frame SHAPE).

using micropilot::visualization_app::bowl_visible_for_mode;
using micropilot::visualization_app::overlays_visible_for_mode;

TEST(SceneAssembly, BowlVisibleForBowlMode)
{
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::BOWL, /*surround_stitching=*/false));
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::BOWL, /*surround_stitching=*/true));
}

TEST(SceneAssembly, BowlVisibleForHybridMode)
{
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::HYBRID, /*surround_stitching=*/false));
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::HYBRID, /*surround_stitching=*/true));
}

TEST(SceneAssembly, BowlHiddenInFreeLookWithSurroundStitchingOff)
{
    EXPECT_FALSE(bowl_visible_for_mode(RenderMode::FREE_LOOK, /*surround_stitching=*/false));
}

TEST(SceneAssembly, BowlVisibleInFreeLookWithSurroundStitchingOn)
{
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::FREE_LOOK, /*surround_stitching=*/true));
}

TEST(SceneAssembly, OverlaysVisibleOnlyInFreeLook)
{
    EXPECT_FALSE(overlays_visible_for_mode(RenderMode::BOWL));
    EXPECT_FALSE(overlays_visible_for_mode(RenderMode::HYBRID));
    EXPECT_TRUE(overlays_visible_for_mode(RenderMode::FREE_LOOK));
}

// ── Velocity-ribbon re-spine (user directive 2026-09-10) ────────────────────

namespace {
mpviz::PointCloudPoint CarpetPt(double x, double y, uint32_t rgba)
{
    mpviz::PointCloudPoint p{};
    p.position = {x, y, 0.0};
    p.rgba = rgba;
    return p;
}
}  // namespace

TEST(SceneAssembly, RespineMovesVelocityRibbonOntoLocalSpineWithStationColors)
{
    using micropilot::visualization_app::SceneAssembly;
    SceneAssembly a;
    // Local path: straight, 5 points, 2m apart (stations 0,2,4,6,8).
    const mpviz::Vec3 lp[] = {{0, 1, 0}, {2, 1, 0}, {4, 1, 0}, {6, 1, 0}, {8, 1, 0}};
    mpviz::PathRibbon local{};
    local.role = mpviz::PathRole::LOCAL;
    local.points = lp;
    local.point_count = 5;
    a.paths.push_back(local);
    // Carpet: its OWN offset spine (y=0), 3 stations at 0/3/6m, colors R,G,B.
    const mpviz::PointCloudPoint cp[] = {CarpetPt(0, 0, 0xff0000ffu), CarpetPt(3, 0, 0xff00ff00u),
                                          CarpetPt(6, 0, 0xffff0000u)};
    mpviz::TrajectoryCarpet carpet{};
    carpet.points = cp;
    carpet.point_count = 3;
    a.trajectory_carpets.push_back(carpet);

    micropilot::visualization_app::respine_velocity_ribbon_onto_local_path(a);

    ASSERT_EQ(a.trajectory_carpets[0].point_count, 5u);
    const auto* pts = a.trajectory_carpets[0].points;
    for (uint32_t i = 0; i < 5; ++i)
    {
        // Geometry = the LOCAL spine verbatim (y=1), not the carpet's y=0.
        EXPECT_DOUBLE_EQ(pts[i].position.x, lp[i].x);
        EXPECT_DOUBLE_EQ(pts[i].position.y, 1.0);
    }
    // Station-nearest colors: 0->R(0), 2->G(3 vs 0: |3-2|<|2-0|), 4->G(3),
    // 6->B(6), 8->B held past the carpet's end.
    EXPECT_EQ(pts[0].rgba, 0xff0000ffu);
    EXPECT_EQ(pts[1].rgba, 0xff00ff00u);
    EXPECT_EQ(pts[2].rgba, 0xff00ff00u);
    EXPECT_EQ(pts[3].rgba, 0xffff0000u);
    EXPECT_EQ(pts[4].rgba, 0xffff0000u);
}

TEST(SceneAssembly, RespineWithoutLocalRibbonLeavesCarpetOnItsOwnSpine)
{
    using micropilot::visualization_app::SceneAssembly;
    SceneAssembly a;
    const mpviz::PointCloudPoint cp[] = {CarpetPt(0, 0, 1u), CarpetPt(3, 0, 2u)};
    mpviz::TrajectoryCarpet carpet{};
    carpet.points = cp;
    carpet.point_count = 2;
    a.trajectory_carpets.push_back(carpet);

    micropilot::visualization_app::respine_velocity_ribbon_onto_local_path(a);

    EXPECT_EQ(a.trajectory_carpets[0].points, cp);  // untouched pointer
    EXPECT_EQ(a.trajectory_carpets[0].point_count, 2u);
    EXPECT_TRUE(a.respined_carpet_points.empty());
}
