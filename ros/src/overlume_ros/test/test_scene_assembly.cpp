// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/scene_assembly.hpp"

#include <gtest/gtest.h>

using overlume::ros::apply_layer_gates;
using overlume::ros::LayerFlags;
using overlume::ros::SceneAssembly;

namespace {

overlume::MapElement MakeElement() {
    overlume::MapElement e{};
    e.points = nullptr;
    e.point_count = 0;
    e.is_polygon = 0;
    return e;
}

}  // namespace

TEST(SceneAssembly, TwoAdaptersOnOneCategoryBothSurvive) {
    SceneAssembly asm_;
    for (int i = 0; i < 3; ++i) asm_.map_elements.push_back(MakeElement());
    for (int i = 0; i < 5; ++i) asm_.map_elements.push_back(MakeElement());

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.map_element_count, 8u);
    EXPECT_EQ(scene.map_elements, asm_.map_elements.data());
}

TEST(SceneAssembly, ClearBetweenTicksDoesNotAccumulate) {
    SceneAssembly asm_;
    for (int i = 0; i < 3; ++i) asm_.map_elements.push_back(MakeElement());
    overlume::SceneGraph scene1{};
    asm_.point_at(scene1);
    EXPECT_EQ(scene1.map_element_count, 3u);

    asm_.clear();
    for (int i = 0; i < 2; ++i) asm_.map_elements.push_back(MakeElement());
    overlume::SceneGraph scene2{};
    asm_.point_at(scene2);
    EXPECT_EQ(scene2.map_element_count, 2u);
}

TEST(SceneAssembly, ClearEmptiesEveryCategoryNotJustMapElements) {
    SceneAssembly asm_;
    asm_.map_elements.push_back(MakeElement());
    asm_.objects.push_back(overlume::TrackedObject{});
    asm_.paths.push_back(overlume::PathRibbon{});
    asm_.grids.push_back(overlume::GroundGridLayer{});
    asm_.alerts.push_back(overlume::AlertPolygon{});
    asm_.markers.push_back(overlume::GenericMarker{});
    asm_.point_clouds.push_back(overlume::PointCloud{});
    asm_.trajectory_carpets.push_back(overlume::TrajectoryCarpet{});

    asm_.clear();

    overlume::SceneGraph scene{};
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

TEST(SceneAssembly, PointCloudRowAppendsIntoScenePointClouds) {
    SceneAssembly asm_;
    asm_.point_clouds.push_back(overlume::PointCloud{});
    asm_.point_clouds.push_back(overlume::PointCloud{});

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 2u);
    EXPECT_EQ(scene.point_clouds, asm_.point_clouds.data());
}

TEST(SceneAssembly, ApplyLayerGatesPointCloudsOffZeroesOnlyPointClouds) {
    SceneAssembly asm_;
    asm_.point_clouds.push_back(overlume::PointCloud{});
    asm_.objects.push_back(overlume::TrackedObject{});

    LayerFlags flags;
    flags.point_clouds = false;
    apply_layer_gates(asm_, flags);

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 0u);
    EXPECT_EQ(scene.object_count, 1u);
}

TEST(SceneAssembly, ApplyLayerGatesPointCloudsOnLeavesItIntact) {
    SceneAssembly asm_;
    asm_.point_clouds.push_back(overlume::PointCloud{});

    LayerFlags flags;
    apply_layer_gates(asm_, flags);

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 1u);
}

TEST(SceneAssembly, ContentPushedAfterApplyLayerGatesSurvivesPointCloudsGateFalse) {
    SceneAssembly asm_;
    asm_.point_clouds.push_back(overlume::PointCloud{});

    LayerFlags flags;
    flags.point_clouds = false;
    apply_layer_gates(asm_, flags);

    overlume::SceneGraph mid{};
    asm_.point_at(mid);
    EXPECT_EQ(mid.point_cloud_count, 0u);

    asm_.point_clouds.push_back(overlume::PointCloud{});

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 1u);
}

TEST(SceneAssembly, TrajectoryCarpetRowAppendsIntoSceneTrajectoryCarpets) {
    SceneAssembly asm_;
    asm_.trajectory_carpets.push_back(overlume::TrajectoryCarpet{});

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.trajectory_carpet_count, 1u);
    EXPECT_EQ(scene.trajectory_carpets, asm_.trajectory_carpets.data());
}

TEST(SceneAssembly, ApplyLayerGatesTrajectoryCarpetOffZeroesOnlyTrajectoryCarpets) {
    SceneAssembly asm_;
    asm_.trajectory_carpets.push_back(overlume::TrajectoryCarpet{});
    asm_.objects.push_back(overlume::TrackedObject{});

    LayerFlags flags;
    flags.trajectory_carpet = false;
    apply_layer_gates(asm_, flags);

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.trajectory_carpet_count, 0u);
    EXPECT_EQ(scene.object_count, 1u);
}

TEST(SceneAssembly, ApplyLayerGatesTrajectoryCarpetOnLeavesItIntact) {
    SceneAssembly asm_;
    asm_.trajectory_carpets.push_back(overlume::TrajectoryCarpet{});

    LayerFlags flags;
    apply_layer_gates(asm_, flags);

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.trajectory_carpet_count, 1u);
}

using overlume::ros::compose_layer_gates;
using overlume::ros::mode_content_mask;
using overlume::ros::RenderMode;

TEST(SceneAssembly, ModeContentMaskBowlIsAllFalse) {
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

TEST(SceneAssembly, ModeContentMaskHybridAllowsOnlyPointClouds) {
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

TEST(SceneAssembly, ModeContentMaskFreeLookIsAllTrue) {
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

TEST(SceneAssembly, ComposeLayerGatesBowlMasksEverythingEvenWhenUserWantsItOn) {
    LayerFlags user;
    LayerFlags effective = compose_layer_gates(user, mode_content_mask(RenderMode::BOWL));
    EXPECT_FALSE(effective.objects);
    EXPECT_FALSE(effective.paths);
    EXPECT_FALSE(effective.point_clouds);
    EXPECT_TRUE(user.objects);
    EXPECT_TRUE(user.point_clouds);
}

TEST(SceneAssembly, ComposeLayerGatesNeverTurnsOnWhatTheUserTurnedOff) {
    LayerFlags user;
    user.objects = false;
    LayerFlags effective = compose_layer_gates(user, mode_content_mask(RenderMode::FREE_LOOK));
    EXPECT_FALSE(effective.objects);
    EXPECT_TRUE(effective.paths);
}

TEST(SceneAssembly, ComposeLayerGatesHybridLeavesUsersPointCloudsChoiceUnion) {
    LayerFlags user;
    user.point_clouds = false;
    LayerFlags effective = compose_layer_gates(user, mode_content_mask(RenderMode::HYBRID));
    EXPECT_FALSE(effective.point_clouds);
}

using overlume::ros::bowl_visible_for_mode;
using overlume::ros::overlays_visible_for_mode;

TEST(SceneAssembly, BowlVisibleForBowlMode) {
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::BOWL, false));
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::BOWL, true));
}

TEST(SceneAssembly, BowlVisibleForHybridMode) {
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::HYBRID, false));
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::HYBRID, true));
}

TEST(SceneAssembly, BowlHiddenInFreeLookWithSurroundStitchingOff) {
    EXPECT_FALSE(bowl_visible_for_mode(RenderMode::FREE_LOOK, false));
}

TEST(SceneAssembly, BowlVisibleInFreeLookWithSurroundStitchingOn) {
    EXPECT_TRUE(bowl_visible_for_mode(RenderMode::FREE_LOOK, true));
}

TEST(SceneAssembly, OverlaysVisibleOnlyInFreeLook) {
    EXPECT_FALSE(overlays_visible_for_mode(RenderMode::BOWL));
    EXPECT_FALSE(overlays_visible_for_mode(RenderMode::HYBRID));
    EXPECT_TRUE(overlays_visible_for_mode(RenderMode::FREE_LOOK));
}

using overlume::ros::environment_effectively_visible;

TEST(SceneAssembly, EnvironmentHiddenInBowlRegardlessOfEnabled) {
    EXPECT_FALSE(environment_effectively_visible(RenderMode::BOWL, true));
    EXPECT_FALSE(environment_effectively_visible(RenderMode::BOWL, false));
}

TEST(SceneAssembly, EnvironmentHiddenInHybridRegardlessOfEnabled) {
    EXPECT_FALSE(environment_effectively_visible(RenderMode::HYBRID, true));
    EXPECT_FALSE(environment_effectively_visible(RenderMode::HYBRID, false));
}

TEST(SceneAssembly, EnvironmentInFreeLookFollowsEnabledSwitch) {
    EXPECT_TRUE(environment_effectively_visible(RenderMode::FREE_LOOK, true));
    EXPECT_FALSE(environment_effectively_visible(RenderMode::FREE_LOOK, false));
}

namespace {
overlume::PointCloudPoint CarpetPt(double x, double y, uint32_t rgba) {
    overlume::PointCloudPoint p{};
    p.position = {x, y, 0.0};
    p.rgba = rgba;
    return p;
}
}  // namespace

TEST(SceneAssembly, RespineMovesVelocityRibbonOntoLocalSpineWithStationColors) {
    using overlume::ros::SceneAssembly;
    SceneAssembly a;
    const overlume::Vec3 lp[] = {{0, 1, 0}, {2, 1, 0}, {4, 1, 0}, {6, 1, 0}, {8, 1, 0}};
    overlume::PathRibbon local{};
    local.role = overlume::PathRole::LOCAL;
    local.points = lp;
    local.point_count = 5;
    a.paths.push_back(local);
    const overlume::PointCloudPoint cp[] = {
        CarpetPt(0, 0, 0xff0000ffu), CarpetPt(3, 0, 0xff00ff00u), CarpetPt(6, 0, 0xffff0000u)};
    overlume::TrajectoryCarpet carpet{};
    carpet.points = cp;
    carpet.point_count = 3;
    a.trajectory_carpets.push_back(carpet);

    overlume::ros::respine_velocity_ribbon_onto_local_path(a);

    ASSERT_EQ(a.trajectory_carpets[0].point_count, 5u);
    const auto* pts = a.trajectory_carpets[0].points;
    for (uint32_t i = 0; i < 5; ++i) {
        EXPECT_DOUBLE_EQ(pts[i].position.x, lp[i].x);
        EXPECT_DOUBLE_EQ(pts[i].position.y, 1.0);
    }
    EXPECT_EQ(pts[0].rgba, 0xff0000ffu);
    EXPECT_EQ(pts[1].rgba, 0xff00ff00u);
    EXPECT_EQ(pts[2].rgba, 0xff00ff00u);
    EXPECT_EQ(pts[3].rgba, 0xffff0000u);
    EXPECT_EQ(pts[4].rgba, 0xffff0000u);
}

TEST(SceneAssembly, RespineWithoutLocalRibbonLeavesCarpetOnItsOwnSpine) {
    using overlume::ros::SceneAssembly;
    SceneAssembly a;
    const overlume::PointCloudPoint cp[] = {CarpetPt(0, 0, 1u), CarpetPt(3, 0, 2u)};
    overlume::TrajectoryCarpet carpet{};
    carpet.points = cp;
    carpet.point_count = 2;
    a.trajectory_carpets.push_back(carpet);

    overlume::ros::respine_velocity_ribbon_onto_local_path(a);

    EXPECT_EQ(a.trajectory_carpets[0].points, cp);
    EXPECT_EQ(a.trajectory_carpets[0].point_count, 2u);
    EXPECT_TRUE(a.respined_carpet_points.empty());
}
