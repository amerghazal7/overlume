// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/profile.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "overlume/scene.h"
#include "fixture_msgs.hpp"

using overlume::ros::classify;
using overlume::ros::find_row;
using overlume::ros::load_profile;
using overlume::ros::load_profile_string;
using overlume::ros::match_rule;
using overlume::ros::NsRender;
using overlume::ros::subscriptions_for;

TEST(Profile, ShippedUrbanProfileLoads) {
    std::vector<std::string> errs;
    auto p =
        overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(errs.empty());
    EXPECT_FALSE(p->rows.empty());
}

TEST(Profile, ShippedOffroadProfileLoads) {
    std::vector<std::string> errs;
    auto p =
        overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/offroad_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(errs.empty());
    EXPECT_FALSE(p->rows.empty());
}

TEST(Profile, ShippedSimProfileLoadsAndMarksTheLatchedHdMapRow) {
    std::vector<std::string> errs;
    auto p = overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto* row = find_row(*p, "/sim/hd_map/markers");
    ASSERT_NE(row, nullptr);
    EXPECT_TRUE(row->transient_local);
}

TEST(Profile, NamespaceRuleIsLongestPrefixWins) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, ns_default: drop, namespaces:"
        " [{prefix: centerline_, render: polyline},"
        "  {prefix: centerline_arrows_, render: drop},"
        "  {prefix: crosswalk_, render: polygon},"
        "  {prefix: crosswalk_stopline_, render: polyline}]}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto& r = p->rows[0];
    EXPECT_EQ(classify(r, "centerline_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(r, "centerline_arrows_0"), NsRender::kDrop);
    EXPECT_EQ(classify(r, "crosswalk_7"), NsRender::kPolygon);
    EXPECT_EQ(classify(r, "crosswalk_stopline_7"), NsRender::kPolyline);
    EXPECT_EQ(classify(r, "traffic_light_2"), NsRender::kDrop);
}

TEST(Profile, KindParsesOnNamespaceRules) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_,    render: polyline, kind: centerline},"
        "  {prefix: left_boundary_, render: polyline, kind: left_boundary},"
        "  {prefix: junk_,          render: polyline}]}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    ASSERT_EQ(p->rows[0].namespaces.size(), 3u);
    EXPECT_EQ(p->rows[0].namespaces[0].kind, overlume::MapKind::CENTERLINE);
    EXPECT_EQ(p->rows[0].namespaces[1].kind, overlume::MapKind::LEFT_BOUNDARY);
    EXPECT_EQ(p->rows[0].namespaces[2].kind, overlume::MapKind::OTHER);
}

TEST(Profile, KindIsRejectedWhenIllegalForRender) {
    std::vector<std::string> polyline_errs;
    auto polyline = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: crosswalk_, render: polyline, kind: crosswalk}]}\n",
        polyline_errs);
    EXPECT_FALSE(polyline.has_value());
    ASSERT_EQ(polyline_errs.size(), 1u);
    EXPECT_NE(polyline_errs[0].find("kind"), std::string::npos);
    EXPECT_NE(polyline_errs[0].find("crosswalk_"), std::string::npos);

    std::vector<std::string> polygon_errs;
    auto polygon = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_, render: polygon, kind: centerline}]}\n",
        polygon_errs);
    EXPECT_FALSE(polygon.has_value());
    ASSERT_EQ(polygon_errs.size(), 1u);
    EXPECT_NE(polygon_errs[0].find("kind"), std::string::npos);
}

TEST(Profile, UnknownKindValueIsRejectedWithRowContext) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_, render: polyline, kind: teleporter}]}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("teleporter"), std::string::npos);
}

TEST(Profile, ShippedUrbanLocalRowMarksCenterlineAndBoundaryKinds) {
    std::vector<std::string> errs;
    auto p = load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto* row = find_row(*p, "/hd_map_local_elements");
    ASSERT_NE(row, nullptr);

    const auto* centerline_rule = match_rule(*row, "centerline_0");
    ASSERT_NE(centerline_rule, nullptr);
    EXPECT_EQ(centerline_rule->render, NsRender::kDrop);
    EXPECT_EQ(centerline_rule->kind, overlume::MapKind::OTHER);

    const auto* left_rule = match_rule(*row, "left_boundary_0");
    ASSERT_NE(left_rule, nullptr);
    EXPECT_EQ(left_rule->render, NsRender::kPolyline);
    EXPECT_EQ(left_rule->kind, overlume::MapKind::LEFT_BOUNDARY);

    const auto* right_rule = match_rule(*row, "right_boundary_0");
    ASSERT_NE(right_rule, nullptr);
    EXPECT_EQ(right_rule->render, NsRender::kPolyline);
    EXPECT_EQ(right_rule->kind, overlume::MapKind::RIGHT_BOUNDARY);

    const auto* crosswalk_rule = match_rule(*row, "crosswalk_7");
    ASSERT_NE(crosswalk_rule, nullptr);
    EXPECT_EQ(crosswalk_rule->render, NsRender::kPolygon);
    EXPECT_EQ(crosswalk_rule->kind, overlume::MapKind::CROSSWALK);

    const auto* stopline_rule = match_rule(*row, "crosswalk_stopline_7");
    ASSERT_NE(stopline_rule, nullptr);
    EXPECT_EQ(stopline_rule->render, NsRender::kPolyline);
    EXPECT_EQ(stopline_rule->kind, overlume::MapKind::STOPLINE);
}

TEST(Profile, DuplicateNamespacePrefixIsRejected) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_, render: polyline},"
        "  {prefix: centerline_, render: drop}]}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("duplicate"), std::string::npos);
    EXPECT_NE(errs[0].find("centerline_"), std::string::npos);
}

TEST(Profile, ShippedProfilesRouteEveryKnownNamespaceOfEveryShippedTopic) {
    std::vector<std::string> errs;
    auto urban =
        overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(urban.has_value());
    auto sim =
        overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    ASSERT_TRUE(sim.has_value());

    EXPECT_EQ(find_row(*urban, "/road_markers"), nullptr);

    const auto* hd_map_global = find_row(*urban, "/hd_map_global_elements");
    ASSERT_NE(hd_map_global, nullptr);
    EXPECT_EQ(classify(*hd_map_global, "centerline_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*hd_map_global, "centerline_arrows_0"), NsRender::kDrop);

    const auto* hd_map_local = find_row(*urban, "/hd_map_local_elements");
    ASSERT_NE(hd_map_local, nullptr);
    EXPECT_EQ(classify(*hd_map_local, "centerline_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*hd_map_local, "centerline_arrows_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*hd_map_local, "left_boundary_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(*hd_map_local, "right_boundary_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(*hd_map_local, "crosswalk_7"), NsRender::kPolygon);
    EXPECT_EQ(classify(*hd_map_local, "crosswalk_stopline_7"), NsRender::kPolyline);
    EXPECT_EQ(classify(*hd_map_local, "crosswalks"), NsRender::kPolygon);

    const auto* dyn_objects = find_row(*urban, "/perception/dynamic_objects_list");
    ASSERT_NE(dyn_objects, nullptr);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_bbox"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_text"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_arrow"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_hd_map_path"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_hd_map_path_dots"), NsRender::kDrop);

    const auto* sim_hd_map = find_row(*sim, "/sim/hd_map/markers");
    ASSERT_NE(sim_hd_map, nullptr);
    EXPECT_EQ(classify(*sim_hd_map, "centerline_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*sim_hd_map, "centerline_arrows_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*sim_hd_map, "crosswalks"), NsRender::kPolygon);
    EXPECT_EQ(classify(*sim_hd_map, "junction"), NsRender::kPolyline);
    EXPECT_EQ(classify(*sim_hd_map, "landmark"), NsRender::kDrop);
    EXPECT_EQ(classify(*sim_hd_map, "landmark_text"), NsRender::kDrop);
}

TEST(Profile, OgmRowCarriesItsUpdateTopicAndNonOgmRowsMayNot) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
        " adapter: ogm, role: dynamic_ogm, update_topic: /g_updates}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].update_topic, "/g_updates");

    std::vector<std::string> errs2;
    auto bad = load_profile_string(
        "name: t\nrows:\n  - {topic: /p, type: nav_msgs/msg/Path,"
        " adapter: path, role: behavior, update_topic: /p_updates}\n",
        errs2);
    EXPECT_FALSE(bad.has_value());
    ASSERT_EQ(errs2.size(), 1u);
    EXPECT_NE(errs2[0].find("update_topic"), std::string::npos);
}

TEST(Profile, JunctionInteriorBoundariesDefaultsToTrueAndParsesExplicitFalse) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /h, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(p->rows[0].junction_interior_boundaries);

    std::vector<std::string> errs2;
    auto p2 = load_profile_string(
        "name: t\nrows:\n  - {topic: /h, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, junction_interior_boundaries: false}\n",
        errs2);
    ASSERT_TRUE(p2.has_value()) << (errs2.empty() ? "" : errs2[0]);
    EXPECT_FALSE(p2->rows[0].junction_interior_boundaries);
}

TEST(Profile, JunctionInteriorBoundariesIsRejectedOnNonHdMapRows) {
    std::vector<std::string> errs;
    auto bad = load_profile_string(
        "name: t\nrows:\n  - {topic: /p, type: nav_msgs/msg/Path,"
        " adapter: path, role: behavior, junction_interior_boundaries: false}\n",
        errs);
    EXPECT_FALSE(bad.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("junction_interior_boundaries"), std::string::npos);
}

TEST(Profile, PointCloudRowDefaultsColorModeAutoStrideOne) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].color_mode, "auto");
    EXPECT_EQ(p->rows[0].max_points, 0u);
    EXPECT_EQ(p->rows[0].stride, 1u);
}

TEST(Profile, PointCloudRowParsesColorModeMaxPointsStride) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, color_mode: height, max_points: 5000,"
        " stride: 4}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].color_mode, "height");
    EXPECT_EQ(p->rows[0].max_points, 5000u);
    EXPECT_EQ(p->rows[0].stride, 4u);
}

TEST(Profile, PointCloudRowRejectsUnknownColorMode) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, color_mode: rainbow}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("color_mode"), std::string::npos);
}

TEST(Profile, PointCloudRowRejectsZeroStride) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, stride: 0}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("stride"), std::string::npos);
}

TEST(Profile, PointCloudRowMinZDefaultsToNaN) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(std::isnan(p->rows[0].min_z_m));
}

TEST(Profile, PointCloudRowParsesMinZ) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, min_z_m: 0.2}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_DOUBLE_EQ(p->rows[0].min_z_m, 0.2);
}

TEST(Profile, PointCloudRowRejectsNonNumericMinZ) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, min_z_m: not_a_number}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
}

TEST(Profile, MinZIsRejectedOnNonPointCloudRows) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /planning/path, type: nav_msgs/msg/Path,"
        " adapter: path, role: local, min_z_m: 0.2}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("min_z_m"), std::string::npos);
}

TEST(Profile, ColorModeMaxPointsStrideAreRejectedOnNonPointCloudRows) {
    std::vector<std::string> errs;
    auto bad = load_profile_string(
        "name: t\nrows:\n  - {topic: /p, type: nav_msgs/msg/Path,"
        " adapter: path, role: behavior, color_mode: flat}\n",
        errs);
    EXPECT_FALSE(bad.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("color_mode"), std::string::npos);
}

TEST(Profile, TrajectoryCarpetAdapterAcceptsRoleCarpetOnly) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /c, type: visualization_msgs/msg/MarkerArray,"
        " adapter: trajectory_carpet, role: carpet}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].role, "carpet");

    std::vector<std::string> bad_errs;
    auto bad = load_profile_string(
        "name: t\nrows:\n  - {topic: /c, type: visualization_msgs/msg/MarkerArray,"
        " adapter: trajectory_carpet, role: neutral}\n",
        bad_errs);
    EXPECT_FALSE(bad.has_value());
    ASSERT_FALSE(bad_errs.empty());
    EXPECT_NE(bad_errs[0].find("neutral"), std::string::npos);
}

TEST(Profile, TrajectoryCarpetAdapterRejectsWrongMessageType) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /c, type: nav_msgs/msg/Path,"
        " adapter: trajectory_carpet, role: carpet}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("trajectory_carpet"), std::string::npos);
}

TEST(Profile, DebugCruiseObstacleMarkerRowUsesGenericAdapter) {
    std::vector<std::string> errs;
    auto p = load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto* row = find_row(*p, "/navigation/debug_cruise_obstacle_marker");
    ASSERT_NE(row, nullptr);
    EXPECT_EQ(row->adapter, "generic");
    EXPECT_EQ(row->role, "neutral");
}

TEST(Profile, LocalMapCornersRowUsesGenericAdapter) {
    std::vector<std::string> errs;
    auto p = load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto* row = find_row(*p, "/local_map_corners");
    ASSERT_NE(row, nullptr);
    EXPECT_EQ(row->adapter, "generic");
    EXPECT_EQ(row->role, "neutral");
}

TEST(Profile, GroundTruthBoxesRowIsBestEffortBecauseItsPublisherIs) {
    std::vector<std::string> errs;
    auto p = load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(find_row(*p, "/sim/ground_truth/boxes"), nullptr);
    std::vector<std::string> errs2;
    auto p2 = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /sim/ground_truth/boxes, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, best_effort: true}\n",
        errs2);
    ASSERT_TRUE(p2.has_value());
    const auto* row = find_row(*p2, "/sim/ground_truth/boxes");
    ASSERT_NE(row, nullptr);
    EXPECT_TRUE(row->best_effort);
}

TEST(Profile, SubscriptionsForCarriesQosAndFansOutOgmRows) {
    std::vector<std::string> errs;

    auto p1 = load_profile_string(
        "name: t\nrows:\n  - {topic: /a, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, best_effort: true, transient_local: true}\n",
        errs);
    ASSERT_TRUE(p1.has_value());
    auto specs1 = subscriptions_for(p1->rows[0]);
    ASSERT_EQ(specs1.size(), 1u);
    EXPECT_EQ(specs1[0].topic, "/a");
    EXPECT_TRUE(specs1[0].best_effort);
    EXPECT_TRUE(specs1[0].transient_local);

    auto p2 = load_profile_string(
        "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
        " adapter: ogm, role: dynamic_ogm, update_topic: /g_updates,"
        " transient_local: true}\n",
        errs);
    ASSERT_TRUE(p2.has_value());
    auto specs2 = subscriptions_for(p2->rows[0]);
    ASSERT_EQ(specs2.size(), 2u);
    EXPECT_EQ(specs2[0].topic, "/g");
    EXPECT_TRUE(specs2[0].transient_local);
    EXPECT_EQ(specs2[1].topic, "/g_updates");
    EXPECT_EQ(specs2[1].type, "map_msgs/msg/OccupancyGridUpdate");
    EXPECT_FALSE(specs2[1].transient_local);

    auto p3 = load_profile_string("name: t\nrows:\n  - {adapter: tf_axes, role: debug}\n", errs);
    ASSERT_TRUE(p3.has_value());
    EXPECT_TRUE(subscriptions_for(p3->rows[0]).empty());
}

TEST(Profile, TfAxesRowHasNoTopicAndEverythingElseMustHaveOne) {
    std::vector<std::string> errs;
    auto with_topic = load_profile_string(
        "name: t\nrows:\n  - {topic: /tf_markers, adapter: tf_axes, role: debug}\n", errs);
    EXPECT_FALSE(with_topic.has_value());

    auto without_topic =
        load_profile_string("name: t\nrows:\n  - {adapter: tf_axes, role: debug}\n", errs);
    EXPECT_TRUE(without_topic.has_value());

    auto other_missing_topic = load_profile_string(
        "name: t\nrows:\n  - {type: nav_msgs/msg/Path, adapter: path, role: behavior}\n", errs);
    EXPECT_FALSE(other_missing_topic.has_value());
}

TEST(Profile, UnknownAdapterIsRejectedWithRowContext) {
    std::vector<std::string> errs;
    auto p = overlume::ros::load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: teleporter, role: lane}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("teleporter"), std::string::npos);
    EXPECT_NE(errs[0].find("/x"), std::string::npos);
}

TEST(Profile, MissingRequiredKeyIsRejected) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /needs_adapter, type: visualization_msgs/msg/MarkerArray,"
        " role: lane}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("adapter"), std::string::npos);
    EXPECT_NE(errs[0].find("/needs_adapter"), std::string::npos);
}

TEST(Profile, TimeoutBelowRenderFadeWindowIsRejected) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, timeout_sec: 0.4}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("timeout_sec"), std::string::npos);
    EXPECT_NE(errs[0].find("1.0"), std::string::npos);
}

TEST(Profile, AllErrorsReportedNotJustTheFirst) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: teleporter, role: "
        "lane}\n"
        "  - {topic: /y, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: "
        "neutral, timeout_sec: 0.1}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    EXPECT_EQ(errs.size(), 2u);
}

TEST(Profile, UnknownExtraKeyIsAWarningNotAHardFailure) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, tpyo_key: 1}\n",
        errs);
    ASSERT_TRUE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("tpyo_key"), std::string::npos);
}

TEST(Profile, DuplicateTopicAdapterPairIsRejected) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: "
        "neutral}\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: "
        "neutral}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    EXPECT_EQ(errs.size(), 1u);
}

TEST(Profile, DuplicateTopicAdapterPairNamesTheFileRowIndexNotTheSurvivingRowIndex) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /z, type: visualization_msgs/msg/MarkerArray, adapter: teleporter, role: "
        "lane}\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: "
        "neutral}\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: "
        "neutral}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 2u);
    EXPECT_NE(errs[1].find("row 2"), std::string::npos) << errs[1];
    EXPECT_EQ(errs[1].find("row 1"), std::string::npos) << errs[1];
}

TEST(Profile, RoleMustBeInTheAdapterClosedSet) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: collision, role: sweeep}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("sweeep"), std::string::npos);
}

TEST(Profile, MalformedScalarTypeIsReportedNotThrown) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, timeout_sec: abc}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
}

TEST(Profile, WholeFileScalarIsReportedNotThrown) {
    std::vector<std::string> errs;
    auto p = load_profile_string("just_a_scalar", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
}

TEST(Profile, ShippedProfilesCarryEveryCollisionPathAndOgmRow) {
    struct Expected {
        std::string topic;
        std::string adapter;
        std::string role;
    };
    const std::vector<Expected> kExpectedRows = {
        {"/behavior_path_planner/collision_markers", "collision", "collision"},
        {"/navigation_motion_obstacle_planner_node/collision_markers", "collision", "predicted"},
        {"/behavior_path_planner/output_path_visualization", "path", "behavior"},
        {"/local_vel_path", "path", "local"},
        {"/navigation/global_path", "path", "global"},
        {"/local_path", "path", "local"},
        {"/perception/dynamic_ogm", "ogm", "dynamic_ogm"},
        {"/perception/gradient_ogm", "ogm", "gradient_ogm"},
    };

    for (const char* profile_file :
         {"urban_profile.yaml", "offroad_profile.yaml", "sim_profile.yaml"}) {
        std::vector<std::string> errs;
        auto p =
            overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/" + profile_file, errs);
        ASSERT_TRUE(p.has_value()) << profile_file << ": " << (errs.empty() ? "" : errs[0]);
        for (const auto& exp : kExpectedRows) {
            const auto* row = find_row(*p, exp.topic);
            ASSERT_NE(row, nullptr) << profile_file << ": missing row for topic " << exp.topic;
            EXPECT_EQ(row->adapter, exp.adapter) << profile_file << ": topic " << exp.topic;
            EXPECT_EQ(row->role, exp.role) << profile_file << ": topic " << exp.topic;
        }

        for (const char* dead_topic :
             {"/navigation_urban_collision_checker_testing_node/collision_markers",
              "/navigation_urban_collision_checker_testing_node/object_predicted_polygons",
              "/navigation_urban_collision_checker_testing_node/ego_footprint_sweep",
              "/navigation_urban_collision_checker_testing_node/ego_merged_polygon",
              "/navigation_urban_collision_checker_testing_node/object_merged_polygons"}) {
            EXPECT_EQ(find_row(*p, dead_topic), nullptr)
                << profile_file << ": dead-namespace row must not still be live: " << dead_topic;
        }
    }
}

TEST(Profile, CoexistsWithTheRendererLibrarysOwnYamlCpp) {
    std::vector<std::string> errs;
    auto p =
        overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->rows.size(), 15u);
    EXPECT_TRUE(overlume::theme_parses(OVERLUME_THEME_DIR, "dark_adas"));
    EXPECT_FALSE(overlume::theme_parses(OVERLUME_THEME_DIR, "no_such_theme"));
    auto p2 = overlume::ros::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    EXPECT_TRUE(p2.has_value());
}

TEST(FixtureMsgs, DynamicObjectsListFixtureRoundTrips) {
    auto arr = overlume::ros::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    ASSERT_EQ(arr.markers.size(), 15u);
    EXPECT_EQ(arr.markers[0].ns, "");
    EXPECT_EQ(arr.markers[0].action, 3);
    EXPECT_EQ(arr.markers[1].ns, "dynamic_objects_bbox");
    EXPECT_EQ(arr.markers[1].type, 1);
    EXPECT_EQ(arr.markers[1].id, 1001);
    EXPECT_EQ(arr.markers[0].header.frame_id, "map");
}

TEST(Profile, FrameIdAndCostmapEncodingParseOnTheirAdapters) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /iv_points_fusion, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, frame_id: seyond}\n"
        "  - {topic: /perception/dynamic_costmap, type: nav_msgs/msg/OccupancyGrid,"
        " adapter: ogm, role: dynamic_ogm, encoding: costmap}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(errs.empty());
    EXPECT_EQ(p->rows[0].frame_id, "seyond");
    EXPECT_EQ(p->rows[1].encoding, "costmap");
    EXPECT_EQ(p->rows[0].encoding, "occupancy");
}

TEST(Profile, EncodingIsRejectedOffOgmRowsAndForUnknownValues) {
    std::vector<std::string> errs;
    EXPECT_FALSE(
        load_profile_string("name: t\nrows:\n  - {topic: /p, type: sensor_msgs/msg/PointCloud2,"
                            " adapter: point_cloud, role: points, encoding: costmap}\n",
                            errs)
            .has_value());
    errs.clear();
    EXPECT_FALSE(
        load_profile_string("name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
                            " adapter: ogm, role: dynamic_ogm, encoding: rgb}\n",
                            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("encoding"), std::string::npos);
}

TEST(Profile, ShippedRobotOffroadProfileTargetsTheRealRobotTopics) {
    std::vector<std::string> errs;
    auto p = overlume::ros::load_profile(
        std::string(TEST_CONFIG_DIR) + "/robot-offroad_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(errs.empty()) << errs[0];
    EXPECT_EQ(p->name, "robot-offroad");
    for (const char* topic : {"/perception/dynamic_costmap", "/perception/geometric_costmap"}) {
        const auto* row = find_row(*p, topic);
        ASSERT_NE(row, nullptr) << topic;
        EXPECT_EQ(row->adapter, "ogm") << topic;
        EXPECT_EQ(row->encoding, "costmap") << topic << ": the robot publishes Nav2 uint8 costs";
        EXPECT_TRUE(row->best_effort) << topic << ": the robot publishes costmaps BEST_EFFORT";
        EXPECT_TRUE(row->update_topic.empty()) << topic;
    }
    const auto* lidar = find_row(*p, "/iv_points_fusion");
    ASSERT_NE(lidar, nullptr);
    EXPECT_EQ(lidar->frame_id, "seyond")
        << "the driver stamps base_link but publishes in the 180-degree-yawed lidar frame";
    EXPECT_EQ(find_row(*p, "/perception/dynamic_ogm"), nullptr);
}
