/** @file test_profile.cpp
 *  @brief Profile YAML loader tests (Epic 2 Task 1 / VM-020).
 */
#include "micropilot_visualization_node/profile.hpp"

#include <gtest/gtest.h>

#include "visual_renderer/scene.h"
#include "fixture_msgs.hpp"

using mpviz_node::classify;
using mpviz_node::find_row;
using mpviz_node::load_profile;
using mpviz_node::load_profile_string;
using mpviz_node::match_rule;
using mpviz_node::NsRender;
using mpviz_node::subscriptions_for;

TEST(Profile, ShippedUrbanProfileLoads)
{
    std::vector<std::string> errs;
    auto p = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(errs.empty());
    EXPECT_FALSE(p->rows.empty());
}

TEST(Profile, ShippedOffroadProfileLoads)
{
    std::vector<std::string> errs;
    auto p = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/offroad_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(errs.empty());
    EXPECT_FALSE(p->rows.empty());
}

TEST(Profile, ShippedSimProfileLoadsAndMarksTheLatchedHdMapRow)
{
    // sim_profile.yaml is the only shipped profile with a TRANSIENT_LOCAL
    // hd_map row (/sim/hd_map/markers); this pins that QoS bit.
    std::vector<std::string> errs;
    auto p = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto* row = find_row(*p, "/sim/hd_map/markers");
    ASSERT_NE(row, nullptr);
    EXPECT_TRUE(row->transient_local);
}

TEST(Profile, NamespaceRuleIsLongestPrefixWins)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, ns_default: drop, namespaces:"
        " [{prefix: centerline_, render: polyline},"
        "  {prefix: centerline_arrows_, render: drop},"
        "  {prefix: crosswalk_, render: polygon},"
        "  {prefix: crosswalk_stopline_, render: polyline}]}\n", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto& r = p->rows[0];
    EXPECT_EQ(classify(r, "centerline_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(r, "centerline_arrows_0"), NsRender::kDrop);  // longer prefix wins
    EXPECT_EQ(classify(r, "crosswalk_7"), NsRender::kPolygon);
    EXPECT_EQ(classify(r, "crosswalk_stopline_7"), NsRender::kPolyline);
    EXPECT_EQ(classify(r, "traffic_light_2"), NsRender::kDrop);  // ns_default
}

TEST(Profile, KindParsesOnNamespaceRules)
{
    // kind is per-rule (not inherited by a sibling rule) and defaults to
    // OTHER when the key is absent.
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_,    render: polyline, kind: centerline},"
        "  {prefix: left_boundary_, render: polyline, kind: left_boundary},"
        "  {prefix: junk_,          render: polyline}]}\n", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    ASSERT_EQ(p->rows[0].namespaces.size(), 3u);
    EXPECT_EQ(p->rows[0].namespaces[0].kind, mpviz::MapKind::CENTERLINE);
    EXPECT_EQ(p->rows[0].namespaces[1].kind, mpviz::MapKind::LEFT_BOUNDARY);
    EXPECT_EQ(p->rows[0].namespaces[2].kind, mpviz::MapKind::OTHER);
}

TEST(Profile, KindIsRejectedWhenIllegalForRender)
{
    // kind: crosswalk is polygon-only; every lane-geometry kind is
    // polyline-only.
    std::vector<std::string> polyline_errs;
    auto polyline = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: crosswalk_, render: polyline, kind: crosswalk}]}\n", polyline_errs);
    EXPECT_FALSE(polyline.has_value());
    ASSERT_EQ(polyline_errs.size(), 1u);
    EXPECT_NE(polyline_errs[0].find("kind"), std::string::npos);
    EXPECT_NE(polyline_errs[0].find("crosswalk_"), std::string::npos);

    std::vector<std::string> polygon_errs;
    auto polygon = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_, render: polygon, kind: centerline}]}\n", polygon_errs);
    EXPECT_FALSE(polygon.has_value());
    ASSERT_EQ(polygon_errs.size(), 1u);
    EXPECT_NE(polygon_errs[0].find("kind"), std::string::npos);
}

TEST(Profile, UnknownKindValueIsRejectedWithRowContext)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_, render: polyline, kind: teleporter}]}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("teleporter"), std::string::npos);
}

TEST(Profile, ShippedUrbanLocalRowMarksCenterlineAndBoundaryKinds)
{
    // centerline_ and left/right_boundary_ carry distinct kinds so the
    // renderer can dash boundaries and keep centerline solid.
    std::vector<std::string> errs;
    auto p = load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto* row = find_row(*p, "/hd_map_local_elements");
    ASSERT_NE(row, nullptr);

    // centerline_ is render: drop, and kind can't be set on a drop rule, so
    // it stays the OTHER default.
    const auto* centerline_rule = match_rule(*row, "centerline_0");
    ASSERT_NE(centerline_rule, nullptr);
    EXPECT_EQ(centerline_rule->render, NsRender::kDrop);
    EXPECT_EQ(centerline_rule->kind, mpviz::MapKind::OTHER);

    const auto* left_rule = match_rule(*row, "left_boundary_0");
    ASSERT_NE(left_rule, nullptr);
    EXPECT_EQ(left_rule->render, NsRender::kPolyline);
    EXPECT_EQ(left_rule->kind, mpviz::MapKind::LEFT_BOUNDARY);

    const auto* right_rule = match_rule(*row, "right_boundary_0");
    ASSERT_NE(right_rule, nullptr);
    EXPECT_EQ(right_rule->render, NsRender::kPolyline);
    EXPECT_EQ(right_rule->kind, mpviz::MapKind::RIGHT_BOUNDARY);

    const auto* crosswalk_rule = match_rule(*row, "crosswalk_7");
    ASSERT_NE(crosswalk_rule, nullptr);
    EXPECT_EQ(crosswalk_rule->render, NsRender::kPolygon);
    EXPECT_EQ(crosswalk_rule->kind, mpviz::MapKind::CROSSWALK);

    const auto* stopline_rule = match_rule(*row, "crosswalk_stopline_7");
    ASSERT_NE(stopline_rule, nullptr);
    EXPECT_EQ(stopline_rule->render, NsRender::kPolyline);
    EXPECT_EQ(stopline_rule->kind, mpviz::MapKind::STOPLINE);
}

TEST(Profile, DuplicateNamespacePrefixIsRejected)
{
    // two rules with the same prefix have no defined winner -> bad row
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /m, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, namespaces:"
        " [{prefix: centerline_, render: polyline},"
        "  {prefix: centerline_, render: drop}]}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("duplicate"), std::string::npos);
    EXPECT_NE(errs[0].find("centerline_"), std::string::npos);
}

TEST(Profile, ShippedProfilesRouteEveryKnownNamespaceOfEveryShippedTopic)
{
    // Table-driven over the namespaces actually observed on the wire,
    // asserting each row's classify() verdict -- a topic whose real
    // namespaces don't match its rules renders as garbage, silently.
    std::vector<std::string> errs;
    auto urban = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(urban.has_value());
    auto sim = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    ASSERT_TRUE(sim.has_value());

    // /road_markers row is disabled (bad upstream publisher data); on
    // re-enable, restore its classify() assertions from git history and
    // bump the row count below.
    EXPECT_EQ(find_row(*urban, "/road_markers"), nullptr);

    const auto* hd_map_global = find_row(*urban, "/hd_map_global_elements");
    ASSERT_NE(hd_map_global, nullptr);
    // centerline_ hidden by default.
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
    // The sim publisher uses plural "crosswalks" (no numeric suffix); the
    // bare "crosswalk" prefix matches both spellings, while the longer
    // crosswalk_stopline_ rule still wins.
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
    // centerline_ hidden by default.
    EXPECT_EQ(classify(*sim_hd_map, "centerline_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*sim_hd_map, "centerline_arrows_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*sim_hd_map, "crosswalks"), NsRender::kPolygon);  // note: no numeric suffix
    EXPECT_EQ(classify(*sim_hd_map, "junction"), NsRender::kPolyline);
    EXPECT_EQ(classify(*sim_hd_map, "landmark"), NsRender::kDrop);
    EXPECT_EQ(classify(*sim_hd_map, "landmark_text"), NsRender::kDrop);
}

TEST(Profile, OgmRowCarriesItsUpdateTopicAndNonOgmRowsMayNot)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
        " adapter: ogm, role: dynamic_ogm, update_topic: /g_updates}\n", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].update_topic, "/g_updates");

    std::vector<std::string> errs2;
    auto bad = load_profile_string(
        "name: t\nrows:\n  - {topic: /p, type: nav_msgs/msg/Path,"
        " adapter: path, role: behavior, update_topic: /p_updates}\n", errs2);
    EXPECT_FALSE(bad.has_value());
    ASSERT_EQ(errs2.size(), 1u);
    EXPECT_NE(errs2[0].find("update_topic"), std::string::npos);
}

TEST(Profile, JunctionInteriorBoundariesDefaultsToTrueAndParsesExplicitFalse)
{
    // Defaults to true; shipped profiles never write this key explicitly.
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /h, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane}\n", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(p->rows[0].junction_interior_boundaries);

    std::vector<std::string> errs2;
    auto p2 = load_profile_string(
        "name: t\nrows:\n  - {topic: /h, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, junction_interior_boundaries: false}\n", errs2);
    ASSERT_TRUE(p2.has_value()) << (errs2.empty() ? "" : errs2[0]);
    EXPECT_FALSE(p2->rows[0].junction_interior_boundaries);
}

TEST(Profile, JunctionInteriorBoundariesIsRejectedOnNonHdMapRows)
{
    // Restricted to adapter: hd_map; an explicit value elsewhere is a typo'd
    // key -- nothing else reads it.
    std::vector<std::string> errs;
    auto bad = load_profile_string(
        "name: t\nrows:\n  - {topic: /p, type: nav_msgs/msg/Path,"
        " adapter: path, role: behavior, junction_interior_boundaries: false}\n", errs);
    EXPECT_FALSE(bad.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("junction_interior_boundaries"), std::string::npos);
}

// ── Epic 3 Task 6 (VM-035): adapter: point_cloud row fields ─────────────────

TEST(Profile, PointCloudRowDefaultsColorModeAutoStrideOne)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points}\n", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].color_mode, "auto");
    EXPECT_EQ(p->rows[0].max_points, 0u);
    EXPECT_EQ(p->rows[0].stride, 1u);
}

TEST(Profile, PointCloudRowParsesColorModeMaxPointsStride)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, color_mode: height, max_points: 5000,"
        " stride: 4}\n", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].color_mode, "height");
    EXPECT_EQ(p->rows[0].max_points, 5000u);
    EXPECT_EQ(p->rows[0].stride, 4u);
}

TEST(Profile, PointCloudRowRejectsUnknownColorMode)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, color_mode: rainbow}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("color_mode"), std::string::npos);
}

TEST(Profile, PointCloudRowRejectsZeroStride)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /lidar/points, type: sensor_msgs/msg/PointCloud2,"
        " adapter: point_cloud, role: points, stride: 0}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("stride"), std::string::npos);
}

TEST(Profile, ColorModeMaxPointsStrideAreRejectedOnNonPointCloudRows)
{
    std::vector<std::string> errs;
    auto bad = load_profile_string(
        "name: t\nrows:\n  - {topic: /p, type: nav_msgs/msg/Path,"
        " adapter: path, role: behavior, color_mode: flat}\n", errs);
    EXPECT_FALSE(bad.has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("color_mode"), std::string::npos);
}

TEST(Profile, GroundTruthBoxesRowIsBestEffortBecauseItsPublisherIs)
{
    // The bag's /sim/ground_truth/boxes publisher is BEST_EFFORT; an rclcpp
    // subscription defaults to RELIABLE and would never match it, silently.
    // Row ships disabled (the ego's own box sits at the robot's origin,
    // flickering).
    std::vector<std::string> errs;
    auto p = load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(find_row(*p, "/sim/ground_truth/boxes"), nullptr);
    // Confirm best_effort: true still parses correctly, independent of the
    // row's disabled state.
    std::vector<std::string> errs2;
    auto p2 = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /sim/ground_truth/boxes, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, best_effort: true}\n", errs2);
    ASSERT_TRUE(p2.has_value());
    const auto* row = find_row(*p2, "/sim/ground_truth/boxes");
    ASSERT_NE(row, nullptr);
    EXPECT_TRUE(row->best_effort);
}

TEST(Profile, SubscriptionsForCarriesQosAndFansOutOgmRows)
{
    std::vector<std::string> errs;

    // generic/hd_map/path/collision row -> 1 spec, qos flags copied through
    auto p1 = load_profile_string(
        "name: t\nrows:\n  - {topic: /a, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, best_effort: true, transient_local: true}\n", errs);
    ASSERT_TRUE(p1.has_value());
    auto specs1 = subscriptions_for(p1->rows[0]);
    ASSERT_EQ(specs1.size(), 1u);
    EXPECT_EQ(specs1[0].topic, "/a");
    EXPECT_TRUE(specs1[0].best_effort);
    EXPECT_TRUE(specs1[0].transient_local);

    // ogm row -> 2 specs (topic + update_topic, the second typed
    // map_msgs/msg/OccupancyGridUpdate). transient_local: true on the row
    // must reach the base-grid spec but NOT the update spec: an update
    // stream is inherently VOLATILE, so a TRANSIENT_LOCAL subscriber would
    // never match a VOLATILE publisher and the patch stream would go
    // silently dead.
    auto p2 = load_profile_string(
        "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
        " adapter: ogm, role: dynamic_ogm, update_topic: /g_updates,"
        " transient_local: true}\n", errs);
    ASSERT_TRUE(p2.has_value());
    auto specs2 = subscriptions_for(p2->rows[0]);
    ASSERT_EQ(specs2.size(), 2u);
    EXPECT_EQ(specs2[0].topic, "/g");
    EXPECT_TRUE(specs2[0].transient_local);
    EXPECT_EQ(specs2[1].topic, "/g_updates");
    EXPECT_EQ(specs2[1].type, "map_msgs/msg/OccupancyGridUpdate");
    EXPECT_FALSE(specs2[1].transient_local);

    // tf_axes row -> 0 specs (nothing publishes TF as markers; it is a
    // producer)
    auto p3 = load_profile_string("name: t\nrows:\n  - {adapter: tf_axes, role: debug}\n", errs);
    ASSERT_TRUE(p3.has_value());
    EXPECT_TRUE(subscriptions_for(p3->rows[0]).empty());
}

TEST(Profile, TfAxesRowHasNoTopicAndEverythingElseMustHaveOne)
{
    std::vector<std::string> errs;
    auto with_topic = load_profile_string(
        "name: t\nrows:\n  - {topic: /tf_markers, adapter: tf_axes, role: debug}\n", errs);
    EXPECT_FALSE(with_topic.has_value());

    auto without_topic = load_profile_string(
        "name: t\nrows:\n  - {adapter: tf_axes, role: debug}\n", errs);
    EXPECT_TRUE(without_topic.has_value());

    auto other_missing_topic = load_profile_string(
        "name: t\nrows:\n  - {type: nav_msgs/msg/Path, adapter: path, role: behavior}\n", errs);
    EXPECT_FALSE(other_missing_topic.has_value());
}

TEST(Profile, UnknownAdapterIsRejectedWithRowContext)
{
    std::vector<std::string> errs;
    auto p = mpviz_node::load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: teleporter, role: lane}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("teleporter"), std::string::npos);  // names the bad value
    EXPECT_NE(errs[0].find("/x"), std::string::npos);          // names the row
}

TEST(Profile, MissingRequiredKeyIsRejected)
{
    // row with no `adapter:` -> error mentions "adapter" and the row's topic
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /needs_adapter, type: visualization_msgs/msg/MarkerArray,"
        " role: lane}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("adapter"), std::string::npos);
    EXPECT_NE(errs[0].find("/needs_adapter"), std::string::npos);
}

TEST(Profile, TimeoutBelowRenderFadeWindowIsRejected)
{
    // timeout_sec must be >= 1.0s: the renderer's fade window ends at
    // kStaleFadeTimeoutSec (cuda/src/libs/visual_renderer/src/renderer_internal.hpp).
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, timeout_sec: 0.4}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("timeout_sec"), std::string::npos);
    EXPECT_NE(errs[0].find("1.0"), std::string::npos);
}

TEST(Profile, AllErrorsReportedNotJustTheFirst)
{
    // two bad rows -> errs.size() == 2 (a config file with three mistakes
    // should take one edit pass, not three)
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: teleporter, role: lane}\n"
        "  - {topic: /y, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: neutral, timeout_sec: 0.1}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    EXPECT_EQ(errs.size(), 2u);
}

TEST(Profile, UnknownExtraKeyIsAWarningNotAHardFailure)
{
    // Profiles are hand-edited; a typo'd optional key must not take the
    // node down.
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, tpyo_key: 1}\n", errs);
    ASSERT_TRUE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("tpyo_key"), std::string::npos);
}

TEST(Profile, DuplicateTopicAdapterPairIsRejected)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: neutral}\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: neutral}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    EXPECT_EQ(errs.size(), 1u);
}

TEST(Profile, DuplicateTopicAdapterPairNamesTheFileRowIndexNotTheSurvivingRowIndex)
{
    // Row 0 is rejected (bad adapter) and never enters profile.rows. Rows 1
    // and 2 are the actual duplicate pair, surviving at indices 0 and 1 --
    // but the error must still name FILE row 2 (the offending occurrence),
    // not the surviving-index or row 1 (the innocent first occurrence).
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /z, type: visualization_msgs/msg/MarkerArray, adapter: teleporter, role: lane}\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: neutral}\n"
        "  - {topic: /x, type: visualization_msgs/msg/MarkerArray, adapter: generic, role: neutral}\n",
        errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 2u);  // the bad-adapter row, then the duplicate
    EXPECT_NE(errs[1].find("row 2"), std::string::npos) << errs[1];
    EXPECT_EQ(errs[1].find("row 1"), std::string::npos) << errs[1];
}

TEST(Profile, RoleMustBeInTheAdapterClosedSet)
{
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: collision, role: sweeep}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_EQ(errs.size(), 1u);
    EXPECT_NE(errs[0].find("sweeep"), std::string::npos);
}

TEST(Profile, MalformedScalarTypeIsReportedNotThrown)
{
    // A mis-typed scalar must produce an error + nullopt, not let
    // YAML::TypedBadConversion escape and abort the process.
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: bad\nrows:\n  - {topic: /x, type: visualization_msgs/msg/MarkerArray,"
        " adapter: generic, role: neutral, timeout_sec: abc}\n", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
}

TEST(Profile, WholeFileScalarIsReportedNotThrown)
{
    // A profile that is just a bare scalar (no mapping at all) must not let
    // YAML::BadSubscript escape from BuildProfile's root["name"]/root["rows"].
    std::vector<std::string> errs;
    auto p = load_profile_string("just_a_scalar", errs);
    EXPECT_FALSE(p.has_value());
    ASSERT_FALSE(errs.empty());
}

TEST(Profile, ShippedProfilesCarryEveryCollisionPathAndOgmRow)
{
    // The other file-level tests only catch a MALFORMED row, never a
    // MISSING one -- "file parses" and "rows non-empty" both still pass if a
    // row is silently deleted. Table-driven over (topic, adapter, role)
    // closes that gap.
    struct Expected
    {
        std::string topic;
        std::string adapter;
        std::string role;
    };
    const std::vector<Expected> kExpectedRows = {
        // 5 collision rows
        {"/navigation_urban_collision_checker_testing_node/collision_markers", "collision",
         "collision"},
        {"/navigation_urban_collision_checker_testing_node/object_predicted_polygons",
         "collision", "predicted"},
        {"/navigation_urban_collision_checker_testing_node/ego_footprint_sweep", "collision",
         "sweep"},
        {"/navigation_urban_collision_checker_testing_node/ego_merged_polygon", "collision",
         "merged_ego"},
        {"/navigation_urban_collision_checker_testing_node/object_merged_polygons", "collision",
         "merged_object"},
        // 4 path rows
        {"/behavior_path_planner/output_path_visualization", "path", "behavior"},
        {"/local_vel_path", "path", "local"},
        {"/navigation/global_path", "path", "global"},
        {"/local_path", "path", "local"},
        // 2 ogm rows
        {"/perception/dynamic_ogm", "ogm", "dynamic_ogm"},
        {"/perception/gradient_ogm", "ogm", "gradient_ogm"},
    };

    for (const char* profile_file : {"urban_profile.yaml", "offroad_profile.yaml", "sim_profile.yaml"}) {
        std::vector<std::string> errs;
        auto p = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/" + profile_file, errs);
        ASSERT_TRUE(p.has_value()) << profile_file << ": " << (errs.empty() ? "" : errs[0]);
        for (const auto& exp : kExpectedRows) {
            const auto* row = find_row(*p, exp.topic);
            ASSERT_NE(row, nullptr) << profile_file << ": missing row for topic " << exp.topic;
            EXPECT_EQ(row->adapter, exp.adapter) << profile_file << ": topic " << exp.topic;
            EXPECT_EQ(row->role, exp.role) << profile_file << ": topic " << exp.topic;
        }
    }
}

// ── Coexisting yaml-cpp builds (vendor + bundled) ───────────────────────────
TEST(Profile, CoexistsWithTheRendererLibrarysOwnYamlCpp)
{
    // No GPU needed, no GTEST_SKIP: if this test can be skipped it is not a
    // guard. Vendor yaml-cpp (gcc/libstdc++) parses a profile...
    std::vector<std::string> errs;
    auto p = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value());
    // Row COUNT, not just has_value(): an ABI-mismatched YAML::Node can link
    // fine and still return a Profile with the right error count (0) but the
    // wrong row count -- silently corrupting data, not crashing.
    // 15, not 17: /road_markers and /sim/ground_truth/boxes ship disabled
    // (see above); +1 for the /iv_points_fusion point_cloud row (2026-09-09).
    // Bump when a row is added or a disabled one re-enabled.
    EXPECT_EQ(p->rows.size(), 15u);
    // ...and the bundled yaml-cpp (clang/libc++), inside libvisual_renderer.a,
    // parses a theme in the SAME process. If the two ever get relinked into
    // one, this is where it shows up.
    EXPECT_TRUE(mpviz::theme_parses(MPVIZ_THEME_DIR, "dark_adas"));
    EXPECT_FALSE(mpviz::theme_parses(MPVIZ_THEME_DIR, "no_such_theme"));
    // ...and the vendor copy still works afterwards (ordering-sensitive
    // static state is the failure mode a single call would miss).
    auto p2 = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    EXPECT_TRUE(p2.has_value());
}

// ── Fixture round-trip: bag_to_fixture.py's committed output ───────────────
TEST(FixtureMsgs, DynamicObjectsListFixtureRoundTrips)
{
    // Fixture generated by scripts/bag_to_fixture.py from the real bag;
    // message 0 has 15 markers -- first is the ns="" action=DELETEALL
    // marker, second is a dynamic_objects_bbox CUBE.
    auto arr = mpviz_node::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    ASSERT_EQ(arr.markers.size(), 15u);
    EXPECT_EQ(arr.markers[0].ns, "");
    EXPECT_EQ(arr.markers[0].action, 3);
    EXPECT_EQ(arr.markers[1].ns, "dynamic_objects_bbox");
    EXPECT_EQ(arr.markers[1].type, 1);  // CUBE
    EXPECT_EQ(arr.markers[1].id, 1001);
    EXPECT_EQ(arr.markers[0].header.frame_id, "map");
}
