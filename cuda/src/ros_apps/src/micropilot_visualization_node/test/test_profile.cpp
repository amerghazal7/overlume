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
    // sim_profile.yaml exists precisely so ProfileRow::transient_local has a
    // shipped user, a test, and a way to reach /sim/hd_map/markers -- the only
    // full-extent map source in the bag (1 msg, 3725 markers, TRANSIENT_LOCAL).
    // Without this file the field is dead config and a QoS bug in it ships
    // undetected until Epic 5's live validation.
    std::vector<std::string> errs;
    auto p = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    const auto* row = find_row(*p, "/sim/hd_map/markers");
    ASSERT_NE(row, nullptr);
    EXPECT_TRUE(row->transient_local);
}

TEST(Profile, NamespaceRuleIsLongestPrefixWins)
{
    // The whole ingest-decimation story rests on this one behaviour.
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
    // The gap this closes: a topic whose real namespaces are NOT the ones the
    // row's rules were written for renders as garbage, silently. Table-driven
    // over the namespaces actually observed on the wire (counts from a
    // full-bag deserialization pass), asserting the row's classify() verdict.
    std::vector<std::string> errs;
    auto urban = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(urban.has_value());
    auto sim = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    ASSERT_TRUE(sim.has_value());

    const auto* road_markers = find_row(*urban, "/road_markers");
    ASSERT_NE(road_markers, nullptr);
    EXPECT_EQ(classify(*road_markers, "road_lane_left_boundary"), NsRender::kPolyline);
    EXPECT_EQ(classify(*road_markers, "road_lane_right_boundary"), NsRender::kPolyline);
    EXPECT_EQ(classify(*road_markers, "some_unknown_ns"), NsRender::kDrop);  // ns_default

    const auto* hd_map_global = find_row(*urban, "/hd_map_global_elements");
    ASSERT_NE(hd_map_global, nullptr);
    EXPECT_EQ(classify(*hd_map_global, "centerline_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(*hd_map_global, "centerline_arrows_0"), NsRender::kDrop);

    const auto* hd_map_local = find_row(*urban, "/hd_map_local_elements");
    ASSERT_NE(hd_map_local, nullptr);
    EXPECT_EQ(classify(*hd_map_local, "centerline_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(*hd_map_local, "centerline_arrows_0"), NsRender::kDrop);
    EXPECT_EQ(classify(*hd_map_local, "left_boundary_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(*hd_map_local, "right_boundary_0"), NsRender::kPolyline);
    EXPECT_EQ(classify(*hd_map_local, "crosswalk_7"), NsRender::kPolygon);
    EXPECT_EQ(classify(*hd_map_local, "crosswalk_stopline_7"), NsRender::kPolyline);

    const auto* dyn_objects = find_row(*urban, "/perception/dynamic_objects_list");
    ASSERT_NE(dyn_objects, nullptr);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_bbox"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_text"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_arrow"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_hd_map_path"), NsRender::kPolyline);
    EXPECT_EQ(classify(*dyn_objects, "dynamic_objects_hd_map_path_dots"), NsRender::kDrop);

    const auto* sim_hd_map = find_row(*sim, "/sim/hd_map/markers");
    ASSERT_NE(sim_hd_map, nullptr);
    EXPECT_EQ(classify(*sim_hd_map, "centerline_0"), NsRender::kPolyline);
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

TEST(Profile, GroundTruthBoxesRowIsBestEffortBecauseItsPublisherIs)
{
    // The bag's metadata.yaml records offered `reliability: 2` (BEST_EFFORT)
    // for /sim/ground_truth/boxes. An rclcpp subscription defaults to
    // RELIABLE, which NEVER matches a BEST_EFFORT publisher -- no error, no
    // warning, a permanently silent topic. This row is the epic's only
    // base_link row and the subject of Task 8's parity E2E, whose own rclpy
    // publisher is RELIABLE, so nothing else in this epic can catch it.
    std::vector<std::string> errs;
    auto p = load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value());
    const auto* row = find_row(*p, "/sim/ground_truth/boxes");
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
    // stream is inherently VOLATILE (each patch supersedes the last), so a
    // TRANSIENT_LOCAL-requesting subscriber would never match a VOLATILE
    // update publisher and the patch stream would go silently dead. See
    // profile.cpp's subscriptions_for().
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
    // timeout_sec: 0.4 -> rejected; error explains the >= 1.0 s rule
    // (the renderer's own fade runs 0.5..1.0 s; see "Staleness" above)
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
    // VM-042: profiles are hand-edited by the autonomy team; a typo'd
    // optional key must not take the node down.
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
    // Row 0 is rejected (bad adapter) and never makes it into profile.rows.
    // Rows 1 and 2 are the actual duplicate pair. profile.rows ends up
    // holding them at surviving-indices 0 and 1 -- the message must still
    // name FILE row 2 (the second, offending occurrence), not row 1 (which
    // is what the surviving-index would misreport, and which is also the
    // innocent, first-seen row of the pair).
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
    // A hand-edited profile (VM-042) with a mis-typed scalar must produce an
    // error + nullopt, not let YAML::TypedBadConversion escape and abort the
    // process. Exercises ParseRow's node[...].as<T>() calls directly.
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
    // Task 1's other two file-level tests (ShippedUrbanProfileLoads,
    // ShippedProfilesRouteEveryKnownNamespaceOfEveryShippedTopic) only catch
    // a MALFORMED row, never a MISSING one -- "file parses" and "rows
    // non-empty" both still pass if a row is silently deleted. The plan
    // ships the 5 collision + 4 path + 2 ogm rows in Task 1 specifically so
    // Task 7 lands severities/adapters against rows something already
    // validated, not against a profile whose completeness nothing checked.
    // Table-driven over (topic, adapter, role) closes that gap.
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

// ── Step 0.3: coexisting yaml-cpps (see scene.h / this task's Step 0) ───────
TEST(Profile, CoexistsWithTheRendererLibrarysOwnYamlCpp)
{
    // No GPU needed, no GTEST_SKIP: if this test can be skipped it is not a
    // guard. Vendor yaml-cpp (gcc/libstdc++) parses a profile...
    std::vector<std::string> errs;
    auto p = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/urban_profile.yaml", errs);
    ASSERT_TRUE(p.has_value());
    // Row COUNT, not just has_value(): merge_yamlcpp.sh recipe 3 (see that
    // script's header) LINKED fine and still returned a Profile with the
    // right error count (0) but the WRONG row count (0 instead of 16) --
    // an ABI-mismatched YAML::Node silently corrupting data, not crashing.
    // has_value() alone would pass on that broken build.
    EXPECT_EQ(p->rows.size(), 16u);
    // ...and the bundled yaml-cpp (clang/libc++), inside libvisual_renderer.a,
    // parses a theme in the SAME process. If the two ever get relinked into
    // one, this is where it shows up -- not in a field crash three epics later.
    EXPECT_TRUE(mpviz::theme_parses(MPVIZ_THEME_DIR, "dark_adas"));
    EXPECT_FALSE(mpviz::theme_parses(MPVIZ_THEME_DIR, "no_such_theme"));
    // ...and the vendor copy still works afterwards (ordering-sensitive
    // static state is the failure mode a single call would miss).
    auto p2 = mpviz_node::load_profile(std::string(TEST_CONFIG_DIR) + "/sim_profile.yaml", errs);
    EXPECT_TRUE(p2.has_value());
}

// ── Step 5 self-check: bag_to_fixture.py's committed output round-trips ────
TEST(FixtureMsgs, DynamicObjectsListFixtureRoundTrips)
{
    // Committed by scripts/bag_to_fixture.py against the real bag (see that
    // script's docstring). Ground truth taken from the same fixture file at
    // authoring time: 15 markers in message 0, first is the ns=""
    // action=DELETEALL marker, second is a dynamic_objects_bbox CUBE.
    auto arr = mpviz_node::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    ASSERT_EQ(arr.markers.size(), 15u);
    EXPECT_EQ(arr.markers[0].ns, "");
    EXPECT_EQ(arr.markers[0].action, 3);
    EXPECT_EQ(arr.markers[1].ns, "dynamic_objects_bbox");
    EXPECT_EQ(arr.markers[1].type, 1);  // CUBE
    EXPECT_EQ(arr.markers[1].id, 1001);
    EXPECT_EQ(arr.markers[0].header.frame_id, "map");
}
