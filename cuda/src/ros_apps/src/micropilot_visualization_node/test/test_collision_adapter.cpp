/** @file test_collision_adapter.cpp
 *  @brief CollisionAdapter tests (Epic 2 Task 7 / VM-026).
 *
 *  FIXTURE GAP 4: the five collision-checker topics were silent in the
 *  recorded bag (a calm scenario, zero messages) -- every fixture below is
 *  synthetic, hand-written, unvalidated against a live publisher.
 */
#include "micropilot_visualization_node/adapters/collision.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using micropilot::visualization_app::FrameTransformer;
using micropilot::visualization_app::SceneAssembly;

namespace
{

// Hand-built tf2_ros::Buffer + FrameTransformer -- an empty buffer is
// enough for every test whose markers stay in the "map" frame
// (FrameTransformer's identity shortcut never touches it). Same fixture
// style as test_hd_map_adapter.cpp's TfFixture.
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

}  // namespace

// ── Step 1: every shipped role maps to its severity ─────────────────────────

TEST(CollisionAdapter, EveryShippedRoleMapsToItsSeverity)
{
    // Table-driven over the five ROWS SHIPPED IN THE REAL urban_profile.yaml
    // (loaded via urban_row(), never a hand-written role string) -- an
    // adapter test that invents its own role strings cannot notice the
    // profile drifting away from this table (epic2 plan, Task 7 Step 1).
    struct Case
    {
        const char* topic;
        uint8_t expected_severity;
    };
    const Case cases[] = {
        {"/navigation_urban_collision_checker_testing_node/collision_markers", 2},
        {"/navigation_urban_collision_checker_testing_node/object_predicted_polygons", 1},
        {"/navigation_urban_collision_checker_testing_node/object_merged_polygons", 1},
        {"/navigation_urban_collision_checker_testing_node/ego_footprint_sweep", 0},
        {"/navigation_urban_collision_checker_testing_node/ego_merged_polygon", 0},
    };

    for (const auto& c : cases)
    {
        TfFixture kTf;
        auto row = mpviz_node::testing::urban_row(c.topic);
        mpviz_node::CollisionAdapter a(row, kTf.tf);

        // One trivial valid triangle -- proves the FULL path (ctor's
        // severity_for_role() -> storage -> fill()), not just the free
        // function in isolation.
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.ns = "test";
        m.id = 1;
        m.type = 4;  // LINE_STRIP
        m.action = 0;
        geometry_msgs::msg::Point p0, p1, p2;
        p1.x = 1.0;
        p2.x = 1.0;
        p2.y = 1.0;
        m.points = {p0, p1, p2};
        arr.markers = {m};

        a.ingest(arr, 1.0);
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.alerts.size(), 1u) << "topic " << c.topic;
        EXPECT_EQ(out.alerts[0].severity, c.expected_severity) << "topic " << c.topic;
    }
}

TEST(CollisionAdapter, UnknownRoleThrowsRatherThanDefaultingToInfo)
{
    // An unknown role reaching this adapter is a Task 1 profile-validator
    // bug (profile.cpp's RoleSets already rejects it for a REAL profile
    // file) -- this is the unit-level guard on severity_for_role() itself:
    // it must assert/throw, never silently return info (severity 0).
    EXPECT_THROW(mpviz_node::severity_for_role("not_a_real_role"), std::invalid_argument);
}

// ── Step 1: open vs. producer-closed polylines both become CLOSED polygons ──

TEST(CollisionAdapter, OpenPolylineIsClosedIntoAPolygon)
{
    // collision_sweep_0.yaml's marker does NOT repeat its first point as
    // its last (4 distinct points) -- stored polygon must come out CLOSED
    // (first == last), point_count == 5.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row(
        "/navigation_urban_collision_checker_testing_node/ego_footprint_sweep");
    mpviz_node::CollisionAdapter open(row, kTf.tf);
    auto openMsg = mpviz_node::testing::load_marker_array("collision_sweep_0.yaml");
    open.ingest(openMsg, 1.0);
    SceneAssembly openOut;
    open.fill(openOut);
    ASSERT_EQ(openOut.alerts.size(), 1u);
    ASSERT_EQ(openOut.alerts[0].point_count, 5u);
    EXPECT_DOUBLE_EQ(openOut.alerts[0].points[0].x, openOut.alerts[0].points[4].x);
    EXPECT_DOUBLE_EQ(openOut.alerts[0].points[0].y, openOut.alerts[0].points[4].y);

    // collision_predicted_0.yaml's marker DOES repeat its first point as
    // its last (5 points on the wire, 4 distinct) -- must come out
    // IDENTICALLY shaped: point_count == 5, no doubled closing vertex.
    TfFixture kTf2;
    auto predictedRow = mpviz_node::testing::urban_row(
        "/navigation_urban_collision_checker_testing_node/object_predicted_polygons");
    mpviz_node::CollisionAdapter closed(predictedRow, kTf2.tf);
    auto closedMsg = mpviz_node::testing::load_marker_array("collision_predicted_0.yaml");
    closed.ingest(closedMsg, 1.0);
    SceneAssembly closedOut;
    closed.fill(closedOut);
    ASSERT_EQ(closedOut.alerts.size(), 1u);
    ASSERT_EQ(closedOut.alerts[0].point_count, 5u);
    EXPECT_DOUBLE_EQ(closedOut.alerts[0].points[0].x, closedOut.alerts[0].points[4].x);
    EXPECT_DOUBLE_EQ(closedOut.alerts[0].points[0].y, closedOut.alerts[0].points[4].y);
}

// ── Step 1: degenerate and NaN polygons dropped and counted ─────────────────

TEST(CollisionAdapter, DegenerateAndNaNPolygonsDroppedAndCounted)
{
    // collision_malformed_0.yaml: a NaN-point marker, a degenerate ring
    // (only 2 distinct points once its own producer-repeated closing point
    // is stripped), and one valid neighbour in between them.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row(
        "/navigation_urban_collision_checker_testing_node/object_merged_polygons");
    mpviz_node::CollisionAdapter a(row, kTf.tf);
    auto msg = mpviz_node::testing::load_marker_array("collision_malformed_0.yaml");
    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_malformed, 2u);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.alerts.size(), 1u) << "the valid neighbour must still come through";
    EXPECT_EQ(out.alerts[0].severity, 1u);  // merged_object -> warning
}

// ── Step 1: a silent topic yields zero alerts and does not wedge ───────────

TEST(CollisionAdapter, SilentTopicYieldsZeroAlertsAndDoesNotWedge)
{
    // FIXTURE GAP 4: this is the recorded-stack REALITY, not an error path
    // -- every collision topic published zero messages in the bag.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row(
        "/navigation_urban_collision_checker_testing_node/collision_markers");
    mpviz_node::CollisionAdapter a(row, kTf.tf);

    EXPECT_EQ(a.stats().msgs, 0u);
    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.alerts.size(), 0u);
}

// ── DELETEALL / DELETE, mirroring HdMapAdapter's own coverage ───────────────

TEST(CollisionAdapter, DeleteAllClearsPreviousPolygons)
{
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row(
        "/navigation_urban_collision_checker_testing_node/ego_footprint_sweep");
    mpviz_node::CollisionAdapter a(row, kTf.tf);
    auto msg = mpviz_node::testing::load_marker_array("collision_sweep_0.yaml");
    a.ingest(msg, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.alerts.size(), 1u);

    visualization_msgs::msg::MarkerArray deleteAll;
    visualization_msgs::msg::Marker d;
    d.action = 3;  // DELETEALL
    deleteAll.markers = {d};
    a.ingest(deleteAll, 2.0);

    SceneAssembly second;
    a.fill(second);
    EXPECT_EQ(second.alerts.size(), 0u);
}

TEST(CollisionAdapter, MarkerPoseComposesAndZeroQuaternionIsIdentity)
{
    // Review 2026-08-20: the pose-composition path (Marker points are
    // RELATIVE to marker.pose, rviz parity) had hd_map coverage but none
    // here. A pose translation plus an all-ZERO quaternion (which rviz
    // forgives as identity and tf2 would NaN) must land the polygon at the
    // translated coordinates, nothing dropped.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row(
        "/navigation_urban_collision_checker_testing_node/ego_footprint_sweep");
    mpviz_node::CollisionAdapter a(row, kTf.tf);
    auto msg = mpviz_node::testing::load_marker_array("collision_sweep_0.yaml");
    ASSERT_FALSE(msg.markers.empty());
    const double base_x = msg.markers[0].points[0].x;
    const double base_y = msg.markers[0].points[0].y;
    msg.markers[0].pose.position.x = 10.0;
    msg.markers[0].pose.position.y = 20.0;
    msg.markers[0].pose.orientation.w = 0.0;  // all-zero quaternion

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    ASSERT_EQ(out.alerts.size(), 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
    EXPECT_DOUBLE_EQ(out.alerts[0].points[0].x, base_x + 10.0);
    EXPECT_DOUBLE_EQ(out.alerts[0].points[0].y, base_y + 20.0);
}
