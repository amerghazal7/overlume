/** @file test_collision_adapter.cpp
 *  @brief CollisionAdapter tests.
 *
 *  FIXTURE GAP 4 (epic2, superseded VM-077): the five collision-checker
 *  topics were silent in the recorded bag -- every fixture below is
 *  synthetic, hand-written. VM-077's measurement pass (2026-09-09) then
 *  confirmed the whole /navigation_urban_collision_checker_testing_node/*
 *  namespace dead stack-wide and retargeted/dormant'd every profile row
 *  onto it (see urban_profile.yaml). `severity_for_role()`'s five-role
 *  closed set is UNCHANGED by that remap (a dormant row's role stays legal
 *  so it has something to uncomment onto) -- these tests exercise the
 *  ADAPTER against hand-built rows (`MakeRow()` below), never `urban_row()`
 *  for a topic that no longer ships a live row.
 */
#include "overlume_ros/adapters/collision.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;

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

// VM-077: the dead-namespace topics these tests used to borrow via
// urban_row() no longer have a live row to fetch (2 retargeted onto new
// topic names, 3 dormant/commented) -- hand-built directly, same "no
// urban_row()/sim_row() to borrow" shape test_point_cloud_adapter.cpp's own
// MakeRow() already uses. The topic name itself is never read by
// CollisionAdapter (severity comes from `role` alone), so a placeholder is
// fine.
overlume_node::ProfileRow MakeRow(const std::string& role)
{
    overlume_node::ProfileRow row;
    row.topic = "/test/collision";
    row.type = "visualization_msgs/msg/MarkerArray";
    row.adapter = "collision";
    row.role = role;
    return row;
}

}  // namespace

// ── Step 1: every shipped role maps to its severity ─────────────────────────

TEST(CollisionAdapter, EveryShippedRoleMapsToItsSeverity)
{
    // Table-driven over the five roles severity_for_role() must accept
    // (profile.cpp's RoleSets closed set for adapter: collision) -- role
    // strings, not urban_row() lookups: VM-077 dormant'd three of these
    // roles' shipped rows (no live topic to fetch via urban_row() any
    // more), but the role itself stays legal so a future re-enable has
    // something to uncomment onto.
    struct Case
    {
        const char* role;
        uint8_t expected_severity;
    };
    const Case cases[] = {
        {"collision", 2},
        {"predicted", 1},
        {"merged_object", 1},
        {"sweep", 0},
        {"merged_ego", 0},
    };

    for (const auto& c : cases)
    {
        TfFixture kTf;
        auto row = MakeRow(c.role);
        overlume_node::CollisionAdapter a(row, kTf.tf);

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
        ASSERT_EQ(out.alerts.size(), 1u) << "role " << c.role;
        EXPECT_EQ(out.alerts[0].severity, c.expected_severity) << "role " << c.role;
    }
}

TEST(CollisionAdapter, UnknownRoleThrowsRatherThanDefaultingToInfo)
{
    // An unknown role reaching this adapter is a Task 1 profile-validator
    // bug (profile.cpp's RoleSets already rejects it for a REAL profile
    // file) -- this is the unit-level guard on severity_for_role() itself:
    // it must assert/throw, never silently return info (severity 0).
    EXPECT_THROW(overlume_node::severity_for_role("not_a_real_role"), std::invalid_argument);
}

// ── Step 1: open vs. producer-closed polylines both become CLOSED polygons ──

TEST(CollisionAdapter, OpenPolylineIsClosedIntoAPolygon)
{
    // collision_sweep_0.yaml's marker does NOT repeat its first point as
    // its last (4 distinct points) -- stored polygon must come out CLOSED
    // (first == last), point_count == 5.
    TfFixture kTf;
    auto row = MakeRow("sweep");
    overlume_node::CollisionAdapter open(row, kTf.tf);
    auto openMsg = overlume_node::testing::load_marker_array("collision_sweep_0.yaml");
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
    auto predictedRow = MakeRow("predicted");
    overlume_node::CollisionAdapter closed(predictedRow, kTf2.tf);
    auto closedMsg = overlume_node::testing::load_marker_array("collision_predicted_0.yaml");
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
    auto row = MakeRow("merged_object");
    overlume_node::CollisionAdapter a(row, kTf.tf);
    auto msg = overlume_node::testing::load_marker_array("collision_malformed_0.yaml");
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
    auto row = MakeRow("collision");
    overlume_node::CollisionAdapter a(row, kTf.tf);

    EXPECT_EQ(a.stats().msgs, 0u);
    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.alerts.size(), 0u);
}

// ── DELETEALL / DELETE, mirroring HdMapAdapter's own coverage ───────────────

TEST(CollisionAdapter, DeleteAllClearsPreviousPolygons)
{
    TfFixture kTf;
    auto row = MakeRow("sweep");
    overlume_node::CollisionAdapter a(row, kTf.tf);
    auto msg = overlume_node::testing::load_marker_array("collision_sweep_0.yaml");
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
    // Marker points are RELATIVE to marker.pose (rviz parity). A pose
    // translation plus an all-ZERO quaternion (which rviz forgives as
    // identity and tf2 would NaN) must land the polygon at the translated
    // coordinates, nothing dropped.
    TfFixture kTf;
    auto row = MakeRow("sweep");
    overlume_node::CollisionAdapter a(row, kTf.tf);
    auto msg = overlume_node::testing::load_marker_array("collision_sweep_0.yaml");
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
