// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

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

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

overlume::ros::ProfileRow MakeRow(const std::string& role) {
    overlume::ros::ProfileRow row;
    row.topic = "/test/collision";
    row.type = "visualization_msgs/msg/MarkerArray";
    row.adapter = "collision";
    row.role = role;
    return row;
}

}  // namespace

TEST(CollisionAdapter, EveryShippedRoleMapsToItsSeverity) {
    struct Case {
        const char* role;
        uint8_t expected_severity;
    };
    const Case cases[] = {
        {"collision", 2}, {"predicted", 1}, {"merged_object", 1}, {"sweep", 0}, {"merged_ego", 0},
    };

    for (const auto& c : cases) {
        TfFixture kTf;
        auto row = MakeRow(c.role);
        overlume::ros::CollisionAdapter a(row, kTf.tf);

        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.ns = "test";
        m.id = 1;
        m.type = 4;
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

TEST(CollisionAdapter, UnknownRoleThrowsRatherThanDefaultingToInfo) {
    EXPECT_THROW(overlume::ros::severity_for_role("not_a_real_role"), std::invalid_argument);
}

TEST(CollisionAdapter, OpenPolylineIsClosedIntoAPolygon) {
    TfFixture kTf;
    auto row = MakeRow("sweep");
    overlume::ros::CollisionAdapter open(row, kTf.tf);
    auto openMsg = overlume::ros::testing::load_marker_array("collision_sweep_0.yaml");
    open.ingest(openMsg, 1.0);
    SceneAssembly openOut;
    open.fill(openOut);
    ASSERT_EQ(openOut.alerts.size(), 1u);
    ASSERT_EQ(openOut.alerts[0].point_count, 5u);
    EXPECT_DOUBLE_EQ(openOut.alerts[0].points[0].x, openOut.alerts[0].points[4].x);
    EXPECT_DOUBLE_EQ(openOut.alerts[0].points[0].y, openOut.alerts[0].points[4].y);

    TfFixture kTf2;
    auto predictedRow = MakeRow("predicted");
    overlume::ros::CollisionAdapter closed(predictedRow, kTf2.tf);
    auto closedMsg = overlume::ros::testing::load_marker_array("collision_predicted_0.yaml");
    closed.ingest(closedMsg, 1.0);
    SceneAssembly closedOut;
    closed.fill(closedOut);
    ASSERT_EQ(closedOut.alerts.size(), 1u);
    ASSERT_EQ(closedOut.alerts[0].point_count, 5u);
    EXPECT_DOUBLE_EQ(closedOut.alerts[0].points[0].x, closedOut.alerts[0].points[4].x);
    EXPECT_DOUBLE_EQ(closedOut.alerts[0].points[0].y, closedOut.alerts[0].points[4].y);
}

TEST(CollisionAdapter, DegenerateAndNaNPolygonsDroppedAndCounted) {
    TfFixture kTf;
    auto row = MakeRow("merged_object");
    overlume::ros::CollisionAdapter a(row, kTf.tf);
    auto msg = overlume::ros::testing::load_marker_array("collision_malformed_0.yaml");
    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_malformed, 2u);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.alerts.size(), 1u) << "the valid neighbour must still come through";
    EXPECT_EQ(out.alerts[0].severity, 1u);
}

TEST(CollisionAdapter, SilentTopicYieldsZeroAlertsAndDoesNotWedge) {
    TfFixture kTf;
    auto row = MakeRow("collision");
    overlume::ros::CollisionAdapter a(row, kTf.tf);

    EXPECT_EQ(a.stats().msgs, 0u);
    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.alerts.size(), 0u);
}

TEST(CollisionAdapter, DeleteAllClearsPreviousPolygons) {
    TfFixture kTf;
    auto row = MakeRow("sweep");
    overlume::ros::CollisionAdapter a(row, kTf.tf);
    auto msg = overlume::ros::testing::load_marker_array("collision_sweep_0.yaml");
    a.ingest(msg, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.alerts.size(), 1u);

    visualization_msgs::msg::MarkerArray deleteAll;
    visualization_msgs::msg::Marker d;
    d.action = 3;
    deleteAll.markers = {d};
    a.ingest(deleteAll, 2.0);

    SceneAssembly second;
    a.fill(second);
    EXPECT_EQ(second.alerts.size(), 0u);
}

TEST(CollisionAdapter, MarkerPoseComposesAndZeroQuaternionIsIdentity) {
    TfFixture kTf;
    auto row = MakeRow("sweep");
    overlume::ros::CollisionAdapter a(row, kTf.tf);
    auto msg = overlume::ros::testing::load_marker_array("collision_sweep_0.yaml");
    ASSERT_FALSE(msg.markers.empty());
    const double base_x = msg.markers[0].points[0].x;
    const double base_y = msg.markers[0].points[0].y;
    msg.markers[0].pose.position.x = 10.0;
    msg.markers[0].pose.position.y = 20.0;
    msg.markers[0].pose.orientation.w = 0.0;

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    ASSERT_EQ(out.alerts.size(), 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
    EXPECT_DOUBLE_EQ(out.alerts[0].points[0].x, base_x + 10.0);
    EXPECT_DOUBLE_EQ(out.alerts[0].points[0].y, base_y + 20.0);
}
