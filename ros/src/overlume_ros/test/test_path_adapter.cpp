// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/path.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

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

nav_msgs::msg::Path MakePath(const std::vector<std::array<double, 3>>& xyz) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    for (const auto& p : xyz) {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = p[0];
        ps.pose.position.y = p[1];
        ps.pose.position.z = p[2];
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    return msg;
}

}  // namespace

TEST(PathAdapter, BehaviorPathHeadingDerivedFromPointsNotOrientation) {
    auto msg = overlume::ros::testing::load_path("behavior_output_path_0.yaml");
    ASSERT_GE(msg.poses.size(), 2u);
    TfFixture kTf;
    overlume::ros::PathAdapter a(
        overlume::ros::testing::urban_row("/behavior_path_planner/output_path_visualization"),
        kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    ASSERT_EQ(out.paths.size(), 1u);
    const overlume::PathRibbon& r = out.paths.front();
    ASSERT_EQ(r.point_count, msg.poses.size());
    for (uint32_t i = 0; i < r.point_count; ++i) {
        EXPECT_DOUBLE_EQ(r.points[i].x, msg.poses[i].pose.position.x);
        EXPECT_DOUBLE_EQ(r.points[i].y, msg.poses[i].pose.position.y);
        EXPECT_DOUBLE_EQ(r.points[i].z, 0.0);
    }
}

TEST(PathAdapter, RoleComesFromTheProfileRowNotTheTopicName) {
    TfFixture kTf;
    auto msg = MakePath({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}});

    overlume::ros::ProfileRow row =
        overlume::ros::testing::urban_row("/behavior_path_planner/output_path_visualization");
    ASSERT_EQ(row.role, "behavior");

    row.role = "behavior";
    {
        overlume::ros::PathAdapter a(row, kTf.tf);
        a.ingest(msg, 1.0);
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.paths.size(), 1u);
        EXPECT_EQ(out.paths.front().role, overlume::PathRole::BEHAVIOR);
    }
    row.role = "global";
    {
        overlume::ros::PathAdapter a(row, kTf.tf);
        a.ingest(msg, 1.0);
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.paths.size(), 1u);
        EXPECT_EQ(out.paths.front().role, overlume::PathRole::GLOBAL);
    }
    row.role = "local";
    {
        overlume::ros::PathAdapter a(row, kTf.tf);
        a.ingest(msg, 1.0);
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.paths.size(), 1u);
        EXPECT_EQ(out.paths.front().role, overlume::PathRole::LOCAL);
    }
}

TEST(PathAdapter, EmptyAndSinglePosePathsAreDroppedAndCounted) {
    TfFixture kTf;
    overlume::ros::PathAdapter a(overlume::ros::testing::urban_row("/local_vel_path"), kTf.tf);

    auto empty = MakePath({});
    a.ingest(empty, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
    SceneAssembly out1;
    a.fill(out1);
    EXPECT_TRUE(out1.paths.empty());

    auto single = MakePath({{5, 5, 0}});
    a.ingest(single, 2.0);
    EXPECT_EQ(a.stats().dropped_malformed, 2u);
    SceneAssembly out2;
    a.fill(out2);
    EXPECT_TRUE(out2.paths.empty());

    EXPECT_EQ(a.stats().msgs, 2u);
}

TEST(PathAdapter, PathChangeReplacesRatherThanAppends) {
    auto long_path = overlume::ros::testing::load_path("behavior_output_path_0.yaml");
    ASSERT_GT(long_path.poses.size(), 33u);
    TfFixture kTf;
    overlume::ros::PathAdapter a(
        overlume::ros::testing::urban_row("/behavior_path_planner/output_path_visualization"),
        kTf.tf);
    a.ingest(long_path, 1.0);
    SceneAssembly out1;
    a.fill(out1);
    ASSERT_EQ(out1.paths.size(), 1u);
    EXPECT_EQ(out1.paths.front().point_count, long_path.poses.size());

    auto short_path = overlume::ros::testing::load_path("local_vel_path_0.yaml");
    ASSERT_EQ(short_path.poses.size(), 33u);
    a.ingest(short_path, 2.0);
    SceneAssembly out2;
    a.fill(out2);
    ASSERT_EQ(out2.paths.size(), 1u);
    EXPECT_EQ(out2.paths.front().point_count, 33u);
}

TEST(PathAdapter, FillStampsLastUpdateSecAnchoredToRowTimeoutNotRawReceipt) {
    TfFixture kTf;
    const auto row =
        overlume::ros::testing::urban_row("/behavior_path_planner/output_path_visualization");
    overlume::ros::PathAdapter a(row, kTf.tf);
    a.ingest(MakePath({{0, 0, 0}, {5, 0, 0}}), 10.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.paths.size(), 1u);
    EXPECT_DOUBLE_EQ(out.paths.front().last_update_sec, 10.0 + (row.timeout_sec - 1.0));
    EXPECT_GE(row.timeout_sec, 1.0);
}
