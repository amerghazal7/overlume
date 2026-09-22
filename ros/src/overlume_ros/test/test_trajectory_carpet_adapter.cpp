// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/trajectory_carpet.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

overlume::ros::ProfileRow MakeRow() {
    overlume::ros::ProfileRow row;
    row.topic = "/navigation_motion_obstacle_planner_node/output_trajectory_carpet";
    row.type = "visualization_msgs/msg/MarkerArray";
    row.adapter = "trajectory_carpet";
    row.role = "carpet";
    row.timeout_sec = 2.0;
    return row;
}

visualization_msgs::msg::Marker MakeTriangleMarker(int32_t action, int32_t type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "output_trajectory_carpet";
    m.id = 0;
    m.action = action;
    m.type = type;
    return m;
}

geometry_msgs::msg::Point MakePoint(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::msg::ColorRGBA MakeColor(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kTypeTriangleList = 11;
constexpr int32_t kTypeLineStrip = 4;

visualization_msgs::msg::Marker MakeTwoQuadCarpetMarker() {
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    m.points = {
        MakePoint(0, 0, 0.15), MakePoint(0, 1, 0.15), MakePoint(1, 1, 0.15), MakePoint(0, 0, 0.15),
        MakePoint(1, 1, 0.15), MakePoint(1, 0, 0.15), MakePoint(1, 0, 0.15), MakePoint(1, 1, 0.15),
        MakePoint(2, 1, 0.15), MakePoint(1, 0, 0.15), MakePoint(2, 1, 0.15), MakePoint(2, 0, 0.15),
    };
    for (int i = 0; i < 12; ++i) {
        m.colors.push_back(MakeColor(static_cast<float>(i) * 0.05f, 0.5f, 0.0f, 0.7f));
    }
    return m;
}

}

TEST(TrajectoryCarpetAdapter, IngestExtractsCenterlineStationsFromDualRailQuadPairing) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume::ros::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {MakeTwoQuadCarpetMarker()};
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.trajectory_carpets.size(), 1u);
    ASSERT_EQ(out.trajectory_carpets[0].point_count, 3u);

    const auto* pts = out.trajectory_carpets[0].points;
    EXPECT_DOUBLE_EQ(pts[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(pts[0].position.y, 0.5);
    EXPECT_DOUBLE_EQ(pts[1].position.x, 1.0);
    EXPECT_DOUBLE_EQ(pts[1].position.y, 0.5);
    EXPECT_DOUBLE_EQ(pts[2].position.x, 2.0);
    EXPECT_DOUBLE_EQ(pts[2].position.y, 0.5);

    auto expected_r = [](int idx) {
        return static_cast<uint8_t>(static_cast<float>(idx) * 0.05f * 255.0f + 0.5f);
    };
    EXPECT_EQ(pts[0].rgba & 0xFFu, expected_r(0));
    EXPECT_EQ((pts[0].rgba >> 24) & 0xFFu, 255u);
    EXPECT_EQ(pts[1].rgba & 0xFFu, expected_r(5));
    EXPECT_EQ((pts[1].rgba >> 24) & 0xFFu, 255u);
    EXPECT_EQ(pts[2].rgba & 0xFFu, expected_r(11));
    EXPECT_EQ((pts[2].rgba >> 24) & 0xFFu, 255u);
}

TEST(TrajectoryCarpetAdapter, MismatchedColorsLengthBakesAlphaZeroSentinel) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume::ros::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTwoQuadCarpetMarker();
    m.colors = {MakeColor(1.0f, 0.0f, 0.0f, 0.7f)};
    arr.markers = {m};

    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.trajectory_carpets.size(), 1u);
    ASSERT_EQ(out.trajectory_carpets[0].point_count, 3u);
    for (uint32_t i = 0; i < 3; ++i) {
        EXPECT_EQ(out.trajectory_carpets[0].points[i].rgba, 0u)
            << "every station's packed rgba must be 0 (alpha byte 0) on a whole-message "
               "colors[]/points[] length mismatch, station "
            << i;
    }
}

TEST(TrajectoryCarpetAdapter, PointCountNotMultipleOfSixDropsMalformed) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume::ros::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    m.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0),
                MakePoint(0, 0, 0), MakePoint(1, 1, 0), MakePoint(1, 0, 0),
                MakePoint(1, 0, 0), MakePoint(1, 1, 0), MakePoint(2, 1, 0)};
    arr.markers = {m};

    a.ingest(arr, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.trajectory_carpets.size(), 0u);

    TfFixture kTf2;
    overlume::ros::TrajectoryCarpetAdapter b(row, kTf2.tf);
    visualization_msgs::msg::MarkerArray arr2;
    auto three = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    three.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0)};
    arr2.markers = {three};
    b.ingest(arr2, 1.0);
    EXPECT_EQ(b.stats().dropped_malformed, 1u);

    TfFixture kTf3;
    overlume::ros::TrajectoryCarpetAdapter c(row, kTf3.tf);
    visualization_msgs::msg::MarkerArray arr3;
    auto lineStrip = MakeTriangleMarker(kActionAdd, kTypeLineStrip);
    lineStrip.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0)};
    arr3.markers = {lineStrip};
    c.ingest(arr3, 1.0);
    EXPECT_EQ(c.stats().dropped_malformed, 1u);
}

TEST(TrajectoryCarpetAdapter, DeleteAllClearsStoredCarpet) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume::ros::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {MakeTwoQuadCarpetMarker()};
    a.ingest(arr, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.trajectory_carpets.size(), 1u);

    visualization_msgs::msg::MarkerArray deleteAll;
    auto d = MakeTriangleMarker(kActionDeleteAll, kTypeTriangleList);
    deleteAll.markers = {d};
    a.ingest(deleteAll, 2.0);

    SceneAssembly second;
    a.fill(second);
    EXPECT_EQ(second.trajectory_carpets.size(), 0u);
}

TEST(TrajectoryCarpetAdapter, ReplacesStoredCarpetWholesaleNotAppend) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume::ros::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray first_arr;
    first_arr.markers = {MakeTwoQuadCarpetMarker()};
    a.ingest(first_arr, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.trajectory_carpets[0].point_count, 3u);

    visualization_msgs::msg::MarkerArray second_arr;
    auto second_m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    second_m.points = {MakePoint(10, 0, 0), MakePoint(10, 1, 0), MakePoint(11, 1, 0),
                       MakePoint(10, 0, 0), MakePoint(11, 1, 0), MakePoint(11, 0, 0)};
    second_arr.markers = {second_m};
    a.ingest(second_arr, 2.0);

    SceneAssembly second;
    a.fill(second);
    ASSERT_EQ(second.trajectory_carpets.size(), 1u);
    EXPECT_EQ(second.trajectory_carpets[0].point_count, 2u)
        << "a second valid message must REPLACE the stored carpet, not append to it";
    EXPECT_DOUBLE_EQ(second.trajectory_carpets[0].points[0].position.x, 10.0);
    EXPECT_DOUBLE_EQ(second.trajectory_carpets[0].points[0].position.y, 0.5);
}

TEST(TrajectoryCarpetAdapter, NanCornerDropsWholeMessageKeepingThePreviousCarpet) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume::ros::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray first_arr;
    first_arr.markers = {MakeTwoQuadCarpetMarker()};
    a.ingest(first_arr, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.trajectory_carpets[0].point_count, 3u);

    visualization_msgs::msg::MarkerArray bad_arr;
    auto bad = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    bad.points = {MakePoint(0, 0, 0), MakePoint(0, 1, 0), MakePoint(1, 1, 0),
                  MakePoint(0, 0, 0), MakePoint(1, 1, 0), MakePoint(std::nan(""), 0, 0)};
    bad_arr.markers = {bad};
    a.ingest(bad_arr, 2.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);

    SceneAssembly second;
    a.fill(second);
    ASSERT_EQ(second.trajectory_carpets.size(), 1u);
    EXPECT_EQ(second.trajectory_carpets[0].point_count, 3u)
        << "a NaN-corner message must drop wholesale, leaving the previous valid carpet intact";
}
