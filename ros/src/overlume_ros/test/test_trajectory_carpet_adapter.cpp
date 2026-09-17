// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

/** @file test_trajectory_carpet_adapter.cpp
 *  @brief TrajectoryCarpetAdapter tests (VM-077, REDIRECTED 2026-09-10).
 *
 *  Measured against the real recorded output_trajectory_carpet (VM-077
 *  measurement report) -- every fixture below reproduces that shape by hand
 *  (TRIANGLE_LIST, one persistent ns='output_trajectory_carpet' id=0
 *  marker, DELETE_ALL + one ADD per tick, always a multiple of 6 points:
 *  dual-rail quads, two triangles each). See the adapter's own header
 *  comment for the full station-extraction pairing algorithm this file
 *  pins down.
 */
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

// Hand-built tf2_ros::Buffer + FrameTransformer -- an empty buffer is
// enough for every fixture below (stays in the "map" frame, identity
// shortcut). Same fixture style as test_collision_adapter.cpp/
// test_point_cloud_adapter.cpp.
struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

// No shipped profile row is needed by these tests -- hand-built directly,
// same "no urban_row()/sim_row() to borrow" shape test_point_cloud_adapter.
// cpp already uses for its own row.
overlume_node::ProfileRow MakeRow() {
    overlume_node::ProfileRow row;
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

// Two dual-rail quads (12 points, n_quads=2 -> 3 stations), straight along
// +X at y=0.5, mirroring the measurement report's own pairing example:
// quad_k = [A,B,C,A,C,D]; quad_0.{D,C} == quad_1.{A,B} IN VALUE (not
// array-index sharing -- the wire re-emits them, per the measurement's own
// "n_quads*6 points" arithmetic).
visualization_msgs::msg::Marker MakeTwoQuadCarpetMarker() {
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    m.points = {
        MakePoint(0, 0, 0.15), MakePoint(0, 1, 0.15), MakePoint(1, 1, 0.15),  // quad0 tri A,B,C
        MakePoint(0, 0, 0.15), MakePoint(1, 1, 0.15), MakePoint(1, 0, 0.15),  // quad0 tri A,C,D
        MakePoint(1, 0, 0.15), MakePoint(1, 1, 0.15), MakePoint(2, 1, 0.15),  // quad1 tri A,B,C
        MakePoint(1, 0, 0.15), MakePoint(2, 1, 0.15), MakePoint(2, 0, 0.15),  // quad1 tri A,C,D
    };
    for (int i = 0; i < 12; ++i) {
        m.colors.push_back(MakeColor(static_cast<float>(i) * 0.05f, 0.5f, 0.0f, 0.7f));
    }
    return m;
}

}  // namespace

// ── Station extraction: centerline midpoint + per-station color pairing ───

TEST(TrajectoryCarpetAdapter, IngestExtractsCenterlineStationsFromDualRailQuadPairing) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {MakeTwoQuadCarpetMarker()};
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.trajectory_carpets.size(), 1u);
    // n_quads=2 -> n_quads+1=3 stations, NOT the raw 12 wire points.
    ASSERT_EQ(out.trajectory_carpets[0].point_count, 3u);

    const auto* pts = out.trajectory_carpets[0].points;
    // station_0 = midpoint(A_0=(0,0), B_0=(0,1)) = (0, 0.5).
    EXPECT_DOUBLE_EQ(pts[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(pts[0].position.y, 0.5);
    // station_1 = midpoint(D_0=(1,0), C_0=(1,1)) = (1, 0.5).
    EXPECT_DOUBLE_EQ(pts[1].position.x, 1.0);
    EXPECT_DOUBLE_EQ(pts[1].position.y, 0.5);
    // station_2 = midpoint(D_1=(2,0), C_1=(2,1)) = (2, 0.5).
    EXPECT_DOUBLE_EQ(pts[2].position.x, 2.0);
    EXPECT_DOUBLE_EQ(pts[2].position.y, 0.5);

    // Color pairing: station_0 from colors[0] (A_0); station_1 from
    // colors[5] (D_0, quad0's 6th point, index base+5=5); station_2 from
    // colors[11] (D_1, quad1's 6th point, index base+5=11). Alpha forced
    // to 255 regardless of the marker's own colors[].a (documented "not
    // yet used").
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

// ── Mismatched colors[]/points[] lengths -> alpha==0 sentinel ───────────────

TEST(TrajectoryCarpetAdapter, MismatchedColorsLengthBakesAlphaZeroSentinel) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTwoQuadCarpetMarker();
    m.colors = {MakeColor(1.0f, 0.0f, 0.0f, 0.7f)};  // 1 color, 12 points -- mismatched
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

// ── Point count not a multiple of 6 (or wrong type) is dropped ─────────────

TEST(TrajectoryCarpetAdapter, PointCountNotMultipleOfSixDropsMalformed) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    // 9 points: a multiple of 3 (would have been legal under the OLD
    // %3==0 rule) but NOT of 6 -- must be rejected under the new rule.
    m.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0),
                MakePoint(0, 0, 0), MakePoint(1, 1, 0), MakePoint(1, 0, 0),
                MakePoint(1, 0, 0), MakePoint(1, 1, 0), MakePoint(2, 1, 0)};
    arr.markers = {m};

    a.ingest(arr, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.trajectory_carpets.size(), 0u);

    // Fewer than 6 points (a single triangle, the old %3 floor) is also
    // malformed now -- one quad is the minimum unit.
    TfFixture kTf2;
    overlume_node::TrajectoryCarpetAdapter b(row, kTf2.tf);
    visualization_msgs::msg::MarkerArray arr2;
    auto three = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    three.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0)};
    arr2.markers = {three};
    b.ingest(arr2, 1.0);
    EXPECT_EQ(b.stats().dropped_malformed, 1u);

    // A non-TRIANGLE_LIST type is malformed too -- this adapter has no
    // representation for anything else.
    TfFixture kTf3;
    overlume_node::TrajectoryCarpetAdapter c(row, kTf3.tf);
    visualization_msgs::msg::MarkerArray arr3;
    auto lineStrip = MakeTriangleMarker(kActionAdd, kTypeLineStrip);
    lineStrip.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0)};
    arr3.markers = {lineStrip};
    c.ingest(arr3, 1.0);
    EXPECT_EQ(c.stats().dropped_malformed, 1u);
}

// ── DELETE_ALL clears the stored carpet ─────────────────────────────────────

TEST(TrajectoryCarpetAdapter, DeleteAllClearsStoredCarpet) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume_node::TrajectoryCarpetAdapter a(row, kTf.tf);

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

// ── A second ingest() REPLACES wholesale, never appends ────────────────────

TEST(TrajectoryCarpetAdapter, ReplacesStoredCarpetWholesaleNotAppend) {
    // Same PathAdapter-style "REPLACES, never merges" contract -- two
    // ingest() calls, second's content is what fill() emits, not a union.
    TfFixture kTf;
    auto row = MakeRow();
    overlume_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray first_arr;
    first_arr.markers = {MakeTwoQuadCarpetMarker()};  // 2 quads -> 3 stations
    a.ingest(first_arr, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.trajectory_carpets[0].point_count, 3u);

    visualization_msgs::msg::MarkerArray second_arr;
    auto second_m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    second_m.points = {MakePoint(10, 0, 0), MakePoint(10, 1, 0), MakePoint(11, 1, 0),
                       MakePoint(10, 0, 0), MakePoint(11, 1, 0), MakePoint(11, 0, 0)};  // 1 quad
    second_arr.markers = {second_m};
    a.ingest(second_arr, 2.0);

    SceneAssembly second;
    a.fill(second);
    ASSERT_EQ(second.trajectory_carpets.size(), 1u);
    EXPECT_EQ(second.trajectory_carpets[0].point_count, 2u)
        << "a second valid message must REPLACE the stored carpet, not append to it";
    // station_0 = midpoint((10,0),(10,1)) = (10, 0.5).
    EXPECT_DOUBLE_EQ(second.trajectory_carpets[0].points[0].position.x, 10.0);
    EXPECT_DOUBLE_EQ(second.trajectory_carpets[0].points[0].position.y, 0.5);
}

// ── A NaN corner drops the whole message, previous carpet keeps rendering ──

TEST(TrajectoryCarpetAdapter, NanCornerDropsWholeMessageKeepingThePreviousCarpet) {
    TfFixture kTf;
    auto row = MakeRow();
    overlume_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray first_arr;
    first_arr.markers = {MakeTwoQuadCarpetMarker()};
    a.ingest(first_arr, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.trajectory_carpets[0].point_count, 3u);

    visualization_msgs::msg::MarkerArray bad_arr;
    auto bad = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    bad.points = {MakePoint(0, 0, 0), MakePoint(0, 1, 0),
                  MakePoint(1, 1, 0), MakePoint(0, 0, 0),
                  MakePoint(1, 1, 0), MakePoint(std::nan(""), 0, 0)};  // D corner is NaN
    bad_arr.markers = {bad};
    a.ingest(bad_arr, 2.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);

    SceneAssembly second;
    a.fill(second);
    ASSERT_EQ(second.trajectory_carpets.size(), 1u);
    EXPECT_EQ(second.trajectory_carpets[0].point_count, 3u)
        << "a NaN-corner message must drop wholesale, leaving the previous valid carpet intact";
}
