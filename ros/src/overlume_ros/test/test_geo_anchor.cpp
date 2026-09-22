// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/geo_anchor.hpp"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

using overlume::ros::ClassifyGeoDatum;
using overlume::ros::GeoAnchor;
using overlume::ros::GeoAnchorSolver;
using overlume::ros::GeoDatumOverride;
using overlume::ros::GreatCircleDistanceM;
using overlume::ros::kMinAnchorBaselineM;
using overlume::ros::kMinAnchorSamples;
using overlume::ros::MapToWgs;
using overlume::ros::SolveAnchor;
using overlume::ros::WgsToMap;

TEST(GeoAnchor, RoundTripWgsToMapAndBackWithin0p1mOver2km) {
    GeoAnchor a{25.0803, 55.3910, 0.0};
    for (auto [dlat, dlon] : {std::pair{0.0, 0.0}, {0.01, 0.0}, {0.0, 0.01}, {-0.015, 0.02}}) {
        const double lat = a.origin_lat_deg + dlat, lon = a.origin_lon_deg + dlon;
        const overlume::Vec3 xy = WgsToMap(a, lat, lon);
        const auto [lat2, lon2] = MapToWgs(a, xy);
        EXPECT_LT(GreatCircleDistanceM(lat, lon, lat2, lon2), 0.1);
    }
}

TEST(GeoAnchor, RoundTripHoldsWithNonzeroHeading) {
    GeoAnchor a{25.0803, 55.3910, 0.35};
    const overlume::Vec3 xy = WgsToMap(a, 25.0850, 55.3950);
    const auto [lat2, lon2] = MapToWgs(a, xy);
    EXPECT_LT(GreatCircleDistanceM(25.0850, 55.3950, lat2, lon2), 0.1);
}

namespace {

struct SamplePairs {
    std::vector<std::pair<double, double>> fixes;
    std::vector<std::pair<double, double>> map_xy;
};

SamplePairs LoadFixture(const std::string& path) {
    SamplePairs out;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tok;
        std::vector<double> vals;
        while (std::getline(ss, tok, ',')) vals.push_back(std::stod(tok));
        if (vals.size() != 4) continue;
        out.fixes.emplace_back(vals[0], vals[1]);
        out.map_xy.emplace_back(vals[2], vals[3]);
    }
    return out;
}

}  // namespace

TEST(GeoAnchor, SolveAnchorMatchesBagDerivedMeanPositionAndCoarseHeading) {
    const std::string path = std::string(OVERLUME_NODE_FIXTURES_DIR) + "/geo_anchor_samples_0.csv";
    const SamplePairs samples = LoadFixture(path);
    ASSERT_GE(samples.fixes.size(), 50u) << "fixture " << path << " missing/short";

    const GeoAnchor anchor = SolveAnchor(samples.fixes, samples.map_xy);

    for (std::size_t i = 0; i < samples.fixes.size(); ++i) {
        const auto& [lat, lon] = samples.fixes[i];
        const auto& [mx, my] = samples.map_xy[i];
        const overlume::Vec3 got = WgsToMap(anchor, lat, lon);
        EXPECT_LT(std::hypot(got.x - mx, got.y - my), 1.0)
            << "sample " << i << " residual too large";
    }

    const auto& [lat1, lon1] = samples.fixes.front();
    const auto& [lat2, lon2] = samples.fixes.back();
    const auto& [mx1, my1] = samples.map_xy.front();
    const auto& [mx2, my2] = samples.map_xy.back();
    const double phi1 = lat1 * M_PI / 180.0, phi2 = lat2 * M_PI / 180.0;
    const double dlon = (lon2 - lon1) * M_PI / 180.0;
    const double bearing = std::atan2(
        std::sin(dlon) * std::cos(phi2),
        std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlon));
    const double angle_map = std::atan2(my2 - my1, mx2 - mx1);
    double expected_heading = bearing + angle_map;
    while (expected_heading > M_PI) expected_heading -= 2.0 * M_PI;
    while (expected_heading <= -M_PI) expected_heading += 2.0 * M_PI;

    double diff = anchor.heading_rad - expected_heading;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff <= -M_PI) diff += 2.0 * M_PI;
    EXPECT_LT(std::abs(diff) * 180.0 / M_PI, 5.0) << "heading diverged by more than a few degrees";
}

namespace {

geometry_msgs::msg::TransformStamped MapToBaseLink(double x, double y) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    t.child_frame_id = "base_link";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    t.transform.rotation = tf2::toMsg(q);
    return t;
}

sensor_msgs::msg::NavSatFix Fix(double lat, double lon) {
    sensor_msgs::msg::NavSatFix f;
    f.latitude = lat;
    f.longitude = lon;
    f.altitude = 11.0;
    f.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    return f;
}

}  // namespace

TEST(GeoAnchorSolver, OverrideSolvesImmediatelyWithoutAnyOnFixCall) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");

    solver.set_override(10.0, 20.0, 45.0);
    ASSERT_TRUE(solver.solved());
    const GeoAnchor a = solver.anchor();
    EXPECT_DOUBLE_EQ(a.origin_lat_deg, 10.0);
    EXPECT_DOUBLE_EQ(a.origin_lon_deg, 20.0);
    EXPECT_DOUBLE_EQ(a.heading_rad, 45.0 * M_PI / 180.0);
}

TEST(GeoAnchorSolver, OnFixAfterOverrideIsIgnored) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    buffer.setTransform(MapToBaseLink(1.0, 2.0), "test_authority", true);
    GeoAnchorSolver solver(buffer, "map", "base_link");

    solver.set_override(10.0, 20.0, 45.0);
    for (int i = 0; i < 5; ++i) solver.on_fix(Fix(25.08, 55.39));

    const GeoAnchor a = solver.anchor();
    EXPECT_DOUBLE_EQ(a.origin_lat_deg, 10.0);
    EXPECT_DOUBLE_EQ(a.origin_lon_deg, 20.0);
}

TEST(GeoAnchorSolver, FewerThanMinSamplesLeavesUnsolved) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    buffer.setTransform(MapToBaseLink(1.0, 2.0), "test_authority", true);
    GeoAnchorSolver solver(buffer, "map", "base_link");

    for (uint32_t i = 0; i < kMinAnchorSamples - 1; ++i) solver.on_fix(Fix(25.08, 55.39));
    EXPECT_FALSE(solver.solved());
}

TEST(GeoAnchorSolver, ReachingMinSamplesWithSufficientBaselineSolves) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");

    ASSERT_LT(kMinAnchorBaselineM, static_cast<double>(kMinAnchorSamples));
    for (uint32_t i = 0; i < kMinAnchorSamples; ++i) {
        buffer.setTransform(MapToBaseLink(static_cast<double>(i), 2.0), "test_authority", true);
        solver.on_fix(Fix(25.08, 55.39));
    }
    EXPECT_TRUE(solver.solved());
}

TEST(GeoAnchorSolver, ReachingMinSamplesWithZeroBaselineStaysUnsolved) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    buffer.setTransform(MapToBaseLink(1.0, 2.0), "test_authority", true);
    GeoAnchorSolver solver(buffer, "map", "base_link");

    for (uint32_t i = 0; i < kMinAnchorSamples + 10; ++i) solver.on_fix(Fix(25.08, 55.39));
    EXPECT_FALSE(solver.solved());
}

TEST(GeoAnchorSolver, ZeroFixesAndZeroTfLeavesUnsolvedNoCrash) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");
    EXPECT_FALSE(solver.solved());

    for (int i = 0; i < 10; ++i) solver.on_fix(Fix(25.08, 55.39));
    EXPECT_FALSE(solver.solved());
}

TEST(ClassifyGeoDatum, AllThreeFiniteIsComplete) {
    EXPECT_EQ(ClassifyGeoDatum(25.08, 55.39, 45.0), GeoDatumOverride::Complete);
}

TEST(ClassifyGeoDatum, AllThreeNanIsNone) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(ClassifyGeoDatum(nan, nan, nan), GeoDatumOverride::None);
}

TEST(ClassifyGeoDatum, ExactlyOneFiniteIsPartial) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(ClassifyGeoDatum(25.08, nan, nan), GeoDatumOverride::Partial);
    EXPECT_EQ(ClassifyGeoDatum(nan, 55.39, nan), GeoDatumOverride::Partial);
    EXPECT_EQ(ClassifyGeoDatum(nan, nan, 45.0), GeoDatumOverride::Partial);
}

TEST(ClassifyGeoDatum, ExactlyTwoFiniteIsPartial) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(ClassifyGeoDatum(25.08, 55.39, nan), GeoDatumOverride::Partial);
    EXPECT_EQ(ClassifyGeoDatum(25.08, nan, 45.0), GeoDatumOverride::Partial);
    EXPECT_EQ(ClassifyGeoDatum(nan, 55.39, 45.0), GeoDatumOverride::Partial);
}

TEST(GeoAnchorSolver, SolvedAnchorOriginHeightIsMeanAcceptedAltitude) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");

    ASSERT_LT(kMinAnchorBaselineM, static_cast<double>(kMinAnchorSamples));
    for (uint32_t i = 0; i < kMinAnchorSamples; ++i) {
        buffer.setTransform(MapToBaseLink(static_cast<double>(i), 2.0), "test_authority", true);
        sensor_msgs::msg::NavSatFix fix = Fix(25.08, 55.39);
        fix.altitude = (i == 0) ? std::numeric_limits<double>::quiet_NaN() : 1.7;
        solver.on_fix(fix);
    }
    ASSERT_TRUE(solver.solved());
    EXPECT_NEAR(solver.anchor().origin_height_m, 1.7, 1e-9);
}

TEST(GeoAnchorSolver, ChooseAnchorHeightAppliesOverrideThenOffset) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NEAR(overlume::ros::ChooseAnchorHeightM(nan, 1.7, -0.3), 1.4, 1e-12);
    EXPECT_NEAR(overlume::ros::ChooseAnchorHeightM(25.0, 1.7, -0.3), 24.7, 1e-12);
    EXPECT_NEAR(overlume::ros::ChooseAnchorHeightM(nan, 0.0, 0.5), 0.5, 1e-12);
}

TEST(GeoAnchorSolver, PartialGeoDatumNeverAppliedLeavesSolverUnsolved) {
    ASSERT_EQ(ClassifyGeoDatum(25.08, std::numeric_limits<double>::quiet_NaN(), 45.0),
              GeoDatumOverride::Partial);
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");
    EXPECT_FALSE(solver.solved());
}
