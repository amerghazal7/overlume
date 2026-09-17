// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

/** @file test_geo_anchor.cpp
 *  @brief GeoAnchor pure math + GeoAnchorSolver tests (VM-050, Epic 4 Task 1).
 */
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

// ── Step 0: pure-math round trip, no ROS ────────────────────────────────────

TEST(GeoAnchor, RoundTripWgsToMapAndBackWithin0p1mOver2km) {
    // Anchor at the recorded bag's own first fix: origin_lat_deg=25.0803,
    // origin_lon_deg=55.3910, heading_rad=0 (identity heading -- the round
    // trip must hold regardless of heading; the next test pins a nonzero
    // heading).
    GeoAnchor a{25.0803, 55.3910, 0.0};
    // Probe points up to ~2 km from the origin in both lat and lon (the
    // AC's own "2 km area" bound), not just the origin itself.
    for (auto [dlat, dlon] : {std::pair{0.0, 0.0}, {0.01, 0.0}, {0.0, 0.01}, {-0.015, 0.02}}) {
        const double lat = a.origin_lat_deg + dlat, lon = a.origin_lon_deg + dlon;
        const overlume::Vec3 xy = WgsToMap(a, lat, lon);
        const auto [lat2, lon2] = MapToWgs(a, xy);
        // 0.1 m at these latitudes is well under 1e-6 deg in both axes;
        // compare in METERS (haversine small-angle distance), not degrees,
        // so the assertion reads directly against the AC's own units.
        EXPECT_LT(GreatCircleDistanceM(lat, lon, lat2, lon2), 0.1);
    }
}

TEST(GeoAnchor, RoundTripHoldsWithNonzeroHeading) {
    GeoAnchor a{25.0803, 55.3910, 0.35};  // ~20 deg, an arbitrary nonzero bearing
    const overlume::Vec3 xy = WgsToMap(a, 25.0850, 55.3950);
    const auto [lat2, lon2] = MapToWgs(a, xy);
    EXPECT_LT(GreatCircleDistanceM(25.0850, 55.3950, lat2, lon2), 0.1);
}

namespace {

// Loads geo_anchor_samples_0.csv (VM-050 Step 1 fixture): lines starting
// with '#' are comments, every other line is "lat,lon,map_x,map_y".
struct SamplePairs {
    std::vector<std::pair<double, double>> fixes;   // (lat_deg, lon_deg)
    std::vector<std::pair<double, double>> map_xy;  // (map_x, map_y)
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

// ── Step 1: SolveAnchor() -- averaged position, bearing-comparison heading ──

TEST(GeoAnchor, SolveAnchorMatchesBagDerivedMeanPositionAndCoarseHeading) {
    const std::string path = std::string(OVERLUME_NODE_FIXTURES_DIR) + "/geo_anchor_samples_0.csv";
    const SamplePairs samples = LoadFixture(path);
    ASSERT_GE(samples.fixes.size(), 50u) << "fixture " << path << " missing/short";

    const GeoAnchor anchor = SolveAnchor(samples.fixes, samples.map_xy);

    // Position: the real contract is that the anchor is the WGS84 position
    // of map (0,0) -- i.e. WgsToMap(anchor, fix_i) reproduces map_i for
    // every sample -- NOT that it equals the samples' own mean fix (that is
    // the ego's mean position, which sits at whatever map-frame offset the
    // ego averaged to, not at the map origin).
    for (std::size_t i = 0; i < samples.fixes.size(); ++i) {
        const auto& [lat, lon] = samples.fixes[i];
        const auto& [mx, my] = samples.map_xy[i];
        const overlume::Vec3 got = WgsToMap(anchor, lat, lon);
        EXPECT_LT(std::hypot(got.x - mx, got.y - my), 1.0)
            << "sample " << i << " residual too large";
    }

    // Heading: independently re-derived here (not via SolveAnchor) from the
    // fixture's own first/last sample pair -- a coarse but real cross-check
    // of the bearing/map-angle composition, not re-deriving the same
    // arithmetic and comparing it to itself.
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

// ── Step 2: geo_datum override short-circuits sampling ──────────────────────

TEST(GeoAnchorSolver, OverrideSolvesImmediatelyWithoutAnyOnFixCall) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");

    // Deliberately DIFFERENT from any bag-derived anchor (fixture gap 2):
    // proves override REPLACES rather than blends with sampled data.
    solver.set_override(10.0, 20.0, 45.0);
    ASSERT_TRUE(solver.solved());
    const GeoAnchor a = solver.anchor();
    EXPECT_DOUBLE_EQ(a.origin_lat_deg, 10.0);
    EXPECT_DOUBLE_EQ(a.origin_lon_deg, 20.0);
    EXPECT_DOUBLE_EQ(a.heading_rad, 45.0 * M_PI / 180.0);
}

TEST(GeoAnchorSolver, OnFixAfterOverrideIsIgnored) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    buffer.setTransform(MapToBaseLink(1.0, 2.0), "test_authority", /*is_static=*/true);
    GeoAnchorSolver solver(buffer, "map", "base_link");

    solver.set_override(10.0, 20.0, 45.0);
    for (int i = 0; i < 5; ++i) solver.on_fix(Fix(25.08, 55.39));

    const GeoAnchor a = solver.anchor();
    EXPECT_DOUBLE_EQ(a.origin_lat_deg, 10.0);
    EXPECT_DOUBLE_EQ(a.origin_lon_deg, 20.0);
}

// ── Step 3: minimum-sample gate + non-fatal "no anchor" path ─────────────────

TEST(GeoAnchorSolver, FewerThanMinSamplesLeavesUnsolved) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    buffer.setTransform(MapToBaseLink(1.0, 2.0), "test_authority", /*is_static=*/true);
    GeoAnchorSolver solver(buffer, "map", "base_link");

    for (uint32_t i = 0; i < kMinAnchorSamples - 1; ++i) solver.on_fix(Fix(25.08, 55.39));
    EXPECT_FALSE(solver.solved());
}

TEST(GeoAnchorSolver, ReachingMinSamplesWithSufficientBaselineSolves) {
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");

    // 1.0 m of map-frame movement per sample -- kMinAnchorSamples worth of
    // that (500 m) clears kMinAnchorBaselineM long before the count gate
    // itself is satisfied.
    ASSERT_LT(kMinAnchorBaselineM, static_cast<double>(kMinAnchorSamples));
    for (uint32_t i = 0; i < kMinAnchorSamples; ++i) {
        buffer.setTransform(MapToBaseLink(static_cast<double>(i), 2.0), "test_authority",
                            /*is_static=*/true);
        solver.on_fix(Fix(25.08, 55.39));
    }
    EXPECT_TRUE(solver.solved());
}

TEST(GeoAnchorSolver, ReachingMinSamplesWithZeroBaselineStaysUnsolved) {
    // A static map->base_link transform: kMinAnchorSamples worth of fixes
    // arrive, but the accumulated track never moves -- the sample-count
    // gate alone is not sufficient (kMinAnchorBaselineM's own reasoning in
    // geo_anchor.hpp), so this must stay unsolved.
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    buffer.setTransform(MapToBaseLink(1.0, 2.0), "test_authority", /*is_static=*/true);
    GeoAnchorSolver solver(buffer, "map", "base_link");

    for (uint32_t i = 0; i < kMinAnchorSamples + 10; ++i) solver.on_fix(Fix(25.08, 55.39));
    EXPECT_FALSE(solver.solved());
}

TEST(GeoAnchorSolver, ZeroFixesAndZeroTfLeavesUnsolvedNoCrash) {
    // Empty buffer: every lookupTransform() throws -- mirrors TfAdapter's
    // own "no data yet, not an error" tf2::TransformException catch.
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");
    EXPECT_FALSE(solver.solved());

    // Fixes arrive but TF never does -- every on_fix() drops its sample.
    for (int i = 0; i < 10; ++i) solver.on_fix(Fix(25.08, 55.39));
    EXPECT_FALSE(solver.solved());
}

// ── ClassifyGeoDatum: pure all-or-nothing decision, no node fixture needed ──

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

TEST(GeoAnchorSolver, PartialGeoDatumNeverAppliedLeavesSolverUnsolved) {
    // Pins "a partial override never becomes an anchor": ClassifyGeoDatum
    // alone doesn't touch the solver, so a caller that (correctly, per the
    // Partial case above) never calls set_override() leaves the solver
    // exactly as unsolved as if no override existed at all.
    ASSERT_EQ(ClassifyGeoDatum(25.08, std::numeric_limits<double>::quiet_NaN(), 45.0),
              GeoDatumOverride::Partial);
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    GeoAnchorSolver solver(buffer, "map", "base_link");
    EXPECT_FALSE(solver.solved());
}
