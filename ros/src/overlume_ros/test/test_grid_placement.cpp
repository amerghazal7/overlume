// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/grid_placement.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

using overlume::ros::FrameTransformer;
using overlume::ros::GridPlacement;
using overlume::ros::PlacementResult;
using overlume::ros::place_grid;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
    FrameTransformer tf_keep_z{buffer, "map", false};

    // map <- base_link: +90 deg about z, translated 10 m along x, 1.5 m up.
    void add_base_link() {
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = 10.0;
        t.transform.translation.z = 1.5;
        t.transform.rotation.z = std::sin(M_PI / 4.0);
        t.transform.rotation.w = std::cos(M_PI / 4.0);
        buffer.setTransform(t, "test", true);
    }
};

std_msgs::msg::Header header(const std::string& frame) {
    std_msgs::msg::Header h;
    h.frame_id = frame;
    return h;
}

geometry_msgs::msg::Pose pose(double x, double y, double z, double yaw) {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.z = std::sin(yaw / 2.0);
    p.orientation.w = std::cos(yaw / 2.0);
    return p;
}

}

TEST(GridPlacement, MapFrameIsIdentityOnOriginAndYaw) {
    TfFixture f;
    GridPlacement out{};
    ASSERT_EQ(place_grid(header("map"), pose(1.0, 2.0, 0.0, 0.3), "", f.tf, out),
              PlacementResult::kOk);
    EXPECT_DOUBLE_EQ(out.origin.x, 1.0);
    EXPECT_DOUBLE_EQ(out.origin.y, 2.0);
    EXPECT_NEAR(out.yaw_rad, 0.3, 1e-12);
}

TEST(GridPlacement, EmptyFrameIdIsIdentity) {
    TfFixture f;
    GridPlacement out{};
    ASSERT_EQ(place_grid(header(""), pose(4.0, -5.0, 0.0, 0.0), "", f.tf, out),
              PlacementResult::kOk);
    EXPECT_DOUBLE_EQ(out.origin.x, 4.0);
    EXPECT_DOUBLE_EQ(out.origin.y, -5.0);
}

TEST(GridPlacement, FrameOverrideReplacesHeaderFrame) {
    TfFixture f;
    f.add_base_link();
    GridPlacement out{};
    // Header says "map" (identity); the override sends it through base_link.
    ASSERT_EQ(place_grid(header("map"), pose(1.0, 0.0, 0.0, 0.0), "base_link", f.tf, out),
              PlacementResult::kOk);
    EXPECT_NEAR(out.origin.x, 10.0, 1e-9);
    EXPECT_NEAR(out.origin.y, 1.0, 1e-9);
}

TEST(GridPlacement, RotatedTfComposesWithInfoOrientationIntoYaw) {
    TfFixture f;
    f.add_base_link();
    GridPlacement out{};
    ASSERT_EQ(place_grid(header("base_link"), pose(1.0, 0.0, 0.0, 0.3), "", f.tf, out),
              PlacementResult::kOk);
    EXPECT_NEAR(out.origin.x, 10.0, 1e-9);
    EXPECT_NEAR(out.origin.y, 1.0, 1e-9);
    EXPECT_NEAR(out.yaw_rad, M_PI / 2.0 + 0.3, 1e-9);
}

TEST(GridPlacement, FlattenZZeroesOriginZ) {
    TfFixture f;
    f.add_base_link();
    GridPlacement flat{};
    GridPlacement kept{};
    ASSERT_EQ(place_grid(header("base_link"), pose(0.0, 0.0, 2.0, 0.0), "", f.tf, flat),
              PlacementResult::kOk);
    ASSERT_EQ(place_grid(header("base_link"), pose(0.0, 0.0, 2.0, 0.0), "", f.tf_keep_z, kept),
              PlacementResult::kOk);
    EXPECT_DOUBLE_EQ(flat.origin.z, 0.0);
    EXPECT_NEAR(kept.origin.z, 3.5, 1e-9);
}

TEST(GridPlacement, MissingTfIsNoTfAndLeavesOutUntouched) {
    TfFixture f;
    GridPlacement out{};
    out.origin = {7.0, 7.0, 7.0};
    out.yaw_rad = 7.0;
    EXPECT_EQ(place_grid(header("nowhere"), pose(1.0, 2.0, 0.0, 0.0), "", f.tf, out),
              PlacementResult::kNoTf);
    EXPECT_DOUBLE_EQ(out.origin.x, 7.0);
    EXPECT_DOUBLE_EQ(out.yaw_rad, 7.0);
}

TEST(GridPlacement, NanOriginIsNonFinite) {
    TfFixture f;
    GridPlacement out{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(place_grid(header("map"), pose(nan, 0.0, 0.0, 0.0), "", f.tf, out),
              PlacementResult::kNonFinite);
}
