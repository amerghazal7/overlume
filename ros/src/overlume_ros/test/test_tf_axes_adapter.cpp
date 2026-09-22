// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/tf_axes.hpp"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include "overlume_ros/scene_assembly.hpp"

using overlume::ros::SceneAssembly;
using overlume::ros::ProfileRow;
using overlume::ros::TfAxesAdapter;

namespace {

geometry_msgs::msg::TransformStamped MapToChild(const std::string& child, double x, double y,
                                                double z) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    msg.child_frame_id = child;
    msg.transform.translation.x = x;
    msg.transform.translation.y = y;
    msg.transform.translation.z = z;
    msg.transform.rotation.w = 1.0;
    return msg;
}

ProfileRow TfAxesRow() {
    ProfileRow row;
    row.adapter = "tf_axes";
    row.role = "debug";
    row.timeout_sec = 1.0;
    return row;
}

}

TEST(TfAxes, EmitsThreeMarkersPerKnownFrame) {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    buffer.setTransform(MapToChild("base_link", 10.0, 20.0, 0.0), "test_authority", true);
    buffer.setTransform(MapToChild("sensor_frame", 1.0, 2.0, 3.0), "test_authority", true);

    TfAxesAdapter a(TfAxesRow(), buffer);
    SceneAssembly out;
    a.fill(out, 5.0);

    EXPECT_GE(out.markers.size(), 6u);
    for (const auto& m : out.markers) {
        EXPECT_EQ(m.primitive, overlume::MarkerPrimitive::LINE_LIST);
        ASSERT_NE(m.points, nullptr);
        EXPECT_EQ(m.point_count, 2u);
        EXPECT_FLOAT_EQ(m.color[3], 1.0f);
        EXPECT_DOUBLE_EQ(m.last_update_sec, 5.0);
    }

    int red = 0, green = 0, blue = 0;
    for (const auto& m : out.markers) {
        if (m.color[0] == 1.0f && m.color[1] == 0.0f && m.color[2] == 0.0f) ++red;
        if (m.color[0] == 0.0f && m.color[1] == 1.0f && m.color[2] == 0.0f) ++green;
        if (m.color[0] == 0.0f && m.color[1] == 0.0f && m.color[2] == 1.0f) ++blue;
    }
    EXPECT_GE(red, 2);
    EXPECT_GE(green, 2);
    EXPECT_GE(blue, 2);

    bool found_base_link_origin = false;
    for (const auto& m : out.markers) {
        if (std::abs(m.points[0].x - 10.0) < 1e-6 && std::abs(m.points[0].y - 20.0) < 1e-6) {
            found_base_link_origin = true;
        }
    }
    EXPECT_TRUE(found_base_link_origin);
    EXPECT_EQ(a.stats().dropped_no_tf, 0u);
}

TEST(TfAxes, UnresolvableFrameIsSkippedAndCountedNotFatal) {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    buffer.setTransform(MapToChild("orphan_child", 1.0, 1.0, 1.0), "test_authority", true);

    TfAxesAdapter a(TfAxesRow(), buffer, "map_that_does_not_exist");
    SceneAssembly out;
    ASSERT_NO_THROW(a.fill(out, 1.0));

    EXPECT_TRUE(out.markers.empty());
    EXPECT_GT(a.stats().dropped_no_tf, 0u);
}
