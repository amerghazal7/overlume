// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

/** @file test_frame_transform.cpp
 *  @brief FrameTransformer tests.
 */
#include "overlume_ros/frame_transform.hpp"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

using overlume::ros::FrameTransformer;

namespace {

// map <- base_link: translate (100, 50, 0), yaw +90 deg about Z.
geometry_msgs::msg::TransformStamped MapToBaseLinkTransform() {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    msg.child_frame_id = "base_link";
    msg.transform.translation.x = 100.0;
    msg.transform.translation.y = 50.0;
    msg.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI_2);
    msg.transform.rotation = tf2::toMsg(q);
    return msg;
}

}  // namespace

TEST(FrameTransform, MapFrameIsIdentityAndDoesNotTouchTheBuffer) {
    // Empty buffer: any real lookup would throw. frame_id == "map" must
    // short-circuit to identity without ever calling into it.
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    FrameTransformer ft(buffer, "map");

    std_msgs::msg::Header header;
    header.frame_id = "map";

    tf2::Transform out;
    ASSERT_TRUE(ft.lookup(header, out));
    EXPECT_EQ(out.getOrigin().x(), 0.0);
    EXPECT_EQ(out.getOrigin().y(), 0.0);
    EXPECT_EQ(out.getOrigin().z(), 0.0);
    EXPECT_EQ(out.getRotation().getAngle(), 0.0);

    // Empty frame_id (no publisher ever set one) -> same identity shortcut.
    std_msgs::msg::Header empty_header;
    tf2::Transform out2;
    ASSERT_TRUE(ft.lookup(empty_header, out2));
    EXPECT_EQ(out2.getOrigin().x(), 0.0);
}

TEST(FrameTransform, BaseLinkPointIsMovedIntoMapFrame) {
    // /sim/ground_truth/boxes really does publish in base_link (every other
    // bag topic is map). Feed a buffer a map<-base_link of (100, 50, yaw 90d),
    // transform (1,0,0) -> expect (100,51,0), not (1,0,0).
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    buffer.setTransform(MapToBaseLinkTransform(), "test_authority", /*is_static=*/true);
    FrameTransformer ft(buffer, "map");

    std_msgs::msg::Header header;
    header.frame_id = "base_link";
    header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    tf2::Transform out;
    ASSERT_TRUE(ft.lookup(header, out));

    tf2::Vector3 p = out * tf2::Vector3(1.0, 0.0, 0.0);
    EXPECT_NEAR(p.x(), 100.0, 1e-6);
    EXPECT_NEAR(p.y(), 51.0, 1e-6);
    EXPECT_NEAR(p.z(), 0.0, 1e-6);
}

TEST(FrameTransform, LookupFailureIsReportedNotSilentlyIdentity) {
    // returns false -> caller drops the MESSAGE (not the marker) + dropped_no_tf.
    // Silently passing untransformed coordinates through is the bug this
    // whole helper exists to prevent: it renders wrong AND looks plausible.
    tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    FrameTransformer ft(buffer, "map");

    std_msgs::msg::Header header;
    header.frame_id = "sensor_frame_nobody_ever_published";
    header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    tf2::Transform out;
    EXPECT_FALSE(ft.lookup(header, out));
}
