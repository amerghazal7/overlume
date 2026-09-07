/** @file test_tf_axes_adapter.cpp
 *  @brief TfAxesAdapter tests (Epic 2 Task 8 Step 7 / VM-027). Hand-built
 *  tf2_ros::Buffer, same fixture style as test_frame_transform.cpp -- no
 *  ROS graph, no launch.
 */
#include "micropilot_visualization_node/adapters/tf_axes.hpp"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include "micropilot_visualization_node/scene_assembly.hpp"

using micropilot::visualization_app::SceneAssembly;
using mpviz_node::ProfileRow;
using mpviz_node::TfAxesAdapter;

namespace
{

geometry_msgs::msg::TransformStamped MapToChild(const std::string& child, double x, double y,
                                                 double z)
{
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

ProfileRow TfAxesRow()
{
    ProfileRow row;
    row.adapter = "tf_axes";
    row.role = "debug";
    row.timeout_sec = 1.0;
    return row;
}

}  // namespace

TEST(TfAxes, EmitsThreeMarkersPerKnownFrame)
{
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    buffer.setTransform(MapToChild("base_link", 10.0, 20.0, 0.0), "test_authority",
                        /*is_static=*/true);
    buffer.setTransform(MapToChild("sensor_frame", 1.0, 2.0, 3.0), "test_authority",
                        /*is_static=*/true);

    TfAxesAdapter a(TfAxesRow(), buffer);
    SceneAssembly out;
    a.fill(out, /*sim_time_sec=*/5.0);

    // Two known non-map frames * 3 axes each = 6. `map` itself may or may
    // not appear in getAllFrameNames() depending on tf2's internal
    // bookkeeping -- assert a floor, not an exact count, so this test
    // doesn't couple itself to that implementation detail.
    EXPECT_GE(out.markers.size(), 6u);
    for (const auto& m : out.markers)
    {
        EXPECT_EQ(m.primitive, mpviz::MarkerPrimitive::LINE_LIST);
        ASSERT_NE(m.points, nullptr);
        EXPECT_EQ(m.point_count, 2u);
        EXPECT_FLOAT_EQ(m.color[3], 1.0f);
        // Stamped fresh every call (this adapter's own header comment) --
        // never stale, regardless of how long the node has been running.
        EXPECT_DOUBLE_EQ(m.last_update_sec, 5.0);
    }

    // At least one X (red), Y (green), Z (blue) axis marker must be
    // present per known frame.
    int red = 0, green = 0, blue = 0;
    for (const auto& m : out.markers)
    {
        if (m.color[0] == 1.0f && m.color[1] == 0.0f && m.color[2] == 0.0f) ++red;
        if (m.color[0] == 0.0f && m.color[1] == 1.0f && m.color[2] == 0.0f) ++green;
        if (m.color[0] == 0.0f && m.color[1] == 0.0f && m.color[2] == 1.0f) ++blue;
    }
    EXPECT_GE(red, 2);
    EXPECT_GE(green, 2);
    EXPECT_GE(blue, 2);

    // base_link's origin must land at (10, 20, 0), not the map origin.
    bool found_base_link_origin = false;
    for (const auto& m : out.markers)
    {
        if (std::abs(m.points[0].x - 10.0) < 1e-6 && std::abs(m.points[0].y - 20.0) < 1e-6)
        {
            found_base_link_origin = true;
        }
    }
    EXPECT_TRUE(found_base_link_origin);
    EXPECT_EQ(a.stats().dropped_no_tf, 0u);
}

TEST(TfAxes, UnresolvableFrameIsSkippedAndCountedNotFatal)
{
    // An empty buffer still returns SOME frame strings in some tf2
    // versions (none, typically) -- what matters here is that a buffer
    // seeded with a transform whose PARENT never resolves back to
    // target_frame_ is skipped and counted, not fatal.
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    // "orphan" is its own root, never connected to "map" at all.
    buffer.setTransform(MapToChild("orphan_child", 1.0, 1.0, 1.0), "test_authority",
                        /*is_static=*/true);

    TfAxesAdapter a(TfAxesRow(), buffer, /*target_frame=*/"map_that_does_not_exist");
    SceneAssembly out;
    ASSERT_NO_THROW(a.fill(out, /*sim_time_sec=*/1.0));

    EXPECT_TRUE(out.markers.empty());
    EXPECT_GT(a.stats().dropped_no_tf, 0u);
}
