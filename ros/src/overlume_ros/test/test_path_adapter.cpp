/** @file test_path_adapter.cpp
 *  @brief PathAdapter tests.
 */
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

namespace
{

// Hand-built tf2_ros::Buffer + FrameTransformer -- identical fixture style
// to test_hd_map_adapter.cpp/test_dynamic_objects_adapter.cpp. An empty
// buffer is enough: every fixture here is already in the "map" frame
// (FrameTransformer's identity shortcut never touches it).
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

nav_msgs::msg::Path MakePath(const std::vector<std::array<double, 3>>& xyz)
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    for (const auto& p : xyz)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = p[0];
        ps.pose.position.y = p[1];
        ps.pose.position.z = p[2];
        ps.pose.orientation.w = 1.0;  // identity -- never read by the adapter
        msg.poses.push_back(ps);
    }
    return msg;
}

}  // namespace

// ── Step 1: positions-only, never orientation ───────────────────────────────

TEST(PathAdapter, BehaviorPathHeadingDerivedFromPointsNotOrientation)
{
    // /behavior_path_planner/output_path_visualization poses carry IDENTITY
    // orientations (0,0,0,1) for every pose in the recorded bag. PathRibbon
    // is positions-only, so this is mostly a "don't be clever" guard --
    // assert the ribbon's points match the msg's pose positions exactly.
    auto msg = overlume_node::testing::load_path("behavior_output_path_0.yaml");
    ASSERT_GE(msg.poses.size(), 2u);
    TfFixture kTf;
    overlume_node::PathAdapter a(overlume_node::testing::urban_row(
                                   "/behavior_path_planner/output_path_visualization"),
                               kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);
    SceneAssembly out;
    a.fill(out);

    ASSERT_EQ(out.paths.size(), 1u);
    const overlume::PathRibbon& r = out.paths.front();
    ASSERT_EQ(r.point_count, msg.poses.size());
    for (uint32_t i = 0; i < r.point_count; ++i)
    {
        EXPECT_DOUBLE_EQ(r.points[i].x, msg.poses[i].pose.position.x);
        EXPECT_DOUBLE_EQ(r.points[i].y, msg.poses[i].pose.position.y);
        // flatten_z is ON by default (2D HD-map plane; see
        // frame_transform.hpp): stored z is 0 regardless of the recorded
        // pose z. HdMapAdapter.FlattenZOffPreservesPublisherZ covers the
        // configurable off-path.
        EXPECT_DOUBLE_EQ(r.points[i].z, 0.0);
    }
}

// ── Step 1: role from the profile row, never the topic name ────────────────

TEST(PathAdapter, RoleComesFromTheProfileRowNotTheTopicName)
{
    // behavior|global|local; renaming a topic is a YAML edit, not a code
    // edit -- construct against the SAME topic three times with a
    // hand-built row differing only in `role`, and assert the ribbon's
    // role follows the row, not the (unchanged) topic string.
    TfFixture kTf;
    auto msg = MakePath({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}});

    overlume_node::ProfileRow row = overlume_node::testing::urban_row(
        "/behavior_path_planner/output_path_visualization");
    ASSERT_EQ(row.role, "behavior");

    row.role = "behavior";
    {
        overlume_node::PathAdapter a(row, kTf.tf);
        a.ingest(msg, 1.0);
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.paths.size(), 1u);
        EXPECT_EQ(out.paths.front().role, overlume::PathRole::BEHAVIOR);
    }
    row.role = "global";
    {
        overlume_node::PathAdapter a(row, kTf.tf);
        a.ingest(msg, 1.0);
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.paths.size(), 1u);
        EXPECT_EQ(out.paths.front().role, overlume::PathRole::GLOBAL);
    }
    row.role = "local";
    {
        overlume_node::PathAdapter a(row, kTf.tf);
        a.ingest(msg, 1.0);
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.paths.size(), 1u);
        EXPECT_EQ(out.paths.front().role, overlume::PathRole::LOCAL);
    }
}

// ── Step 1: empty and single-pose paths are dropped and counted ────────────

TEST(PathAdapter, EmptyAndSinglePosePathsAreDroppedAndCounted)
{
    TfFixture kTf;
    overlume_node::PathAdapter a(
        overlume_node::testing::urban_row("/local_vel_path"), kTf.tf);

    auto empty = MakePath({});
    a.ingest(empty, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
    SceneAssembly out1;
    a.fill(out1);
    EXPECT_TRUE(out1.paths.empty());  // never received valid data

    auto single = MakePath({{5, 5, 0}});
    a.ingest(single, 2.0);
    EXPECT_EQ(a.stats().dropped_malformed, 2u);
    SceneAssembly out2;
    a.fill(out2);
    EXPECT_TRUE(out2.paths.empty());

    // msgs counts every ingest() call, including dropped ones.
    EXPECT_EQ(a.stats().msgs, 2u);
}

// ── Step 1: a new path REPLACES the stored one, never appends ──────────────

TEST(PathAdapter, PathChangeReplacesRatherThanAppends)
{
    auto long_path = overlume_node::testing::load_path("behavior_output_path_0.yaml");
    ASSERT_GT(long_path.poses.size(), 33u);
    TfFixture kTf;
    overlume_node::PathAdapter a(overlume_node::testing::urban_row(
                                   "/behavior_path_planner/output_path_visualization"),
                               kTf.tf);
    a.ingest(long_path, 1.0);
    SceneAssembly out1;
    a.fill(out1);
    ASSERT_EQ(out1.paths.size(), 1u);
    EXPECT_EQ(out1.paths.front().point_count, long_path.poses.size());

    // The real recorded 33-pose /local_vel_path message -- real recorded
    // coordinates beat a synthetic straight line.
    auto short_path = overlume_node::testing::load_path("local_vel_path_0.yaml");
    ASSERT_EQ(short_path.poses.size(), 33u);
    a.ingest(short_path, 2.0);
    SceneAssembly out2;
    a.fill(out2);
    ASSERT_EQ(out2.paths.size(), 1u);
    EXPECT_EQ(out2.paths.front().point_count, 33u);
}

// Anchored-stamp regression (flicker hardening 2026-09-10): a path row's
// fade must be anchored to its own timeout cutoff, not raw receipt time --
// raw stamping sawtooths any topic whose inter-message gap can cross the
// library's 0.5s fade start (the recorded /local_vel_path peaks at 0.461s,
// zero margin). Mirrors hd_map.cpp's VM-034 kMapFadeWindowSec tests.
TEST(PathAdapter, FillStampsLastUpdateSecAnchoredToRowTimeoutNotRawReceipt)
{
    TfFixture kTf;
    const auto row = overlume_node::testing::urban_row(
        "/behavior_path_planner/output_path_visualization");
    overlume_node::PathAdapter a(row, kTf.tf);
    a.ingest(MakePath({{0, 0, 0}, {5, 0, 0}}), /*sim_time_sec=*/10.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.paths.size(), 1u);
    // stamp = receipt + (timeout_sec - 1.0): the fade window occupies the
    // last second before the node's own timeout cutoff stops calling fill().
    EXPECT_DOUBLE_EQ(out.paths.front().last_update_sec, 10.0 + (row.timeout_sec - 1.0));
    EXPECT_GE(row.timeout_sec, 1.0);  // the validation premise the offset relies on
}
