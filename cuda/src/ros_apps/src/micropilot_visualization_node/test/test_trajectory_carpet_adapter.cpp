/** @file test_trajectory_carpet_adapter.cpp
 *  @brief TrajectoryCarpetAdapter tests (VM-077).
 *
 *  Measured against the real recorded output_trajectory_carpet (VM-077
 *  measurement report §1) -- every fixture below reproduces that shape by
 *  hand (TRIANGLE_LIST, one persistent ns='output_trajectory_carpet' id=0
 *  marker, DELETE_ALL + one ADD per tick).
 */
#include "micropilot_visualization_node/adapters/trajectory_carpet.hpp"

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

using micropilot::visualization_app::FrameTransformer;
using micropilot::visualization_app::SceneAssembly;

namespace
{

// Hand-built tf2_ros::Buffer + FrameTransformer -- an empty buffer is
// enough for every fixture below (stays in the "map" frame, identity
// shortcut). Same fixture style as test_collision_adapter.cpp/
// test_point_cloud_adapter.cpp.
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

// No shipped profile row exists YET for adapter: trajectory_carpet at the
// point these tests are written (Step 0 precedes Step 3's row) -- hand-built
// directly here, same "no urban_row()/sim_row() to borrow" shape
// test_point_cloud_adapter.cpp already uses for its own row.
mpviz_node::ProfileRow MakeRow()
{
    mpviz_node::ProfileRow row;
    row.topic = "/navigation_motion_obstacle_planner_node/output_trajectory_carpet";
    row.type = "visualization_msgs/msg/MarkerArray";
    row.adapter = "trajectory_carpet";
    row.role = "carpet";
    row.timeout_sec = 2.0;
    return row;
}

visualization_msgs::msg::Marker MakeTriangleMarker(int32_t action, int32_t type)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "output_trajectory_carpet";
    m.id = 0;
    m.action = action;
    m.type = type;
    return m;
}

geometry_msgs::msg::Point MakePoint(double x, double y, double z)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::msg::ColorRGBA MakeColor(float r, float g, float b, float a)
{
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

}  // namespace

// ── Step 0: per-vertex color packed from colors[] ───────────────────────────

TEST(TrajectoryCarpetAdapter, IngestPacksPerVertexColorFromMarkerColorsArray)
{
    TfFixture kTf;
    auto row = MakeRow();
    mpviz_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    m.points = {MakePoint(0, 0, 0.15), MakePoint(1, 0, 0.15), MakePoint(1, 1, 0.15),
                MakePoint(1, 1, 0.15), MakePoint(2, 0, 0.15), MakePoint(2, 1, 0.15)};
    m.colors = {MakeColor(1.0f, 0.0f, 0.0f, 0.7f), MakeColor(0.0f, 1.0f, 0.0f, 0.7f),
                MakeColor(0.5f, 0.5f, 0.0f, 0.7f), MakeColor(0.5f, 0.5f, 0.0f, 0.7f),
                MakeColor(0.0f, 1.0f, 0.0f, 0.7f), MakeColor(1.0f, 0.0f, 0.0f, 0.7f)};
    arr.markers = {m};

    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.trajectory_carpets.size(), 1u);
    ASSERT_EQ(out.trajectory_carpets[0].point_count, 6u);

    // Unpacks back to the marker's own colors[i].{r,g,b}, alpha forced to
    // 255 ("supplied" sentinel) regardless of the marker's own (documented
    // "not yet used") colors[].a -- same convention
    // GenericMarkerAdapter's fan_colors already uses.
    const uint32_t rgba0 = out.trajectory_carpets[0].points[0].rgba;
    EXPECT_EQ(rgba0 & 0xFFu, 255u);          // r
    EXPECT_EQ((rgba0 >> 8) & 0xFFu, 0u);     // g
    EXPECT_EQ((rgba0 >> 16) & 0xFFu, 0u);    // b
    EXPECT_EQ((rgba0 >> 24) & 0xFFu, 255u);  // a (forced)

    const uint32_t rgba1 = out.trajectory_carpets[0].points[1].rgba;
    EXPECT_EQ(rgba1 & 0xFFu, 0u);
    EXPECT_EQ((rgba1 >> 8) & 0xFFu, 255u);
    EXPECT_EQ((rgba1 >> 24) & 0xFFu, 255u);
}

// ── Step 0: mismatched colors[]/points[] lengths -> alpha==0 sentinel ───────

TEST(TrajectoryCarpetAdapter, MismatchedColorsLengthBakesAlphaZeroSentinel)
{
    TfFixture kTf;
    auto row = MakeRow();
    mpviz_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    m.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0)};
    m.colors = {MakeColor(1.0f, 0.0f, 0.0f, 0.7f)};  // 1 color, 3 points -- mismatched
    arr.markers = {m};

    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.trajectory_carpets.size(), 1u);
    ASSERT_EQ(out.trajectory_carpets[0].point_count, 3u);
    for (uint32_t i = 0; i < 3; ++i)
    {
        EXPECT_EQ(out.trajectory_carpets[0].points[i].rgba, 0u)
            << "every point's packed rgba must be 0 (alpha byte 0) on a whole-message "
               "colors[]/points[] length mismatch, index " << i;
    }
}

// ── Step 0: point count not a multiple of 3 is dropped as malformed ────────

TEST(TrajectoryCarpetAdapter, PointCountNotMultipleOfThreeDropsMalformed)
{
    TfFixture kTf;
    auto row = MakeRow();
    mpviz_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    m.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0), MakePoint(2, 0, 0)};
    arr.markers = {m};

    a.ingest(arr, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.trajectory_carpets.size(), 0u);

    // A non-TRIANGLE_LIST type is malformed too -- this adapter has no
    // representation for anything else.
    TfFixture kTf2;
    mpviz_node::TrajectoryCarpetAdapter b(row, kTf2.tf);
    visualization_msgs::msg::MarkerArray arr2;
    auto lineStrip = MakeTriangleMarker(kActionAdd, kTypeLineStrip);
    lineStrip.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0)};
    arr2.markers = {lineStrip};
    b.ingest(arr2, 1.0);
    EXPECT_EQ(b.stats().dropped_malformed, 1u);
}

// ── Step 0: DELETE_ALL clears the stored carpet ─────────────────────────────

TEST(TrajectoryCarpetAdapter, DeleteAllClearsStoredCarpet)
{
    TfFixture kTf;
    auto row = MakeRow();
    mpviz_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    m.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0)};
    arr.markers = {m};
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

// ── Step 0: a second ingest() REPLACES wholesale, never appends ────────────

TEST(TrajectoryCarpetAdapter, ReplacesStoredCarpetWholesaleNotAppend)
{
    // Same PathAdapter-style "REPLACES, never merges" contract -- two
    // ingest() calls, second's content is what fill() emits, not a union.
    TfFixture kTf;
    auto row = MakeRow();
    mpviz_node::TrajectoryCarpetAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray first_arr;
    auto first_m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    first_m.points = {MakePoint(0, 0, 0), MakePoint(1, 0, 0), MakePoint(1, 1, 0),
                       MakePoint(2, 0, 0), MakePoint(2, 1, 0), MakePoint(3, 0, 0)};  // 6 points
    first_arr.markers = {first_m};
    a.ingest(first_arr, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_EQ(first.trajectory_carpets[0].point_count, 6u);

    visualization_msgs::msg::MarkerArray second_arr;
    auto second_m = MakeTriangleMarker(kActionAdd, kTypeTriangleList);
    second_m.points = {MakePoint(10, 0, 0), MakePoint(11, 0, 0), MakePoint(11, 1, 0)};  // 3 points
    second_arr.markers = {second_m};
    a.ingest(second_arr, 2.0);

    SceneAssembly second;
    a.fill(second);
    ASSERT_EQ(second.trajectory_carpets.size(), 1u);
    EXPECT_EQ(second.trajectory_carpets[0].point_count, 3u)
        << "a second valid message must REPLACE the stored carpet, not append to it";
    EXPECT_DOUBLE_EQ(second.trajectory_carpets[0].points[0].position.x, 10.0);
}
