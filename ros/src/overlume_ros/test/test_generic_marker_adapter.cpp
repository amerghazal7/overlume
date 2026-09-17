/** @file test_generic_marker_adapter.cpp
 *  @brief GenericMarkerAdapter tests: the spec's §7 parity guarantee.
 */
#include "overlume_ros/adapters/generic_marker.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;
using overlume_node::GenericMarkerAdapter;

namespace
{

// Same shape as test_hd_map_adapter.cpp/test_collision_adapter.cpp's
// TfFixture -- an empty buffer is enough for every test whose markers
// stay in the "map" frame.
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

geometry_msgs::msg::Point Pt(double x, double y, double z = 0.0)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::msg::ColorRGBA Rgba(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

visualization_msgs::msg::Marker BaseMarker(const std::string& ns, int32_t id, int32_t type)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = 0;  // ADD
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 1.0;
    return m;
}

visualization_msgs::msg::Marker DeleteAll()
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.action = 3;
    return m;
}

// urban_profile.yaml's /sim/ground_truth/boxes row ships commented out (the
// ego's own ground-truth box flickers at the robot proxy's origin), so
// urban_row("/sim/ground_truth/boxes") throws for every test below. Build
// the CANONICAL disabled row text directly instead (best_effort: true is
// load-bearing), independent of the row's shipped state.
overlume_node::ProfileRow GroundTruthBoxesRow()
{
    std::vector<std::string> errs;
    auto p = overlume_node::load_profile_string(
        "name: t\nrows:\n  - {topic: /sim/ground_truth/boxes, "
        "type: visualization_msgs/msg/MarkerArray, adapter: generic, role: neutral, "
        "best_effort: true}\n",
        errs);
    if (!p) throw std::runtime_error("GroundTruthBoxesRow: profile failed to parse");
    return p->rows[0];
}

}  // namespace

// ── Step 4: fan-out is the ONLY freeze-respecting route for CUBE_LIST ──────

TEST(GenericMarkerAdapter, CubeListFansOutIntoOneMarkerPerPoint)
{
    TfFixture kTf;
    auto row = GroundTruthBoxesRow();
    GenericMarkerAdapter a(row, kTf.tf);

    auto m = BaseMarker("boxes", 1, /*CUBE_LIST=*/6);
    m.points = {Pt(1, 0, 0), Pt(2, 0, 0), Pt(3, 0, 0)};
    m.scale.x = m.scale.y = m.scale.z = 2.0;
    m.color = Rgba(0.1f, 0.2f, 0.3f, 1.0f);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers = {m};
    a.ingest(msg, 1.0);

    SceneAssembly out;
    a.fill(out);

    ASSERT_EQ(out.markers.size(), 3u);
    for (const auto& g : out.markers)
    {
        EXPECT_EQ(g.primitive, overlume::MarkerPrimitive::CUBE);
        EXPECT_DOUBLE_EQ(g.scale.x, 2.0);
        EXPECT_FLOAT_EQ(g.color[0], 0.1f);
        EXPECT_FLOAT_EQ(g.color[3], 1.0f);
    }
    EXPECT_NEAR(out.markers[0].position.x, 1.0, 1e-9);
    EXPECT_NEAR(out.markers[1].position.x, 2.0, 1e-9);
    EXPECT_NEAR(out.markers[2].position.x, 3.0, 1e-9);
}

TEST(GenericMarkerAdapter, SphereListFanOutUsesPerPointColorsWhenPopulated)
{
    TfFixture kTf;
    auto row = GroundTruthBoxesRow();
    GenericMarkerAdapter a(row, kTf.tf);

    auto m = BaseMarker("spheres", 2, /*SPHERE_LIST=*/7);
    m.points = {Pt(0, 0, 0), Pt(0, 5, 0)};
    m.colors = {Rgba(1.0f, 0.0f, 0.0f, 0.0f), Rgba(0.0f, 1.0f, 0.0f, 0.0f)};

    visualization_msgs::msg::MarkerArray msg;
    msg.markers = {m};
    a.ingest(msg, 1.0);

    SceneAssembly out;
    a.fill(out);

    ASSERT_EQ(out.markers.size(), 2u);
    EXPECT_EQ(out.markers[0].primitive, overlume::MarkerPrimitive::SPHERE);
    EXPECT_FLOAT_EQ(out.markers[0].color[0], 1.0f);
    // colors[].a is documented "not yet used" -- forced to 1.0 so a
    // supplied per-point color is never silently read as "no color"
    // (scene.h's alpha==0 sentinel).
    EXPECT_FLOAT_EQ(out.markers[0].color[3], 1.0f);
    EXPECT_FLOAT_EQ(out.markers[1].color[1], 1.0f);
}

// ── Frames: /sim/ground_truth/boxes publishes in base_link ─────────────────

TEST(GenericMarkerAdapter, BaseLinkMarkersLandAroundTheEgoNotTheMapOrigin)
{
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    geometry_msgs::msg::TransformStamped xf;
    xf.header.frame_id = "map";
    xf.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    xf.child_frame_id = "base_link";
    xf.transform.translation.x = 150.0;  // ego 150m out
    xf.transform.translation.y = 0.0;
    xf.transform.rotation.w = 1.0;
    buffer.setTransform(xf, "test_authority", /*is_static=*/true);
    FrameTransformer ft(buffer);

    auto row = GroundTruthBoxesRow();
    GenericMarkerAdapter a(row, ft);

    auto box1 = BaseMarker("boxes", 1, /*CUBE=*/1);
    box1.header.frame_id = "base_link";
    box1.pose.position.x = 3.0;
    auto box2 = BaseMarker("boxes", 2, /*CUBE=*/1);
    box2.header.frame_id = "base_link";
    box2.pose.position.x = -3.0;

    visualization_msgs::msg::MarkerArray msg;
    msg.markers = {box1, box2};
    a.ingest(msg, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.markers.size(), 2u);
    for (const auto& g : out.markers)
    {
        // Around the ego (150 +- a few m), nowhere near the map origin.
        EXPECT_GT(g.position.x, 100.0);
        EXPECT_LT(g.position.x, 200.0);
    }
}

// ── Malformed / DELETEALL / namespace rule ──────────────────────────────────

TEST(GenericMarkerAdapter, MalformedMarkersDroppedAndCountedNeighboursStillRender)
{
    TfFixture kTf;
    auto row = GroundTruthBoxesRow();
    GenericMarkerAdapter a(row, kTf.tf);

    auto good = BaseMarker("boxes", 1, /*CUBE=*/1);
    auto bad_scale = BaseMarker("boxes", 2, /*CUBE=*/1);
    bad_scale.scale.x = 0.0;
    auto bad_type = BaseMarker("boxes", 3, /*unknown type*/ 999);
    auto bad_mesh = BaseMarker("boxes", 4, /*MESH_RESOURCE=*/10);
    // mesh_resource left empty -- malformed.

    visualization_msgs::msg::MarkerArray msg;
    msg.markers = {good, bad_scale, bad_type, bad_mesh};
    a.ingest(msg, 1.0);

    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.markers.size(), 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 3u);
}

TEST(GenericMarkerAdapter, DeleteAllClearsPreviousMarkers)
{
    TfFixture kTf;
    auto row = GroundTruthBoxesRow();
    GenericMarkerAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray msg1;
    msg1.markers = {BaseMarker("boxes", 1, /*CUBE=*/1)};
    a.ingest(msg1, 1.0);

    visualization_msgs::msg::MarkerArray msg2;
    msg2.markers = {DeleteAll()};
    a.ingest(msg2, 1.1);

    SceneAssembly out;
    a.fill(out);
    EXPECT_TRUE(out.markers.empty());
}

TEST(GenericMarkerAdapter, DroppedByRuleNamespaceNeverReachesStorage)
{
    auto row = GroundTruthBoxesRow();
    row.namespaces.push_back(
        overlume_node::NsRule{"noisy_", overlume_node::NsRender::kDrop, overlume::MapKind::OTHER});
    TfFixture kTf;
    GenericMarkerAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers = {BaseMarker("noisy_debug", 1, /*CUBE=*/1), BaseMarker("boxes", 2, /*CUBE=*/1)};
    a.ingest(msg, 1.0);

    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.markers.size(), 1u);
    EXPECT_EQ(a.stats().dropped_by_rule, 1u);
}

// ── Non-zero lifetime expiry, judged against ingest sim time ───────────────

TEST(GenericMarkerAdapter, NonZeroLifetimeExpiresOnALaterIngestPastIt)
{
    TfFixture kTf;
    auto row = GroundTruthBoxesRow();
    GenericMarkerAdapter a(row, kTf.tf);

    auto m = BaseMarker("boxes", 1, /*CUBE=*/1);
    m.lifetime.sec = 0;
    m.lifetime.nanosec = 200000000;  // 0.2s, the shipped row's real lifetime

    visualization_msgs::msg::MarkerArray msg1;
    msg1.markers = {m};
    a.ingest(msg1, 10.0);
    {
        SceneAssembly out;
        a.fill(out);
        EXPECT_EQ(out.markers.size(), 1u);
    }

    // A later message, 0.5s on (past the 0.2s lifetime), with NO re-ADD of
    // id 1 -- it must have expired, swept at the top of THIS ingest() call.
    visualization_msgs::msg::MarkerArray msg2;
    msg2.markers = {BaseMarker("boxes", 2, /*CUBE=*/1)};
    a.ingest(msg2, 10.5);
    {
        SceneAssembly out;
        a.fill(out);
        ASSERT_EQ(out.markers.size(), 1u);
        // Only id 2 (a fresh position at the origin) survives; id 1 (also
        // at the origin, same BaseMarker default pose) is gone.
    }
    EXPECT_EQ(a.stats().dropped_stale, 0u);  // node-side counter, untouched by expiry
}

// ── text/mesh_path storage outlives fill() ──────────────────────────────────

TEST(GenericMarkerAdapter, TextAndMeshPathStorageSurvivesFill)
{
    TfFixture kTf;
    auto row = GroundTruthBoxesRow();
    GenericMarkerAdapter a(row, kTf.tf);

    auto text = BaseMarker("boxes", 1, /*TEXT_VIEW_FACING=*/9);
    text.text = "hello";
    auto mesh = BaseMarker("boxes", 2, /*MESH_RESOURCE=*/10);
    mesh.mesh_resource = "file:///tmp/some_mesh.glb";

    visualization_msgs::msg::MarkerArray msg;
    msg.markers = {text, mesh};
    a.ingest(msg, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.markers.size(), 2u);
    bool found_text = false, found_mesh = false;
    for (const auto& g : out.markers)
    {
        if (g.primitive == overlume::MarkerPrimitive::TEXT)
        {
            ASSERT_NE(g.text, nullptr);
            EXPECT_STREQ(g.text, "hello");
            found_text = true;
        }
        if (g.primitive == overlume::MarkerPrimitive::MESH)
        {
            ASSERT_NE(g.mesh_path, nullptr);
            EXPECT_STREQ(g.mesh_path, "/tmp/some_mesh.glb");
            found_mesh = true;
        }
    }
    EXPECT_TRUE(found_text);
    EXPECT_TRUE(found_mesh);
}
