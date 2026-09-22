// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/dynamic_objects.hpp"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;
using overlume::ros::ClassInferenceTable;
using overlume::ros::DynamicObjectsAdapter;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

visualization_msgs::msg::Marker DeleteAll() {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.action = 3;
    return m;
}

geometry_msgs::msg::Point Pt(double x, double y, double z = 0.0) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

visualization_msgs::msg::Marker Bbox(int32_t id, double x, double y, double z, double qx, double qy,
                                     double qz, double qw, double sx = 4.0, double sy = 2.0,
                                     double sz = 1.5) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "dynamic_objects_bbox";
    m.id = id;
    m.type = 1;
    m.action = 0;
    m.pose.position = Pt(x, y, z);
    m.pose.orientation.x = qx;
    m.pose.orientation.y = qy;
    m.pose.orientation.z = qz;
    m.pose.orientation.w = qw;
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    return m;
}

visualization_msgs::msg::Marker Text(int32_t id, const std::string& text) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "dynamic_objects_text";
    m.id = id;
    m.type = 9;
    m.action = 0;
    m.text = text;
    return m;
}

visualization_msgs::msg::Marker Arrow(int32_t id, geometry_msgs::msg::Point p0,
                                      geometry_msgs::msg::Point p1) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "dynamic_objects_arrow";
    m.id = id;
    m.type = 0;
    m.action = 0;
    m.points = {p0, p1};
    return m;
}

visualization_msgs::msg::Marker PathMarker(const char* ns, int32_t id,
                                           const std::vector<geometry_msgs::msg::Point>& pts) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = id;
    m.type = 5;
    m.action = 0;
    m.points = pts;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    return m;
}

void BuildChainedLineList(size_t n_segments, std::vector<geometry_msgs::msg::Point>& wire,
                          std::vector<overlume::Vec3>& expected) {
    wire.clear();
    expected.clear();
    for (size_t i = 0; i <= n_segments; ++i) expected.push_back({static_cast<double>(i), 0.0, 0.0});
    for (size_t s = 0; s < n_segments; ++s) {
        wire.push_back(Pt(expected[s].x, expected[s].y, expected[s].z));
        wire.push_back(Pt(expected[s + 1].x, expected[s + 1].y, expected[s + 1].z));
    }
}

const overlume::TrackedObject* FindById(const SceneAssembly& out, uint32_t id) {
    for (const auto& o : out.objects) {
        if (o.id == id) return &o;
    }
    return nullptr;
}

}

TEST(ClassInference, PrefixWinsOverFootprint) {
    const auto cfg = overlume::ros::testing::inference_table();
    EXPECT_EQ(overlume::ros::infer(cfg, "V_1105", {5.03, 2.15, 1.65}), overlume::ObjectClass::CAR);
}

TEST(ClassInference, UnknownPrefixFallsBackToFootprintBands) {
    const auto cfg = overlume::ros::testing::inference_table();
    EXPECT_EQ(overlume::ros::infer(cfg, "Z_9", {0.6, 0.6, 1.8}), overlume::ObjectClass::PEDESTRIAN);
    EXPECT_EQ(overlume::ros::infer(cfg, "Z_9", {1.9, 0.7, 1.7}), overlume::ObjectClass::CYCLIST);
    EXPECT_EQ(overlume::ros::infer(cfg, "Z_9", {12.0, 2.5, 3.2}), overlume::ObjectClass::BUS);
}

TEST(ClassInference, NoLabelAndNoMatchingBandIsUnknownNotACrash) {
    const auto cfg = overlume::ros::testing::inference_table();
    EXPECT_EQ(overlume::ros::infer(cfg, nullptr, {0, 0, 0}), overlume::ObjectClass::UNKNOWN);
}

TEST(DynamicObjects, FourNamespacesFuseIntoOneTrackedObject) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    std::vector<geometry_msgs::msg::Point> path_wire;
    std::vector<overlume::Vec3> path_expected;
    BuildChainedLineList(72, path_wire, path_expected);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, -66.8, -13.2, 0.39, 0.0, 0.0, -1.0, 0.0));
    msg.markers.push_back(Text(1007, "V_1007"));
    msg.markers.push_back(Arrow(1007, Pt(0, 0, 0), Pt(2, 0, 0)));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path", 1007, path_wire));

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    ASSERT_GT(out.objects.size(), 0u);
    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    EXPECT_GT(o->dimensions.x, 0.0);
    ASSERT_NE(o->label, nullptr);
    EXPECT_EQ(std::string(o->label).front(), 'V');
    EXPECT_EQ(o->cls, overlume::ObjectClass::CAR);
    EXPECT_DOUBLE_EQ(o->last_update_sec, 1.0);
}

TEST(DynamicObjects, HeadingComesFromTheBboxPoseOrientationNotTheArrow) {
    auto msg = overlume::ros::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o1001 = FindById(out, 1001);
    ASSERT_NE(o1001, nullptr);
    EXPECT_NEAR(o1001->heading_rad, 3.139082256947653, 1e-6);

    const auto* o1002 = FindById(out, 1002);
    ASSERT_NE(o1002, nullptr);
    EXPECT_NEAR(o1002->heading_rad, -1.8689938783645632, 1e-6);

    const auto* o1008 = FindById(out, 1008);
    ASSERT_NE(o1008, nullptr);
    EXPECT_NEAR(o1008->heading_rad, -2.517800807952881, 1e-6);
}

TEST(DynamicObjects, ArrowSuppliesVelocityOnlyAndZeroLengthIsZeroVelocity) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(2001, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(2001, "V_2001"));
    msg.markers.push_back(Arrow(2001, Pt(0, 0, 0), Pt(3, 4, 0)));
    msg.markers.push_back(Bbox(2002, 10, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(2002, "V_2002"));
    msg.markers.push_back(Arrow(2002, Pt(5, 5, 0), Pt(5, 5, 0)));

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* moving = FindById(out, 2001);
    ASSERT_NE(moving, nullptr);
    EXPECT_DOUBLE_EQ(moving->velocity.x, 3.0);
    EXPECT_DOUBLE_EQ(moving->velocity.y, 4.0);
    EXPECT_DOUBLE_EQ(moving->heading_rad, 0.0);

    const auto* stopped = FindById(out, 2002);
    ASSERT_NE(stopped, nullptr);
    EXPECT_DOUBLE_EQ(stopped->velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(stopped->velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(stopped->velocity.z, 0.0);
}

TEST(DynamicObjects, FirstFramePerObjectHasNoArrow_StillEmitsObject) {
    auto msg = overlume::ros::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1001);
    ASSERT_NE(o, nullptr);
    EXPECT_DOUBLE_EQ(o->velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(o->velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(o->velocity.z, 0.0);
    EXPECT_NEAR(o->heading_rad, 3.139082256947653, 1e-6);
    EXPECT_GT(o->dimensions.x, 0.0);
}

TEST(DynamicObjects, PredictedPathLineListPairsCollapseToAPolyline) {
    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<overlume::Vec3> expected;
    BuildChainedLineList(72, wire, expected);
    ASSERT_EQ(wire.size(), 144u);

    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(1007, "V_1007"));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path", 1007, wire));
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    EXPECT_EQ(o->predicted_path_count, 73u);
    ASSERT_GT(o->predicted_path_count, 0u);
    EXPECT_DOUBLE_EQ(o->predicted_path[0].x, expected.front().x);
    EXPECT_DOUBLE_EQ(o->predicted_path[o->predicted_path_count - 1].x, expected.back().x);
}

TEST(DynamicObjects, NonContiguousLineListIsDroppedNotStitched) {
    std::vector<geometry_msgs::msg::Point> disjoint = {Pt(0, 0, 0), Pt(1, 0, 0), Pt(5, 0, 0),
                                                       Pt(6, 0, 0)};
    std::vector<overlume::Vec3> out_pts;
    EXPECT_FALSE(overlume::ros::line_list_to_polyline(disjoint, out_pts));

    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(1007, "V_1007"));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path", 1007, disjoint));
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    EXPECT_EQ(o->predicted_path_count, 0u);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
}

TEST(DynamicObjects, PredictedPathReadsPerVertexColorsTopLevelRgbaIsBlack) {
    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<overlume::Vec3> expected;
    BuildChainedLineList(4, wire, expected);

    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(1007, "V_1007"));
    auto path = PathMarker("dynamic_objects_hd_map_path", 1007, wire);
    for (size_t i = 0; i < wire.size(); ++i) {
        std_msgs::msg::ColorRGBA c;
        c.r = 1.0f;
        c.g = 0.5f;
        c.b = 0.25f;
        c.a = 1.0f;
        path.colors.push_back(c);
    }
    msg.markers.push_back(path);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    EXPECT_EQ(o->predicted_path_count, 5u);
}

TEST(DynamicObjects, PathDotsNamespaceIsDroppedByRuleAndCounted) {
    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<overlume::Vec3> expected;
    BuildChainedLineList(2, wire, expected);

    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    auto row = overlume::ros::testing::urban_row("/perception/dynamic_objects_list");
    DynamicObjectsAdapter a(row, kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(1007, "V_1007"));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path", 1007, wire));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path_dots", 1007, wire));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path_dots", 1008, wire));

    ASSERT_EQ(overlume::ros::classify(row, "dynamic_objects_hd_map_path_dots"),
              overlume::ros::NsRender::kDrop);
    ASSERT_EQ(overlume::ros::classify(row, "dynamic_objects_hd_map_path"),
              overlume::ros::NsRender::kPolyline);

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    EXPECT_EQ(o->predicted_path_count, 3u);
    EXPECT_EQ(a.stats().dropped_by_rule, 2u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(DynamicObjects, DeleteAllMarkerClearsPreviousFrame) {
    auto msg = overlume::ros::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    a.ingest(msg, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_GT(first.objects.size(), 0u);

    visualization_msgs::msg::MarkerArray clear_only;
    clear_only.markers.push_back(DeleteAll());
    a.ingest(clear_only, 2.0);
    SceneAssembly second;
    a.fill(second);
    EXPECT_EQ(second.objects.size(), 0u);
}

TEST(DynamicObjects, MalformedMarkersDroppedAndCounted) {
    auto msg = overlume::ros::testing::load_marker_array("dynamic_objects_malformed.yaml");
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_malformed, 4u);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.objects.size(), 1u);
    EXPECT_EQ(out.objects[0].id, 3005u);
}

namespace {
constexpr double kQuarterTurn = 0.70710678118654752440;
}

TEST(DynamicObjects, PathMarkerOwnPoseComposesBeforePolylineConversion) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<overlume::Vec3> expected;
    BuildChainedLineList(1, wire, expected);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(1007, "V_1007"));
    auto path = PathMarker("dynamic_objects_hd_map_path", 1007, wire);
    path.pose.position.x = 5.0;
    msg.markers.push_back(path);

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    ASSERT_EQ(o->predicted_path_count, 2u);
    EXPECT_NEAR(o->predicted_path[0].x, 5.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[0].y, 0.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[1].x, 6.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[1].y, 0.0, 1e-9);
}

TEST(DynamicObjects, ArrowPureTranslationPoseLeavesVelocityUnchanged) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(2001, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(2001, "V_2001"));
    auto arrow = Arrow(2001, Pt(0, 0, 0), Pt(3, 4, 0));
    arrow.pose.position.x = 500.0;
    arrow.pose.position.y = -200.0;
    msg.markers.push_back(arrow);

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 2001);
    ASSERT_NE(o, nullptr);
    EXPECT_DOUBLE_EQ(o->velocity.x, 3.0);
    EXPECT_DOUBLE_EQ(o->velocity.y, 4.0);
}

TEST(DynamicObjects, ArrowNinetyDegreeYawPoseRotatesVelocity) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(2001, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(2001, "V_2001"));
    auto arrow = Arrow(2001, Pt(0, 0, 0), Pt(3, 4, 0));
    arrow.pose.orientation.z = kQuarterTurn;
    arrow.pose.orientation.w = kQuarterTurn;
    msg.markers.push_back(arrow);

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 2001);
    ASSERT_NE(o, nullptr);
    EXPECT_NEAR(o->velocity.x, -4.0, 1e-9);
    EXPECT_NEAR(o->velocity.y, 3.0, 1e-9);
}

TEST(DynamicObjects, MarkerPoseAndNonMapFrameComposeInFrameInsideOrder) {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    geometry_msgs::msg::TransformStamped xf;
    xf.header.frame_id = "map";
    xf.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    xf.child_frame_id = "base_link";
    xf.transform.translation.x = 1000.0;
    xf.transform.rotation.z = kQuarterTurn;
    xf.transform.rotation.w = kQuarterTurn;
    buffer.setTransform(xf, "test_authority", true);
    FrameTransformer ft(buffer);

    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            ft, classes);

    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<overlume::Vec3> expected;
    BuildChainedLineList(1, wire, expected);

    auto bbox = Bbox(1007, 0, 0, 0, 0, 0, 0, 1);
    bbox.header.frame_id = "base_link";
    auto text = Text(1007, "V_1007");
    text.header.frame_id = "base_link";
    auto path = PathMarker("dynamic_objects_hd_map_path", 1007, wire);
    path.header.frame_id = "base_link";
    path.pose.position.x = 5.0;

    visualization_msgs::msg::MarkerArray msg;
    auto del = DeleteAll();
    del.header.frame_id = "base_link";
    msg.markers = {del, bbox, text, path};

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    ASSERT_EQ(o->predicted_path_count, 2u);
    EXPECT_NEAR(o->predicted_path[0].x, 1000.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[0].y, 5.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[1].x, 1000.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[1].y, 6.0, 1e-9);
}

TEST(DynamicObjects, NanArrowPoseIsDroppedAsMalformed) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(2001, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(2001, "V_2001"));
    auto arrow = Arrow(2001, Pt(0, 0, 0), Pt(3, 4, 0));
    arrow.pose.position.x = std::nan("");
    msg.markers.push_back(arrow);

    a.ingest(msg, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);

    SceneAssembly out;
    a.fill(out);
    const auto* o = FindById(out, 2001);
    ASSERT_NE(o, nullptr);
    EXPECT_DOUBLE_EQ(o->velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(o->velocity.y, 0.0);
}

TEST(DynamicObjects, ZeroQuaternionBboxPoseIsIdentityHeadingNotNan) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(4001, 3.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    msg.markers.push_back(Text(4001, "V_4001"));

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 4001);
    ASSERT_NE(o, nullptr);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
    EXPECT_NEAR(o->position.x, 3.0, 1e-9);
    EXPECT_NEAR(o->position.y, -2.0, 1e-9);
    EXPECT_NEAR(o->heading_rad, 0.0, 1e-9);
}

TEST(DynamicObjects, NonZeroBboxZIsFlattenedToTheMapPlane) {
    TfFixture kTf;
    auto classes = overlume::ros::testing::inference_table();
    DynamicObjectsAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(4002, 3.0, -2.0, 0.83, 0.0, 0.0, 0.0, 1.0));
    msg.markers.push_back(Text(4002, "V_4002"));

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 4002);
    ASSERT_NE(o, nullptr);
    EXPECT_DOUBLE_EQ(o->position.z, 0.0);
}
