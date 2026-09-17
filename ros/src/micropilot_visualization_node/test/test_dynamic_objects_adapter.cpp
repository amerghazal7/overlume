/** @file test_dynamic_objects_adapter.cpp
 *  @brief DynamicObjectsAdapter + class inference tests.
 *
 *  Fixture note: perception_dynamic_objects_list_0.yaml is bbox+text only
 *  (every track's first frame has zero arrows); _list_1.yaml adds arrows but
 *  none zero-length, and neither carries a hd_map_path/hd_map_path_dots
 *  namespace or a disjoint LINE_LIST. Those cases are hand-built
 *  MarkerArrays below instead of new committed fixtures.
 */
#include "micropilot_visualization_node/adapters/dynamic_objects.hpp"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using micropilot::visualization_app::FrameTransformer;
using micropilot::visualization_app::SceneAssembly;
using mpviz_node::ClassInferenceTable;
using mpviz_node::DynamicObjectsAdapter;

namespace
{

// Same shape as test_hd_map_adapter.cpp's TfFixture -- an empty buffer is
// enough for every test whose markers stay in the "map" frame.
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

visualization_msgs::msg::Marker DeleteAll()
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.action = 3;
    return m;
}

geometry_msgs::msg::Point Pt(double x, double y, double z = 0.0)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

visualization_msgs::msg::Marker Bbox(int32_t id, double x, double y, double z, double qx, double qy,
                                     double qz, double qw, double sx = 4.0, double sy = 2.0,
                                     double sz = 1.5)
{
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

visualization_msgs::msg::Marker Text(int32_t id, const std::string& text)
{
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
                                      geometry_msgs::msg::Point p1)
{
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
                                           const std::vector<geometry_msgs::msg::Point>& pts)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = id;
    m.type = 5;  // LINE_LIST
    m.action = 0;
    m.points = pts;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    return m;
}

// n_segments chained segments over a straight line: waypoint i = (i, 0, 0),
// i in [0, n_segments]. Returns the pairwise-duplicated wire representation
// AND the expected unique polyline (for assertions).
void BuildChainedLineList(size_t n_segments, std::vector<geometry_msgs::msg::Point>& wire,
                          std::vector<mpviz::Vec3>& expected)
{
    wire.clear();
    expected.clear();
    for (size_t i = 0; i <= n_segments; ++i) expected.push_back({static_cast<double>(i), 0.0, 0.0});
    for (size_t s = 0; s < n_segments; ++s)
    {
        wire.push_back(Pt(expected[s].x, expected[s].y, expected[s].z));
        wire.push_back(Pt(expected[s + 1].x, expected[s + 1].y, expected[s + 1].z));
    }
}

const mpviz::TrackedObject* FindById(const SceneAssembly& out, uint32_t id)
{
    for (const auto& o : out.objects)
    {
        if (o.id == id) return &o;
    }
    return nullptr;
}

}  // namespace

// ── Class inference (Task 3 Step 2) ─────────────────────────────────────────

TEST(ClassInference, PrefixWinsOverFootprint)
{
    const auto cfg = mpviz_node::testing::inference_table();
    EXPECT_EQ(mpviz_node::infer(cfg, "V_1105", {5.03, 2.15, 1.65}), mpviz::ObjectClass::CAR);
}

TEST(ClassInference, UnknownPrefixFallsBackToFootprintBands)
{
    const auto cfg = mpviz_node::testing::inference_table();
    EXPECT_EQ(mpviz_node::infer(cfg, "Z_9", {0.6, 0.6, 1.8}), mpviz::ObjectClass::PEDESTRIAN);
    EXPECT_EQ(mpviz_node::infer(cfg, "Z_9", {1.9, 0.7, 1.7}), mpviz::ObjectClass::CYCLIST);
    EXPECT_EQ(mpviz_node::infer(cfg, "Z_9", {12.0, 2.5, 3.2}), mpviz::ObjectClass::BUS);
}

TEST(ClassInference, NoLabelAndNoMatchingBandIsUnknownNotACrash)
{
    const auto cfg = mpviz_node::testing::inference_table();
    EXPECT_EQ(mpviz_node::infer(cfg, nullptr, {0, 0, 0}), mpviz::ObjectClass::UNKNOWN);
}

// ── Fusion / field-source rules (Task 3 Step 1) ──────────────────────────────

TEST(DynamicObjects, FourNamespacesFuseIntoOneTrackedObject)
{
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    std::vector<geometry_msgs::msg::Point> path_wire;
    std::vector<mpviz::Vec3> path_expected;
    BuildChainedLineList(72, path_wire, path_expected);  // 144 pts -> 73 verts

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, -66.8, -13.2, 0.39, 0.0, 0.0, -1.0, 0.0));  // yaw == pi
    msg.markers.push_back(Text(1007, "V_1007"));
    msg.markers.push_back(Arrow(1007, Pt(0, 0, 0), Pt(2, 0, 0)));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path", 1007, path_wire));

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    ASSERT_GT(out.objects.size(), 0u);
    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    EXPECT_GT(o->dimensions.x, 0.0);              // from *_bbox scale (true extents)
    ASSERT_NE(o->label, nullptr);
    EXPECT_EQ(std::string(o->label).front(), 'V');  // from *_text
    EXPECT_EQ(o->cls, mpviz::ObjectClass::CAR);
    EXPECT_DOUBLE_EQ(o->last_update_sec, 1.0);
}

TEST(DynamicObjects, HeadingComesFromTheBboxPoseOrientationNotTheArrow)
{
    // Real bag quaternions (perception_dynamic_objects_list_0.yaml, frame
    // "map" -- identity transform) -- verified against an independent
    // atan2(2(wz+xy), 1-2(y^2+z^2)) computation, NOT the adapter's own
    // formula, so this is an actual check of the math, not a tautology.
    auto msg = mpviz_node::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    // This fixture has NO arrow markers at all (every track's first frame)
    // -- if heading were ever read from the arrow this would be 0.0 for
    // every object instead of the bbox-derived values below.
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

TEST(DynamicObjects, ArrowSuppliesVelocityOnlyAndZeroLengthIsZeroVelocity)
{
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    // Moving object: arrow (0,0,0)->(3,4,0), length 5.
    msg.markers.push_back(Bbox(2001, 0, 0, 0, 0, 0, 0, 1));  // identity orientation -> yaw 0
    msg.markers.push_back(Text(2001, "V_2001"));
    msg.markers.push_back(Arrow(2001, Pt(0, 0, 0), Pt(3, 4, 0)));
    // Stopped object: zero-length arrow.
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
    EXPECT_DOUBLE_EQ(moving->heading_rad, 0.0);  // untouched by the arrow

    const auto* stopped = FindById(out, 2002);
    ASSERT_NE(stopped, nullptr);
    EXPECT_DOUBLE_EQ(stopped->velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(stopped->velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(stopped->velocity.z, 0.0);
}

TEST(DynamicObjects, FirstFramePerObjectHasNoArrow_StillEmitsObject)
{
    auto msg = mpviz_node::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1001);
    ASSERT_NE(o, nullptr);           // still rendered despite no arrow this frame
    EXPECT_DOUBLE_EQ(o->velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(o->velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(o->velocity.z, 0.0);
    EXPECT_NEAR(o->heading_rad, 3.139082256947653, 1e-6);  // correct bbox heading, not 0
    EXPECT_GT(o->dimensions.x, 0.0);
}

// ── Predicted path (Task 3 Step 1a) ──────────────────────────────────────────

TEST(DynamicObjects, PredictedPathLineListPairsCollapseToAPolyline)
{
    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<mpviz::Vec3> expected;
    BuildChainedLineList(72, wire, expected);  // 144 pts, 72 segments, 73 verts
    ASSERT_EQ(wire.size(), 144u);

    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
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
    EXPECT_EQ(o->predicted_path_count, 73u);  // NOT 144: half+1, deduplicated
    ASSERT_GT(o->predicted_path_count, 0u);
    EXPECT_DOUBLE_EQ(o->predicted_path[0].x, expected.front().x);
    EXPECT_DOUBLE_EQ(o->predicted_path[o->predicted_path_count - 1].x, expected.back().x);
}

TEST(DynamicObjects, NonContiguousLineListIsDroppedNotStitched)
{
    // Free-function check: two genuinely disjoint segments (p1 != p2).
    std::vector<geometry_msgs::msg::Point> disjoint = {Pt(0, 0, 0), Pt(1, 0, 0), Pt(5, 0, 0),
                                                        Pt(6, 0, 0)};
    std::vector<mpviz::Vec3> out_pts;
    EXPECT_FALSE(mpviz_node::line_list_to_polyline(disjoint, out_pts));

    // Adapter-level: the object still renders, just without a predicted path.
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
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
    ASSERT_NE(o, nullptr);  // object still renders, pathless
    EXPECT_EQ(o->predicted_path_count, 0u);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
}

TEST(DynamicObjects, PredictedPathReadsPerVertexColorsTopLevelRgbaIsBlack)
{
    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<mpviz::Vec3> expected;
    BuildChainedLineList(4, wire, expected);  // small chain, 5 verts

    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(1007, "V_1007"));
    auto path = PathMarker("dynamic_objects_hd_map_path", 1007, wire);
    // Per-vertex colors populated (ncolors == npts), top-level rgba black --
    // exactly the shape the recorded bag ships. TrackedObject has no color
    // field at all (scene.h is frozen to {..., predicted_path, ...}) so
    // there is nowhere for this to leak into; this asserts the geometry
    // conversion is unaffected by colors[] being present.
    for (size_t i = 0; i < wire.size(); ++i)
    {
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

TEST(DynamicObjects, PathDotsNamespaceIsDroppedByRuleAndCounted)
{
    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<mpviz::Vec3> expected;
    BuildChainedLineList(2, wire, expected);  // 4 pts -> 3 verts

    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    auto row = mpviz_node::testing::urban_row("/perception/dynamic_objects_list");
    DynamicObjectsAdapter a(row, kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(1007, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(1007, "V_1007"));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path", 1007, wire));
    // Same path, redrawn as dots -- must be dropped by rule, not fused in
    // as a second predicted path (which would double predicted_path_count).
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path_dots", 1007, wire));
    msg.markers.push_back(PathMarker("dynamic_objects_hd_map_path_dots", 1008, wire));

    ASSERT_EQ(mpviz_node::classify(row, "dynamic_objects_hd_map_path_dots"), mpviz_node::NsRender::kDrop);
    ASSERT_EQ(mpviz_node::classify(row, "dynamic_objects_hd_map_path"), mpviz_node::NsRender::kPolyline);

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 1007);
    ASSERT_NE(o, nullptr);
    EXPECT_EQ(o->predicted_path_count, 3u);  // the converted polyline's count, NOT 2x it
    EXPECT_EQ(a.stats().dropped_by_rule, 2u);  // the two _dots markers above
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

// ── DELETEALL / malformed (Task 3 Step 1, last two tests) ────────────────────

TEST(DynamicObjects, DeleteAllMarkerClearsPreviousFrame)
{
    auto msg = mpviz_node::testing::load_marker_array("perception_dynamic_objects_list_0.yaml");
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
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

TEST(DynamicObjects, MalformedMarkersDroppedAndCounted)
{
    // Hand-edited fixture: NaN bbox pose (3001), zero-extent bbox (3002),
    // an empty text string (3003), a bbox with no matching text marker at
    // all (3004) -- all four dropped and counted -- plus one fully valid
    // track (3005) proving neighbours survive.
    auto msg = mpviz_node::testing::load_marker_array("dynamic_objects_malformed.yaml");
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_malformed, 4u);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.objects.size(), 1u);
    EXPECT_EQ(out.objects[0].id, 3005u);
}

// ── PATH/ARROW points[] are RELATIVE to that marker's own pose ─────────────

namespace
{
constexpr double kQuarterTurn = 0.70710678118654752440;  // sin/cos(45 deg) -> +90 deg yaw
}  // namespace

TEST(DynamicObjects, PathMarkerOwnPoseComposesBeforePolylineConversion)
{
    // The PATH marker's own pose (5,0,0), NO rotation, applied to a local
    // 2-vertex chain (0,0,0)->(1,0,0) -> polyline lands at (5,0,0),(6,0,0).
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<mpviz::Vec3> expected;
    BuildChainedLineList(1, wire, expected);  // local verts (0,0,0),(1,0,0)

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

TEST(DynamicObjects, ArrowPureTranslationPoseLeavesVelocityUnchanged)
{
    // Translation cancels in the p1-p0 difference -- a translate-only arrow
    // pose must reproduce the identity-pose velocity exactly (see
    // ArrowSuppliesVelocityOnlyAndZeroLengthIsZeroVelocity's (3,4) case).
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(2001, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(2001, "V_2001"));
    auto arrow = Arrow(2001, Pt(0, 0, 0), Pt(3, 4, 0));
    arrow.pose.position.x = 500.0;
    arrow.pose.position.y = -200.0;  // pure translation, no rotation
    msg.markers.push_back(arrow);

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 2001);
    ASSERT_NE(o, nullptr);
    EXPECT_DOUBLE_EQ(o->velocity.x, 3.0);
    EXPECT_DOUBLE_EQ(o->velocity.y, 4.0);
}

TEST(DynamicObjects, ArrowNinetyDegreeYawPoseRotatesVelocity)
{
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(2001, 0, 0, 0, 0, 0, 0, 1));
    msg.markers.push_back(Text(2001, "V_2001"));
    auto arrow = Arrow(2001, Pt(0, 0, 0), Pt(3, 4, 0));
    arrow.pose.orientation.z = kQuarterTurn;
    arrow.pose.orientation.w = kQuarterTurn;  // +90 deg yaw, no translation
    msg.markers.push_back(arrow);

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 2001);
    ASSERT_NE(o, nullptr);
    // (3,4) rotated +90 deg about Z -> (-4,3).
    EXPECT_NEAR(o->velocity.x, -4.0, 1e-9);
    EXPECT_NEAR(o->velocity.y, 3.0, 1e-9);
}

TEST(DynamicObjects, MarkerPoseAndNonMapFrameComposeInFrameInsideOrder)
{
    // Combined case: frame_transform * (marker_pose * point), never the
    // other way round. PATH marker pose: pure translation (5,0,0), no
    // rotation. Frame map<-base_link: yaw +90 deg AND translation
    // (1000,0,0). Correct order composes the marker's translation INSIDE
    // the header frame before the frame's own rotation is applied -- the
    // wrong order yields a visibly different point, which this test would
    // catch.
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    geometry_msgs::msg::TransformStamped xf;
    xf.header.frame_id = "map";
    xf.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    xf.child_frame_id = "base_link";
    xf.transform.translation.x = 1000.0;
    xf.transform.rotation.z = kQuarterTurn;
    xf.transform.rotation.w = kQuarterTurn;  // +90 deg yaw
    buffer.setTransform(xf, "test_authority", /*is_static=*/true);
    FrameTransformer ft(buffer);

    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            ft, classes);

    std::vector<geometry_msgs::msg::Point> wire;
    std::vector<mpviz::Vec3> expected;
    BuildChainedLineList(1, wire, expected);  // local verts (0,0,0),(1,0,0)

    auto bbox = Bbox(1007, 0, 0, 0, 0, 0, 0, 1);
    bbox.header.frame_id = "base_link";
    auto text = Text(1007, "V_1007");
    text.header.frame_id = "base_link";
    auto path = PathMarker("dynamic_objects_hd_map_path", 1007, wire);
    path.header.frame_id = "base_link";
    path.pose.position.x = 5.0;  // marker pose: translation only, in base_link

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
    // vertex (0,0,0): marker pose -> (5,0,0); frame (yaw90, +1000x) ->
    // (-0+1000, 5+0, 0) = (1000, 5, 0).
    EXPECT_NEAR(o->predicted_path[0].x, 1000.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[0].y, 5.0, 1e-9);
    // vertex (1,0,0): marker pose -> (6,0,0); frame -> (1000, 6, 0).
    EXPECT_NEAR(o->predicted_path[1].x, 1000.0, 1e-9);
    EXPECT_NEAR(o->predicted_path[1].y, 6.0, 1e-9);
}

TEST(DynamicObjects, NanArrowPoseIsDroppedAsMalformed)
{
    // A pose on an arrow marker is normal Marker semantics -- a NaN pose is
    // not. The object still renders (bbox+text are fine); velocity stays
    // zero because the malformed arrow contributed nothing.
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
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

TEST(DynamicObjects, ZeroQuaternionBboxPoseIsIdentityHeadingNotNan)
{
    // rviz treats a zero-filled orientation as identity; tf2 would make
    // getYaw() NaN and corrupt the object's transform. Zero quat -> object
    // still emitted, heading 0, position honoured, nothing counted
    // malformed.
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(4001, 3.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0));  // all-zero quat
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

TEST(DynamicObjects, NonZeroBboxZIsFlattenedToTheMapPlane)
{
    // bbox centers carry z (half the box height); rendered objects must
    // flatten to the 2D HD-map plane.
    TfFixture kTf;
    auto classes = mpviz_node::testing::inference_table();
    DynamicObjectsAdapter a(mpviz_node::testing::urban_row("/perception/dynamic_objects_list"),
                            kTf.tf, classes);

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(DeleteAll());
    msg.markers.push_back(Bbox(4002, 3.0, -2.0, 0.83, 0.0, 0.0, 0.0, 1.0));  // z = half height
    msg.markers.push_back(Text(4002, "V_4002"));

    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const auto* o = FindById(out, 4002);
    ASSERT_NE(o, nullptr);
    EXPECT_DOUBLE_EQ(o->position.z, 0.0);
}
