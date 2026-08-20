/** @file test_hd_map_adapter.cpp
 *  @brief HdMapAdapter tests (Epic 2 Task 2 / VM-024).
 */
#include "micropilot_visualization_node/adapters/hd_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using micropilot::visualization_app::FrameTransformer;
using micropilot::visualization_app::SceneAssembly;

namespace
{

// Hand-built tf2_ros::Buffer + FrameTransformer, identical fixture style to
// test_frame_transform.cpp -- an empty buffer is enough for every test
// whose markers stay in the "map" frame (FrameTransformer's identity
// shortcut never touches it).
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

}  // namespace

TEST(HdMapAdapter, LocalElementsFixtureYieldsLanesAndCrosswalks)
{
    auto msg = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(mpviz_node::testing::urban_row("/hd_map_local_elements"), kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);
    SceneAssembly out;
    a.fill(out);  // appends, never overwrites

    EXPECT_GT(out.map_elements.size(), 0u);
    // Old -> new count math (VM-036-adjacent, user directive 2026-08-20:
    // urban's centerline_ rule now ships dashed: true). Before dashing this
    // fixture's committed markers (16 centerline_, 16 left_boundary_, 16
    // right_boundary_, 5 crosswalk_ polygons, 5 crosswalk_stopline_
    // polylines; the 64 centerline_arrows_ markers are all dropped by rule
    // and were never counted) yielded 16+16+16+5+5 = 58 MapElements.
    // Dashing only touches the 16 centerline_ markers; summing each one's
    // REAL arc length (verified against the committed fixture, not
    // guessed) through kDashLenM=1.5/kGapLenM=1.5 chopping gives 204 dash
    // pieces in place of those 16 markers (per-marker: 934->14, 782->15,
    // 2225->15, 1029->8, 838->13, 955->9, 690->14, 685->15, 792->14,
    // 12->15, 68->14, 813->9, 787->14, 7->14, 1305->8, 1453->13). New
    // total: 204 + 16 + 16 + 5 + 5 = 246.
    EXPECT_EQ(out.map_elements.size(), 246u);
    // is_polygon comes from the row's namespace rules (crosswalk_ ->
    // polygon) and from NOTHING else -- on the wire every hd_map marker in
    // this fixture is a LINE_STRIP.
    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                             [](const mpviz::MapElement& m) { return m.is_polygon == 1; }));
    // ...and the crosswalk_stopline_ sibling is kept as a polyline, not
    // silently dropped.
    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                             [](const mpviz::MapElement& m) { return m.is_polygon == 0; }));

    // Step 7's dump-and-exit: run once with MPVIZ_EMIT_GEOM set to emit
    // the .geom fixture the library-side MapGolden.* tests render from
    // (epic2 plan, Task 2 Step 7). `.geom` format (golden.cpp/golden.hpp):
    // one element per line, `<is_polygon> <n> <x1> <y1> <z1> ... <xn> <yn>
    // <zn>`. ponytail: a text dump, not a serializer.
    if (const char* geom_path = std::getenv("MPVIZ_EMIT_GEOM"))
    {
        std::ofstream geom(geom_path);
        geom << std::setprecision(12);
        for (const auto& e : out.map_elements)
        {
            geom << static_cast<int>(e.is_polygon) << ' ' << e.point_count;
            for (uint32_t i = 0; i < e.point_count; ++i)
            {
                geom << ' ' << e.points[i].x << ' ' << e.points[i].y << ' ' << e.points[i].z;
            }
            geom << '\n';
        }
    }
}

TEST(HdMapAdapter, SimProfileRowMakesCrosswalksPolygonsToo)
{
    // The assertion above covers ONE row's rules. /sim/hd_map/markers --
    // the only full-extent map source -- uses namespace "crosswalks"
    // (plural, no numeric suffix), which urban's "crosswalk_" prefix does
    // NOT match. Same adapter, sim row, same shape of assertion.
    auto msg = mpviz_node::testing::load_marker_array("sim_hd_map_markers_0.yaml");
    TfFixture kTf;
    auto row = mpviz_node::testing::sim_row("/sim/hd_map/markers");
    mpviz_node::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                             [](const mpviz::MapElement& m) { return m.is_polygon == 1; }));

    // Relationship, not a magic number (epic2 plan, "Fixture strategy" /
    // Task 2 Step 6 comment) -- this must survive a re-cut fixture:
    //   dropped_by_rule == count of fixture markers whose ns classifies kDrop
    //   dropped_malformed == 0
    uint64_t expected_drops = 0;
    for (const auto& m : msg.markers)
    {
        if (m.action == 3) continue;  // DELETEALL isn't a namespace decision
        if (mpviz_node::classify(row, m.ns) == mpviz_node::NsRender::kDrop) ++expected_drops;
    }
    EXPECT_GT(expected_drops, 0u);
    EXPECT_EQ(a.stats().dropped_by_rule, expected_drops);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HdMapAdapter, ArrowNamespaceIsDroppedByLongestPrefixWins)
{
    // "centerline_" -> polyline and "centerline_arrows_" -> drop in the
    // same row: ns "centerline_arrows_0" matches BOTH by prefix and the
    // LONGER rule must win. A naive prefix include-list would admit every
    // arrow marker and the epic's headline decimation claim evaporates.
    auto msg = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);

    // 64 centerline_arrows_* markers in this fixture (verified against the
    // committed YAML) -- every one dropped BY RULE, never counted as
    // malformed even though they carry no points[] on the wire.
    EXPECT_EQ(a.stats().dropped_by_rule, 64u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HdMapAdapter, NonMapFrameMessageIsTransformedNotCopied)
{
    // /hd_map_local_elements is frame "map" today, but the adapter goes
    // through FrameTransformer like every other adapter: feed the same
    // fixture with header.frame_id = "base_link" and a known
    // map<-base_link, assert the points moved.
    auto msg = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");
    for (auto& m : msg.markers) m.header.frame_id = "base_link";

    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    geometry_msgs::msg::TransformStamped xf;
    xf.header.frame_id = "map";
    xf.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    xf.child_frame_id = "base_link";
    xf.transform.translation.x = 1000.0;
    xf.transform.translation.y = 2000.0;
    xf.transform.rotation.w = 1.0;
    buffer.setTransform(xf, "test_authority", /*is_static=*/true);
    FrameTransformer ft(buffer);

    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, ft);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);
    ASSERT_GT(out.map_elements.size(), 0u);

    // A straight copy would leave every point within ~100m of the map
    // ORIGIN (the fixture's own recorded coordinates); the transform
    // above must move all of them out past (900, 1900).
    bool any_far = false;
    for (const auto& e : out.map_elements)
    {
        for (uint32_t i = 0; i < e.point_count; ++i)
        {
            if (e.points[i].x > 900.0 && e.points[i].y > 1900.0) any_far = true;
        }
    }
    EXPECT_TRUE(any_far);
}

TEST(HdMapAdapter, TfLookupFailureDropsTheMessageAndCounts)
{
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);

    auto good = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");
    a.ingest(good, 1.0);
    SceneAssembly before;
    a.fill(before);
    ASSERT_GT(before.map_elements.size(), 0u);

    auto bad = good;
    for (auto& m : bad.markers) m.header.frame_id = "sensor_frame_nobody_ever_published";
    a.ingest(bad, 2.0);
    EXPECT_EQ(a.stats().dropped_no_tf, 1u);

    // stats().dropped_no_tf == 1, previous elements still rendered (spec
    // §9: "keeps rendering what exists and fades", never wrong-place
    // geometry) -- never a silent identity substitution either.
    SceneAssembly after;
    a.fill(after);
    EXPECT_EQ(after.map_elements.size(), before.map_elements.size());
}

TEST(HdMapAdapter, DeleteAllClearsPreviousElements)
{
    // Every real message starts with a DELETEALL marker (ns="", id=0);
    // ingesting two messages must not accumulate.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);
    auto msg = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");

    a.ingest(msg, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_GT(first.map_elements.size(), 0u);

    // Gap >= the row's rate-limit cooldown (max_rate_hz: 2.0 -> 0.5s) so
    // this second ingest is actually processed, not skipped.
    a.ingest(msg, 3.0);
    SceneAssembly second;
    a.fill(second);
    EXPECT_EQ(second.map_elements.size(), first.map_elements.size());
}

TEST(HdMapAdapter, MalformedMarkersAreDroppedAndCounted)
{
    // Hand-edited fixture: a LINE_STRIP with 1 point, one with a NaN
    // point, one with an empty points[]. All three dropped; the valid
    // marker in the same message still comes through.
    auto msg = mpviz_node::testing::load_marker_array("hd_map_malformed_0.yaml");
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_malformed, 3u);
    EXPECT_EQ(a.stats().dropped_by_rule, 0u);

    SceneAssembly out;
    a.fill(out);
    // Old -> new: urban's centerline_ rule now ships dashed: true, and
    // centerline_ok is a straight 10 m 2-point line -- it no longer stays
    // ONE element. At kDashLenM=1.5/kGapLenM=1.5: dashes at [0,1.5],
    // [3,4.5],[6,7.5],[9,10] = 4 kept runs (the last is 1.0 m, still >=
    // the 0.25 m drop threshold), each a straight 2-point piece (no
    // interior vertices to preserve on a 2-point input).
    ASSERT_EQ(out.map_elements.size(), 4u);
    for (const auto& e : out.map_elements) EXPECT_EQ(e.point_count, 2u);
}

TEST(HdMapAdapter, RateLimitHonoursMaxRateHz)
{
    // max_rate_hz: 2.0 -> ingesting faster than 2 Hz rebuilds at most
    // twice a second. Four single-marker messages (no DELETEALL between
    // them, so an accepted one always ADDs a new element) spaced closer
    // than the 0.5s cooldown for two of the four gaps: only the ones
    // spaced >= 0.5s apart from the last ACCEPTED rebuild land.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);

    auto make_one_marker = [](const char* ns, int32_t id, double x0)
    {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.ns = ns;
        m.id = id;
        m.type = 4;
        m.action = 0;
        geometry_msgs::msg::Point p0;
        p0.x = x0;
        geometry_msgs::msg::Point p1;
        p1.x = x0 + 10.0;
        m.points = {p0, p1};
        arr.markers = {m};
        return arr;
    };

    a.ingest(make_one_marker("centerline_a", 1, 0.0), 1.0);   // accepted (first ever)
    a.ingest(make_one_marker("centerline_b", 2, 10.0), 1.1);  // gap 0.1s < 0.5s -> skipped
    a.ingest(make_one_marker("centerline_c", 3, 20.0), 1.2);  // gap 0.2s < 0.5s -> skipped
    a.ingest(make_one_marker("centerline_d", 4, 30.0), 1.6);  // gap 0.6s >= 0.5s -> accepted

    SceneAssembly out;
    a.fill(out);
    // Only the 2 accepted messages' markers made it in -- with no
    // rate-limit honoured all 4 (no DELETEALL between them) would
    // accumulate. Old -> new: urban's centerline_ rule now ships
    // dashed: true, and both accepted markers ("centerline_a", 0->10;
    // "centerline_d", 30->40) are straight 10 m lines -- each chops into
    // 4 dash elements exactly like the worked example in ChopIntoDashes'
    // comment ([0,1.5],[3,4.5],[6,7.5],[9,10]), so 2 accepted markers ->
    // 2*4 = 8 elements, not 2.
    EXPECT_EQ(out.map_elements.size(), 8u);
}

namespace
{

// Builds a one-row profile with a single namespace rule, for the
// dash-chopping tests below -- independent of the shipped config so these
// tests exercise the geometry op in isolation from urban_profile.yaml.
mpviz_node::ProfileRow DashRuleRow(bool dashed)
{
    std::vector<std::string> errs;
    const std::string yaml =
        "name: t\nrows:\n  - {topic: /hd_map, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, ns_default: drop, namespaces:"
        " [{prefix: centerline_,    render: polyline, dashed: " +
        std::string(dashed ? "true" : "false") +
        "},"
        "  {prefix: left_boundary_, render: polyline}]}\n";
    auto p = mpviz_node::load_profile_string(yaml, errs);
    if (!p) throw std::runtime_error("DashRuleRow: profile failed to parse");
    return p->rows[0];
}

// One straight 10 m LINE_STRIP marker under the given namespace -- the
// same shape RateLimitHonoursMaxRateHz's make_one_marker uses above.
visualization_msgs::msg::MarkerArray StraightTenMeterMarker(const char* ns)
{
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = 1;
    m.type = 4;
    m.action = 0;
    geometry_msgs::msg::Point p0;
    geometry_msgs::msg::Point p1;
    p1.x = 10.0;
    m.points = {p0, p1};
    arr.markers = {m};
    return arr;
}

}  // namespace

TEST(HdMapAdapter, DashedCenterlineChopsByArcLengthWithInterpolatedEndpoints)
{
    // 10 m straight centerline, dashed: true, at kDashLenM=1.5/kGapLenM=1.5:
    // keep windows start every (1.5+1.5)=3.0 m -- [0,1.5],[3,4.5],[6,7.5],
    // [9,10] (the last window is truncated by the polyline's own end to
    // 1.0 m, still >= the 0.25 m drop threshold) = 4 kept dash elements,
    // each a straight 2-point piece with exactly interpolated endpoints.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/true), kTf.tf);
    a.ingest(StraightTenMeterMarker("centerline_x"), 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 4u);

    // storage_ holds exactly one (ns, id) key here, and fill() walks its
    // vector<StoredElement> in chop (insertion) order, so element i IS
    // dash i -- no sorting needed to make this assertion order-independent.
    const double kExpectedX[4][2] = {{0.0, 1.5}, {3.0, 4.5}, {6.0, 7.5}, {9.0, 10.0}};
    for (int i = 0; i < 4; ++i)
    {
        const auto& e = out.map_elements[i];
        ASSERT_EQ(e.point_count, 2u);
        EXPECT_NEAR(e.points[0].x, kExpectedX[i][0], 1e-9);
        EXPECT_NEAR(e.points[1].x, kExpectedX[i][1], 1e-9);
        EXPECT_NEAR(e.points[0].y, 0.0, 1e-9);
        EXPECT_NEAR(e.points[1].y, 0.0, 1e-9);
        EXPECT_EQ(e.is_polygon, 0u);
    }
}

TEST(HdMapAdapter, NonDashedNamespaceOfIdenticalGeometryStaysOneElement)
{
    // Same 10 m geometry as the dashed test above, but under a namespace
    // whose rule has no dashed: true -- must stay ONE element, byte-for-byte
    // the pre-dashing behavior. Two variants: a boundary-shaped namespace
    // (left_boundary_, never dashed by design) and the SAME centerline_
    // prefix with its rule's dashed flag explicitly false.
    TfFixture kTf;

    mpviz_node::HdMapAdapter boundary(DashRuleRow(/*dashed=*/true), kTf.tf);
    boundary.ingest(StraightTenMeterMarker("left_boundary_x"), 1.0);
    SceneAssembly boundary_out;
    boundary.fill(boundary_out);
    ASSERT_EQ(boundary_out.map_elements.size(), 1u);
    EXPECT_EQ(boundary_out.map_elements[0].point_count, 2u);

    mpviz_node::HdMapAdapter not_dashed(DashRuleRow(/*dashed=*/false), kTf.tf);
    not_dashed.ingest(StraightTenMeterMarker("centerline_x"), 1.0);
    SceneAssembly not_dashed_out;
    not_dashed.fill(not_dashed_out);
    ASSERT_EQ(not_dashed_out.map_elements.size(), 1u);
    EXPECT_EQ(not_dashed_out.map_elements[0].point_count, 2u);
}

TEST(HdMapAdapter, DashedMarkerStillCountsAsOneIngestedMarkerForStats)
{
    // A dashed centerline is one MARKER on the wire and one ingest() call
    // regardless of how many MapElements it explodes into -- msgs and the
    // dropped_* counters must not scale with dash count.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/true), kTf.tf);
    a.ingest(StraightTenMeterMarker("centerline_x"), 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 4u);  // the dash explosion, for context

    EXPECT_EQ(a.stats().msgs, 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
    EXPECT_EQ(a.stats().dropped_by_rule, 0u);
    EXPECT_EQ(a.stats().dropped_no_tf, 0u);
}

// ── rviz-parity fix (user report 2026-08-20): points[] are RELATIVE to
// marker.pose ──────────────────────────────────────────────────────────────

TEST(HdMapAdapter, MarkerPoseComposesRotationBeforeTranslation)
{
    // pose position (10,20,0), yaw +90 deg, points (0,0) and (5,0) -> stored
    // (10,20) and (10,25) -- rotation applied BEFORE translation, per
    // tf2::Transform's own point-multiply composition order. Non-dashed
    // namespace so the marker stays exactly one MapElement.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/false), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "centerline_x";
    m.id = 1;
    m.type = 4;  // LINE_STRIP
    m.action = 0;
    m.pose.position.x = 10.0;
    m.pose.position.y = 20.0;
    constexpr double kQuarterTurn = 0.70710678118654752440;  // sin/cos(45 deg)
    m.pose.orientation.z = kQuarterTurn;
    m.pose.orientation.w = kQuarterTurn;
    geometry_msgs::msg::Point p0;
    geometry_msgs::msg::Point p1;
    p1.x = 5.0;
    m.points = {p0, p1};
    arr.markers = {m};

    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    ASSERT_EQ(out.map_elements.size(), 1u);
    ASSERT_EQ(out.map_elements[0].point_count, 2u);
    EXPECT_NEAR(out.map_elements[0].points[0].x, 10.0, 1e-9);
    EXPECT_NEAR(out.map_elements[0].points[0].y, 20.0, 1e-9);
    EXPECT_NEAR(out.map_elements[0].points[1].x, 10.0, 1e-9);
    EXPECT_NEAR(out.map_elements[0].points[1].y, 25.0, 1e-9);
}

TEST(HdMapAdapter, IdentityMarkerPoseIsByteIdenticalToRawPoints)
{
    // Same shape marker, pose left at its default (all-zero position,
    // identity quaternion -- geometry_msgs' own default, Quaternion.msg's
    // `float64 w 1`) -- must reproduce today's un-posed behaviour exactly:
    // no rotation, no translation (regression for the identity fast path).
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/false), kTf.tf);
    a.ingest(StraightTenMeterMarker("centerline_x"), 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    ASSERT_EQ(out.map_elements[0].point_count, 2u);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[0].x, 0.0);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[0].y, 0.0);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[1].x, 10.0);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[1].y, 0.0);
}

TEST(HdMapAdapter, NanMarkerPoseIsDroppedAsMalformedNotAppliedRaw)
{
    // A pose on a marker is normal Marker semantics -- a NaN pose is not.
    // The malformed marker is dropped; a valid neighbour still comes
    // through (spec §9, "drop the one primitive, never propagate").
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/false), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    auto bad = StraightTenMeterMarker("centerline_x").markers[0];
    bad.id = 1;
    bad.pose.position.x = std::nan("");
    auto good = StraightTenMeterMarker("left_boundary_x").markers[0];
    good.id = 2;
    arr.markers = {bad, good};

    a.ingest(arr, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[1].x, 10.0);
}

TEST(HdMapAdapter, DashChopHappensAfterPoseComposition)
{
    // Review finding 2026-08-20: structurally the chop runs on posed points,
    // but no test pinned it -- a refactor moving ChopIntoDashes ahead of the
    // pose multiply would pass every other test. Same 10 m straight
    // centerline, dashed, posed +100 in x: the first dash must land at
    // [100, 101.5], i.e. chopping ran on POSED coordinates.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/true), kTf.tf);
    auto arr = StraightTenMeterMarker("centerline_x");
    arr.markers[0].pose.position.x = 100.0;
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 4u);
    ASSERT_EQ(out.map_elements[0].point_count, 2u);
    EXPECT_NEAR(out.map_elements[0].points[0].x, 100.0, 1e-9);
    EXPECT_NEAR(out.map_elements[0].points[1].x, 101.5, 1e-9);
}

TEST(HdMapAdapter, ZeroQuaternionPoseIsTreatedAsIdentityRotationNotNan)
{
    // rviz renders a zero-filled orientation as identity (with a console
    // warning); handing it to tf2 NaNs every point and silently voids the
    // whole marker as dropped_malformed. Zero quat + translation -> points
    // still come through translated, nothing counted malformed.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/false), kTf.tf);
    auto arr = StraightTenMeterMarker("centerline_x");
    arr.markers[0].pose.position.y = 7.0;
    arr.markers[0].pose.orientation.w = 0.0;  // all-zero quaternion
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
    ASSERT_EQ(out.map_elements[0].point_count, 2u);
    EXPECT_NEAR(out.map_elements[0].points[0].y, 7.0, 1e-9);
    EXPECT_NEAR(out.map_elements[0].points[1].x, 10.0, 1e-9);
}

TEST(HdMapAdapter, NonZeroZPointsAreFlattenedToTheMapPlane)
{
    // flatten_z (user directive 2026-08-20): the HD map is a 2D plane, so
    // publisher z must not float geometry above it. Default is ON.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/false), kTf.tf);
    auto arr = StraightTenMeterMarker("centerline_x");
    for (auto& p : arr.markers[0].points) p.z = 3.0;
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[0].z, 0.0);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[1].z, 0.0);
}

TEST(HdMapAdapter, FlattenZOffPreservesPublisherZ)
{
    // The configurability half of the directive: a FrameTransformer built
    // with flatten_z=false passes z through untouched -- the switch to flip
    // when the HD-map layer grows 3D coordinates.
    TfFixture kTf;
    micropilot::visualization_app::FrameTransformer tf3d(kTf.buffer, "map", /*flatten_z=*/false);
    mpviz_node::HdMapAdapter a(DashRuleRow(/*dashed=*/false), tf3d);
    auto arr = StraightTenMeterMarker("centerline_x");
    for (auto& p : arr.markers[0].points) p.z = 3.0;
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[0].z, 3.0);
}
