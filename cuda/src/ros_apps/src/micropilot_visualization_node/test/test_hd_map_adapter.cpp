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
    // Epic 3 Task 1 (VM-036, decision #3): dashing moved renderer-side, so
    // the adapter never chops a marker into pieces any more -- one marker,
    // one MapElement, always. This fixture's committed markers: 16
    // centerline_, 16 left_boundary_, 16 right_boundary_, 5 crosswalk_
    // polygons, 5 crosswalk_stopline_ polylines (the 64 centerline_arrows_
    // markers are all dropped by rule and never counted) = 58 elements,
    // PLUS the road-surface fill (decision #5): every one of those 16 lanes
    // carries BOTH boundaries (verified against the committed fixture,
    // "Named fixture gaps" #2), so pairing left_boundary_{id}/
    // right_boundary_{id} by lane_id synthesizes 16 more kind==ROAD_SURFACE
    // elements. Pre-2026-09-08 total: 58 + 16 = 74.
    //
    // User directive 2026-09-08 -- two changes to that count:
    // (1) centerline_ is now `render: drop` (hidden by default) in
    //     urban_profile.yaml -- the 16 centerline_ markers no longer reach
    //     storage_ at all (they land in dropped_by_rule instead, see
    //     ArrowNamespaceIsDroppedByLongestPrefixWins below). 58 - 16 = 42
    //     lane/crosswalk elements + 16 ROAD_SURFACE = 58 total.
    // (2) Road-edge detection (IsRoadEdge()/hd_map.cpp) reclassifies some
    //     LEFT_BOUNDARY/RIGHT_BOUNDARY elements to ROAD_EDGE in place --
    //     the TOTAL count would otherwise be unaffected (same 32 boundary
    //     elements, just re-labeled), so 58 stayed 58 as of that directive.
    //
    // Junction-cleanup directive 2026-09-08 (round 4) -- a THIRD change,
    // MEASURED, not guessed, and CORRECTED by a code-review finding (blocking):
    // the original version of this comment claimed all 24 raw
    // SegSegIntersect2D detections on this fixture were near "a real road
    // edge never legitimately crosses another, so this only ever fires
    // inside a junction" -- false. Re-classifying each of the 24: 21 are
    // shared-endpoint abutments between chained lanelet boundaries (outgoing
    // angle 175.2-179.8 deg -- one continuous straight line through the
    // node -- or 0.5-2.8 deg -- two rails leaving the same node
    // codirectionally, duplicate surveys of the same rail), not crossings at
    // all. SegSegIntersect2D now rejects near-parallel AND near-antiparallel
    // segment pairs (see its own comment) instead of only exactly-parallel
    // ones, and with that fix only ONE of the 24 raw detections survives as
    // an accepted crossing: lanes 792 and 685, crossing at (-40.36,-21.90)
    // (the review's other two candidate "interior" locations,
    // (-46.95,49.99) and (-53.50,-2.95), are themselves near-parallel by
    // angle even though neither t nor u sits at exactly 0/1 -- a chained
    // polyline can carry an extra sample point near a node, so "not an exact
    // endpoint" doesn't imply "not a survey kink"; both are correctly
    // rejected too). That one crossing lands in the interior of BOTH edges
    // with enough clearance either side, so both split into 2 pieces (a
    // 2.0 m back-off gap around (-40.36,-21.90)), for +2 ROAD_EDGE elements:
    // 15 -> 17. Every other promoted ROAD_EDGE element now renders at its
    // full original length -- no more silent endpoint trimming from
    // near-collinear "crossings". LEFT_BOUNDARY/RIGHT_BOUNDARY counts are
    // untouched (the cut never applies to boundaries) and this fixture's
    // window carries no JUNCTION-kind geometry at all (verified:
    // urban_profile.yaml has no `junction` namespace rule on either hd_map
    // row), so the polygon-clip mechanism is the no-op here -- this count
    // moved by the crossing-cut alone.
    // 58 + 2 = 60.
    //
    // Arc-aware cut refinement (follow-up, 2026-09-08) -- RE-MEASURED, not
    // assumed: this count stays 60 (the refinement only ever moves a
    // boundary's own arc-length station -- it can turn 2 pieces into 1 by
    // fully consuming a piece, but never adds or removes a piece by
    // itself, and doesn't do either here). The GEOMETRY does change,
    // though: `left_boundary_792`'s promoted ROAD_EDGE crosses
    // `right_boundary_685` at (-40.36,-21.90) (this fixture's one genuine
    // crossing). Lane 792's real contiguous R<20 m run (by
    // `FindArcSpanNear`'s own definition of an arc) is arc-length 17.114 to
    // 24.822 (73.96 deg total turn, radii 15.09/11.66/5.00/10.02 m) -- this
    // is where the fillet actually starts and ends.
    // CORRECTED (code-review finding 2026-09-08, blocking): an earlier
    // version of this comment claimed the OLD fixed 2.0 m backoff (arc-length
    // 25.328, interpolated) "lands squarely inside lane 792's own recorded
    // corner fillet" -- false by the run's own numbers above: 25.328 sits
    // 0.5052 m PAST the run's far end (24.822), i.e. just OUTSIDE the
    // fillet, not inside it. An earlier version of the refinement itself
    // then only reached arc-length 22.178 (a run vertex, but not the run's
    // own true start) -- STILL inside the fillet, and in fact a REGRESSION
    // under "cut where the arc starts": the un-refined boundary was already
    // past the far end, and the old refinement moved it back INTO the
    // fillet's own sharpest interior vertex (R=4.9973 m). FindArcSpanNear now
    // extends a run past kArcSearchMarginM=6.0 m (see that function's own
    // comment) while curvature keeps clearing kArcRadiusThresholdM, so the
    // kept HEAD piece's own far boundary now lands exactly at arc-length
    // 17.114, the run's own TRUE start (-46.4658864625989,
    // -14.974389719737527) -- verified directly against this test's own
    // MPVIZ_EMIT_GEOM dump. See
    // `ArcSnapReachesTheArcsTrueStartEvenBeyondTheOldSearchMargin` below,
    // which pins this exact case on self-contained synthetic geometry built
    // from this fixture's own real recorded vertices.
    // The piece COUNT is unaffected (still 2 pieces for this lane), so this
    // fixture is silent on the count but not on the geometry; the dedicated
    // ArcSnap*/PureArcCornerConnector*/TerminalArcStub* tests below
    // (synthetic geometry built from the measurement pass's own numbers)
    // and the /tmp/junction_arc_cut_before_after.png visual check against
    // the real bag are what actually prove the mechanism end to end.
    EXPECT_EQ(out.map_elements.size(), 60u);
    // is_polygon comes from the row's namespace rules (crosswalk_ ->
    // polygon) and from NOTHING else -- on the wire every hd_map marker in
    // this fixture is a LINE_STRIP.
    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                             [](const mpviz::MapElement& m) { return m.is_polygon == 1; }));
    // ...and the crosswalk_stopline_ sibling is kept as a polyline, not
    // silently dropped.
    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                             [](const mpviz::MapElement& m) { return m.is_polygon == 0; }));
    // centerline_ is hidden by default (user directive 2026-09-08) -- zero
    // CENTERLINE elements reach fill()'s output for this fixture now.
    EXPECT_EQ(std::count_if(out.map_elements.begin(), out.map_elements.end(),
                             [](const mpviz::MapElement& m) { return m.kind == mpviz::MapKind::CENTERLINE; }),
              0);

    // Road-surface fill (decision #5): 16 synthesized ROAD_SURFACE
    // elements, every one point_count == 2*16 == 32 regardless of the real
    // rail's own recorded point count (lanes 813/955 are 10/11 and 8/9 on
    // the wire -- resampling is what makes this uniform).
    const auto road_count =
        std::count_if(out.map_elements.begin(), out.map_elements.end(),
                       [](const mpviz::MapElement& m) { return m.kind == mpviz::MapKind::ROAD_SURFACE; });
    EXPECT_EQ(road_count, 16);
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_SURFACE) EXPECT_EQ(e.point_count, 32u);
    }

    // Road-edge detection (user directive 2026-09-08, IsRoadEdge() in
    // hd_map.cpp): MEASURED against this fixture's real geometry (see
    // hd_map.cpp's own comment on kRoadEdgeCoincidenceThresholdM for the
    // full worked distances) -- of the 16 lanes' 32 boundary elements, 15
    // have no coincident opposite-side twin and promote to ROAD_EDGE (9
    // stay LEFT_BOUNDARY, 8 stay RIGHT_BOUNDARY); lane 934 is the one lane
    // fully interior on both sides (paired left AND right) and contributes
    // no ROAD_EDGE. left/right/promoted-before-cutting still sums to 32,
    // the original boundary total -- promotion re-labels, it never drops or
    // duplicates an element. The junction-cleanup mutual-crossing cut then
    // splits 2 of those 15 ROAD_EDGE elements (lanes 792/685, the one
    // genuine crossing measured above) into 2 pieces each, so the ELEMENT
    // count carrying kind==ROAD_EDGE is 17, not 15 -- left_count/right_count
    // are untouched (boundaries never get the crossing cut), so the
    // three-way sum is 32 + 2 == 34, not 32.
    const auto left_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const mpviz::MapElement& m) { return m.kind == mpviz::MapKind::LEFT_BOUNDARY; });
    const auto right_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const mpviz::MapElement& m) { return m.kind == mpviz::MapKind::RIGHT_BOUNDARY; });
    const auto road_edge_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const mpviz::MapElement& m) { return m.kind == mpviz::MapKind::ROAD_EDGE; });
    EXPECT_EQ(left_count, 9);
    EXPECT_EQ(right_count, 8);
    EXPECT_EQ(road_edge_count, 17);
    EXPECT_EQ(left_count + right_count + road_edge_count, 34);
    // Lane 934 is the measured fully-interior exemplar: both its boundaries
    // stay LEFT_BOUNDARY/RIGHT_BOUNDARY, never ROAD_EDGE.
    for (const auto& e : out.map_elements)
    {
        if (e.lane_id != 934u) continue;
        if (e.kind == mpviz::MapKind::LEFT_BOUNDARY || e.kind == mpviz::MapKind::RIGHT_BOUNDARY ||
            e.kind == mpviz::MapKind::ROAD_SURFACE)
        {
            continue;
        }
        ADD_FAILURE() << "lane 934 has an unexpected kind " << static_cast<int>(e.kind)
                      << " -- it is measured fully-interior and should never promote to ROAD_EDGE";
    }

    // Arc-aware cut refinement (follow-up, 2026-09-08) -- lane 792's own
    // HEAD piece (see the count comment above) is measured to end exactly
    // at (-46.4658864625989, -14.974389719737527), a REAL recorded vertex --
    // not the OLD fixed-backoff interpolated point this same fixture
    // produced before the refinement. Pinned by coordinate, not just by
    // count, since the piece COUNT alone (17 ROAD_EDGE, unchanged above)
    // cannot tell the two behaviours apart. CORRECTED (code-review finding
    // 2026-09-08, blocking): an earlier version of this test pinned
    // (-44.9537814989, -19.7772110336) here -- the window-clipped REMAINDER
    // of lane 792's own real corner fillet that the OLD +/-6.0 m search
    // margin alone could reach, NOT the fillet's own true start. See
    // `ArcSnapReachesTheArcsTrueStartEvenBeyondTheOldSearchMargin` below,
    // which pins this exact case on self-contained synthetic geometry.
    {
        std::vector<mpviz::MapElement> lane792;
        for (const auto& e : out.map_elements)
        {
            if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 792u) lane792.push_back(e);
        }
        ASSERT_EQ(lane792.size(), 2u);
        std::sort(lane792.begin(), lane792.end(),
                  [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                      return a.points[0].y > b.points[0].y;  // head piece starts highest (y~2.1)
                  });
        const auto& head = lane792[0];
        EXPECT_NEAR(head.points[head.point_count - 1].x, -46.4658864625989, 1e-6);
        EXPECT_NEAR(head.points[head.point_count - 1].y, -14.974389719737527, 1e-6);
    }

    // Step 7's dump-and-exit: run once with MPVIZ_EMIT_GEOM set to emit
    // the .geom fixture the library-side MapGolden.* tests render from
    // (epic2 plan, Task 2 Step 7; format extended Epic 3 Task 1 to carry
    // kind/lane_id -- see golden.hpp/golden.cpp). One element per line:
    // `<is_polygon> <kind> <lane_id> <n> <x1> <y1> <z1> ... <xn> <yn> <zn>`.
    // ponytail: a text dump, not a serializer.
    if (const char* geom_path = std::getenv("MPVIZ_EMIT_GEOM"))
    {
        std::ofstream geom(geom_path);
        geom << std::setprecision(12);
        for (const auto& e : out.map_elements)
        {
            geom << static_cast<int>(e.is_polygon) << ' ' << static_cast<int>(e.kind) << ' '
                 << e.lane_id << ' ' << e.point_count;
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

TEST(HdMapAdapter, SimCrosswalksPluralUnsuffixedGetsCrosswalkKind)
{
    // Review finding (VM-036 Task 1 Step 3): test_profile.cpp:157 only
    // covers urban's "crosswalk_" rule at rule level (match_rule/classify).
    // Nothing exercises the ADAPTER on sim's plural, unsuffixed "crosswalks"
    // namespace -- map_elements.cpp gates build_crosswalk_hatch() on
    // kind == CROSSWALK, so if sim_profile.yaml's row ever lost its
    // `kind: crosswalk`, is_polygon would still be set (SimProfileRowMakes-
    // CrosswalksPolygonsToo, above, would stay green) but hatching would go
    // dark on sim's only full-extent map source, silently.
    auto msg = mpviz_node::testing::load_marker_array("sim_hd_map_markers_0.yaml");
    TfFixture kTf;
    auto row = mpviz_node::testing::sim_row("/sim/hd_map/markers");
    mpviz_node::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    bool found = false;
    for (const auto& e : out.map_elements)
    {
        if (e.is_polygon != 1) continue;
        found = true;
        EXPECT_EQ(e.kind, mpviz::MapKind::CROSSWALK);
    }
    ASSERT_TRUE(found) << "no polygon (crosswalk) element found in sim fixture output";
}

TEST(HdMapAdapter, CrosswalkTrailingDuplicateVertexIsDeduped)
{
    // Review finding (VM-036 Task 1 Step 2): the real crosswalk_8043 marker
    // (committed fixture) arrives with 5 points, point[0] == point[4] (a
    // closed-polygon closing vertex). The adapter's dedupe must drop that
    // trailing duplicate so the STORED element has 4 points, not 5 --
    // build_crosswalk_hatch()'s n==4 guard never fires on real data
    // otherwise. Reverting the dedupe in hd_map.cpp must fail this test.
    auto msg = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(mpviz_node::testing::urban_row("/hd_map_local_elements"), kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);
    SceneAssembly out;
    a.fill(out);

    // crosswalk_8043's marker id is 8043 (crosswalk isn't lane-paired, so
    // lane_id doesn't identify it); find it by its recorded first vertex
    // instead of depending on storage iteration order.
    bool found = false;
    for (const auto& e : out.map_elements)
    {
        if (e.kind != mpviz::MapKind::CROSSWALK || e.point_count == 0) continue;
        if (std::abs(e.points[0].x - (-39.50850289011474)) < 1e-6 &&
            std::abs(e.points[0].y - 45.33743457749722) < 1e-6)
        {
            found = true;
            EXPECT_EQ(e.point_count, 4u);
            // Review finding (VM-036 Task 1 Step 3, decision #4):
            // crosswalks are not lane-paired -- KindCarriesLaneId()
            // (hd_map.cpp) excludes CROSSWALK, so this element's lane_id
            // must stay 0 regardless of the marker's own (irrelevant) id.
            EXPECT_EQ(e.lane_id, 0u);
        }
    }
    ASSERT_TRUE(found) << "crosswalk_8043 element not found in fill() output";
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
    // malformed even though they carry no points[] on the wire. PLUS
    // (user directive 2026-09-08): centerline_ itself is now `render: drop`
    // too (hidden by default) -- its 16 markers land in dropped_by_rule
    // alongside the arrows. 64 + 16 = 80.
    EXPECT_EQ(a.stats().dropped_by_rule, 80u);
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
    // marker in the same message still comes through. RENAMED (user
    // directive 2026-09-08) from centerline_* to left_boundary_*: urban's
    // centerline_ rule is now `render: drop` by default, so a centerline_
    // marker here would be dropped BY RULE before ever reaching this test's
    // malformed-detection path -- left_boundary_ stays `render: polyline`
    // (see the fixture's own header comment).
    auto msg = mpviz_node::testing::load_marker_array("hd_map_malformed_0.yaml");
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_malformed, 3u);
    EXPECT_EQ(a.stats().dropped_by_rule, 0u);

    SceneAssembly out;
    a.fill(out);
    // Epic 3 Task 1 (VM-036, decision #3): dashing moved renderer-side, so
    // the adapter never chops -- left_boundary_ok stays ONE 2-point
    // element.
    ASSERT_EQ(out.map_elements.size(), 1u);
    for (const auto& e : out.map_elements) EXPECT_EQ(e.point_count, 2u);
}

TEST(HdMapAdapter, FillStampsLastUpdateSecOnEveryElementIncludingRoadSurface)
{
    // Review finding (VM-034 blocking): fill() used to leave
    // MapElement::last_update_sec at its zero-init default on every emitted
    // element -- with the library's fade now live (staleness_alpha checks
    // sim_time_sec against this field), that made the whole HD-map layer
    // render at alpha 0 shortly after node start. Ingest at a known sim
    // time, fill(), and assert EVERY element (including the synthesized
    // ROAD_SURFACE one) carries that time offset into the row's own
    // timeout_sec (fade-pulse policy round 2, 2026-09-08 -- see
    // kMapFadeWindowSec in hd_map.cpp), not a bare ingest-time stamp.
    auto msg = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);
    constexpr double kSimTime = 42.0;
    a.ingest(msg, kSimTime);
    SceneAssembly out;
    a.fill(out);

    // kMapFadeWindowSec = 1.0 (hd_map.cpp) -- mirrored here, not included,
    // since it's a private implementation constant.
    constexpr double kMapFadeWindowSec = 1.0;
    const double expected_stamp = kSimTime + (row.timeout_sec - kMapFadeWindowSec);
    ASSERT_GT(out.map_elements.size(), 0u);
    bool saw_road_surface = false;
    for (const auto& e : out.map_elements)
    {
        EXPECT_DOUBLE_EQ(e.last_update_sec, expected_stamp)
            << "element kind=" << static_cast<int>(e.kind) << " lane_id=" << e.lane_id
            << " was not stamped from the ingest sim time offset by (timeout_sec - "
               "kMapFadeWindowSec)";
        if (e.kind == mpviz::MapKind::ROAD_SURFACE) saw_road_surface = true;
    }
    EXPECT_TRUE(saw_road_surface) << "fixture no longer synthesizes a ROAD_SURFACE element";
}

TEST(HdMapAdapter, ThrottledRowStaysFreshOnReceiptNotOnAcceptedRebuildCadence)
{
    // Review finding (VM-034 blocking, fade-pulse policy): a row throttled
    // by max_rate_hz (e.g. /hd_map_global_elements: 0.5 -> 2.0s min rebuild
    // gap) keeps RECEIVING at its real publish rate between accepted
    // rebuilds -- stamping last_update_sec from the throttled rebuild time
    // (stats_.last_msg_sec) would blink the whole layer dark for most of
    // every 2s window even while the topic is genuinely alive. Decided
    // fix: stamp from last_recv_sec_, set on every TF-lookup-succeeded
    // ingest() call, BEFORE the rate gate, offset into the row's own
    // timeout_sec (fade-pulse policy round 2, 2026-09-08 -- see
    // kMapFadeWindowSec in hd_map.cpp). Pin it at a 0.5 Hz row: ingest
    // every 0.3s (far faster than the 2.0s cooldown, so every rebuild past
    // the first is skipped) and assert fill() at t=1.5s reports
    // 1.5 + (timeout_sec - kMapFadeWindowSec), not the one accepted
    // rebuild's own time (0.0) offset the same way.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_global_elements");
    ASSERT_DOUBLE_EQ(row.max_rate_hz, 0.5) << "urban_profile.yaml's row no longer matches this test's premise";
    mpviz_node::HdMapAdapter a(row, kTf.tf);

    // Same one-marker shape as RateLimitHonoursMaxRateHz above, built
    // inline (its own helper isn't declared until further down this file).
    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "left_boundary_x";
    m.id = 1;
    m.type = 4;  // LINE_STRIP
    m.action = 0;
    geometry_msgs::msg::Point p0, p1;
    p1.x = 10.0;
    m.points = {p0, p1};
    msg.markers = {m};

    for (double t = 0.0; t <= 1.5 + 1e-9; t += 0.3)
    {
        a.ingest(msg, t);
    }

    SceneAssembly out;
    a.fill(out);
    // kMapFadeWindowSec = 1.0 (hd_map.cpp) -- mirrored here, not included,
    // since it's a private implementation constant.
    constexpr double kMapFadeWindowSec = 1.0;
    const double expected_stamp = 1.5 + (row.timeout_sec - kMapFadeWindowSec);
    ASSERT_GT(out.map_elements.size(), 0u);
    for (const auto& e : out.map_elements)
    {
        EXPECT_NEAR(e.last_update_sec, expected_stamp, 1e-9)
            << "stamp tracked the throttled rebuild cadence instead of topic liveness -- "
               "the map layer would blink dark between rebuilds";
    }
}

TEST(HdMapAdapter, LowRateReceiptDoesNotSawtoothBetweenReceipts)
{
    // Review finding (VM-034 blocking, fade-pulse policy round 2,
    // 2026-09-08): ThrottledRowStaysFreshOnReceiptNotOnAcceptedRebuildCadence
    // (above) only ever ingests at 0.3s spacing (3.33 Hz) -- it never pins
    // behaviour in the 1-2 Hz regime the original review criterion named,
    // and a straight last_update_sec == last_recv_sec_ stamp DOES sawtooth
    // there: staleness_alpha starts fading at age > kStaleFadeStartSec =
    // 0.5s (renderer_internal.hpp), so a 1 Hz row (1.0s between receipts)
    // would ride alpha all the way down to 0.0 just before every next
    // receipt. Pin the ACTUAL post-fix behaviour instead: ingest once per
    // second (t = 0, 1, 2, 3, 4) and, just before each next receipt (i.e.
    // fill() called at t=0.999, 1.999, ...), assert the stamped
    // last_update_sec still keeps the element's implied age comfortably
    // under kStaleFadeStartSec -- i.e. it has NOT started fading, unlike
    // the pre-fix straight stamp.
    TfFixture kTf;
    auto row = mpviz_node::testing::urban_row("/hd_map_local_elements");
    mpviz_node::HdMapAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "left_boundary_x";
    m.id = 1;
    m.type = 4;  // LINE_STRIP
    m.action = 0;
    geometry_msgs::msg::Point p0, p1;
    p1.x = 10.0;
    m.points = {p0, p1};
    msg.markers = {m};

    constexpr double kStaleFadeStartSec = 0.5;  // mirrors renderer_internal.hpp, not included
    constexpr double kMapFadeWindowSec = 1.0;   // mirrors hd_map.cpp's own constant, not included
    constexpr double kReceiptPeriodSec = 1.0;    // the 1 Hz regime under test
    for (int receipt = 0; receipt < 5; ++receipt)
    {
        const double t_recv = receipt * kReceiptPeriodSec;
        a.ingest(msg, t_recv);

        SceneAssembly out;
        a.fill(out);
        ASSERT_GT(out.map_elements.size(), 0u);

        // "Just before the next receipt": age is measured against the row's
        // real publish cadence, not last_recv_sec_'s own stamped value --
        // this is what a bare straight-stamp implementation would fail.
        const double now_just_before_next_receipt = t_recv + kReceiptPeriodSec - 1e-3;
        const double expected_stamp = t_recv + (row.timeout_sec - kMapFadeWindowSec);
        for (const auto& e : out.map_elements)
        {
            EXPECT_DOUBLE_EQ(e.last_update_sec, expected_stamp)
                << "receipt #" << receipt << ": stamp is not the offset-into-timeout_sec value";
            const double age = now_just_before_next_receipt - e.last_update_sec;
            EXPECT_LT(age, kStaleFadeStartSec)
                << "receipt #" << receipt << ": a 1 Hz-received row has already started "
                   "fading (age >= kStaleFadeStartSec) just before its next receipt -- "
                   "the fix does not hold in the 1-2 Hz regime";
        }
    }
}

TEST(HdMapAdapter, PublishOnceTransientLocalRowStaysOpaqueWellPastOldOneSecondFadeFloor)
{
    // Review finding (VM-034 blocking, fade-pulse policy round 2,
    // 2026-09-08): sim_profile.yaml's /sim/hd_map/markers is publish-once/
    // transient_local (timeout_sec: 5.0) -- last_recv_sec_ freezes at its
    // one ingest while SceneGraph::sim_time_sec keeps climbing. A straight
    // last_update_sec == last_recv_sec_ stamp faded this row to alpha 0 by
    // +1.0s even though visualization_node.cpp keeps calling fill() for it
    // until +5.0s -- "map never appears" for 4 of its 5 visible seconds.
    // Ingest once at t=0.0, advance the sim clock to t=2.0 (past the OLD
    // 1.0s fade floor, nowhere near the row's real timeout_sec=5.0 cutoff),
    // and assert the row is still FULLY opaque (age comfortably under
    // kStaleFadeStartSec), not faded.
    TfFixture kTf;
    auto row = mpviz_node::testing::sim_row("/sim/hd_map/markers");
    ASSERT_DOUBLE_EQ(row.timeout_sec, 5.0) << "sim_profile.yaml's row no longer matches this test's premise";
    mpviz_node::HdMapAdapter a(row, kTf.tf);

    auto msg = mpviz_node::testing::load_marker_array("sim_hd_map_markers_0.yaml");
    a.ingest(msg, /*sim_time_sec=*/0.0);  // the ONE latched receipt this row ever gets

    SceneAssembly out;
    a.fill(out);
    ASSERT_GT(out.map_elements.size(), 0u);

    constexpr double kStaleFadeStartSec = 0.5;  // mirrors renderer_internal.hpp, not included
    constexpr double kSimTimeNow = 2.0;          // well past the old 1.0s fade floor
    for (const auto& e : out.map_elements)
    {
        const double age = kSimTimeNow - e.last_update_sec;
        EXPECT_LT(age, kStaleFadeStartSec)
            << "the publish-once/transient_local row faded before its own timeout_sec cutoff -- "
               "this is exactly the 'map never appears' regression the fix must close";
    }
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

    // RENAMED (user directive 2026-09-08) from centerline_* to
    // left_boundary_*: urban's centerline_ rule is now `render: drop` by
    // default, so a centerline_ marker here would never reach storage_ at
    // all -- left_boundary_ stays `render: polyline` and this test's own
    // point (rate-limiting REBUILDS) doesn't care which polyline-rendering
    // kind matched.
    a.ingest(make_one_marker("left_boundary_a", 1, 0.0), 1.0);   // accepted (first ever)
    a.ingest(make_one_marker("left_boundary_b", 2, 10.0), 1.1);  // gap 0.1s < 0.5s -> skipped
    a.ingest(make_one_marker("left_boundary_c", 3, 20.0), 1.2);  // gap 0.2s < 0.5s -> skipped
    a.ingest(make_one_marker("left_boundary_d", 4, 30.0), 1.6);  // gap 0.6s >= 0.5s -> accepted

    SceneAssembly out;
    a.fill(out);
    // Only the 2 accepted messages' markers made it in -- with no
    // rate-limit honoured all 4 (no DELETEALL between them) would
    // accumulate. Epic 3 Task 1 (VM-036, decision #3): dashing moved
    // renderer-side, so the adapter never chops -- 2 accepted markers ->
    // 2 elements.
    EXPECT_EQ(out.map_elements.size(), 2u);
}

namespace
{

// Builds a one-row profile with centerline_/left_boundary_/right_boundary_
// namespace rules, for the pose-composition/flatten-z tests below --
// independent of the shipped config so they exercise the geometry op in
// isolation from urban_profile.yaml. Epic 3 Task 1 (VM-036, decision #3):
// dashing moved renderer-side, so this no longer takes the retired
// per-namespace chop parameter -- the adapter never chops any namespace
// now, for any kind. right_boundary_ added (user directive 2026-09-08) for
// the road-edge-detection tests below, which need both rails.
mpviz_node::ProfileRow MapRuleRow()
{
    std::vector<std::string> errs;
    const std::string yaml =
        "name: t\nrows:\n  - {topic: /hd_map, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, ns_default: drop, namespaces:"
        " [{prefix: centerline_,     render: polyline, kind: centerline},"
        "  {prefix: left_boundary_,  render: polyline, kind: left_boundary},"
        "  {prefix: right_boundary_, render: polyline, kind: right_boundary}]}\n";
    auto p = mpviz_node::load_profile_string(yaml, errs);
    if (!p) throw std::runtime_error("MapRuleRow: profile failed to parse");
    return p->rows[0];
}

// Straight LINE_STRIP marker at a given X offset, running the full Y span
// -- three of these side by side (X=0, X=3.2, X=6.4) model three adjacent
// lanes sharing painted lines: lane A's right rail == lane B's left rail,
// lane B's right rail == lane C's left rail (road-edge-detection tests
// below, user directive 2026-09-08).
visualization_msgs::msg::Marker RailMarker(const char* ns, int32_t id, double x)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = id;
    m.type = 4;
    m.action = 0;
    geometry_msgs::msg::Point p0, p1;
    p0.x = x;
    p0.y = 0.0;
    p1.x = x;
    p1.y = 20.0;
    m.points = {p0, p1};
    return m;
}

// Same shape as MapRuleRow() but adds a `junction` namespace rule (kind:
// junction) for the junction-cleanup tests below (user directive
// 2026-09-08) -- optionally with `junction_interior_boundaries: false` set
// at row level (default true, matching every shipped profile).
mpviz_node::ProfileRow JunctionRuleRow(bool junction_interior_boundaries = true)
{
    std::vector<std::string> errs;
    std::string yaml =
        "name: t\nrows:\n  - {topic: /hd_map, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, ns_default: drop, ";
    if (!junction_interior_boundaries) yaml += "junction_interior_boundaries: false, ";
    yaml +=
        "namespaces:"
        " [{prefix: left_boundary_,  render: polyline, kind: left_boundary},"
        "  {prefix: right_boundary_, render: polyline, kind: right_boundary},"
        "  {prefix: junction,        render: polyline, kind: junction}]}\n";
    auto p = mpviz_node::load_profile_string(yaml, errs);
    if (!p) throw std::runtime_error("JunctionRuleRow: profile failed to parse: " +
                                      (errs.empty() ? "" : errs[0]));
    return p->rows[0];
}

// A LINE_STRIP marker with explicit (x,y) points (z=0), full control over
// point spacing -- the junction-cleanup tests below need vertices that
// straddle a polygon/crossing boundary without landing exactly on it.
visualization_msgs::msg::Marker LineMarker(const char* ns, int32_t id,
                                            const std::vector<std::pair<double, double>>& xy)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = id;
    m.type = 4;
    m.action = 0;
    for (const auto& [x, y] : xy)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        m.points.push_back(p);
    }
    return m;
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

TEST(HdMapAdapter, MarkerOfAnyKindStaysOneElementOutOfTheAdapter)
{
    // Epic 3 Task 1 (VM-036, decision #3): dashing moved renderer-side, so
    // the adapter never chops, for any kind -- a straight 10 m marker
    // stays ONE element under a boundary namespace AND under a centerline
    // namespace. Successor to the pre-Epic3
    // DashedCenterlineChopsByArcLengthWithInterpolatedEndpoints /
    // DashChopHappensAfterPoseComposition tests, whose premise (an
    // adapter-side chop) no longer exists.
    TfFixture kTf;

    mpviz_node::HdMapAdapter boundary(MapRuleRow(), kTf.tf);
    boundary.ingest(StraightTenMeterMarker("left_boundary_x"), 1.0);
    SceneAssembly boundary_out;
    boundary.fill(boundary_out);
    ASSERT_EQ(boundary_out.map_elements.size(), 1u);
    EXPECT_EQ(boundary_out.map_elements[0].point_count, 2u);
    // Road-edge detection (user directive 2026-09-08): this marker has no
    // paired lane on the opposite side at all (it is the only lane in the
    // scene), so IsRoadEdge() promotes it -- the emitted kind is ROAD_EDGE,
    // not the raw LEFT_BOUNDARY the rule matched. This test's own point
    // (one marker -> one element, never chopped) is unaffected.
    EXPECT_EQ(boundary_out.map_elements[0].kind, mpviz::MapKind::ROAD_EDGE);

    mpviz_node::HdMapAdapter centerline(MapRuleRow(), kTf.tf);
    centerline.ingest(StraightTenMeterMarker("centerline_x"), 1.0);
    SceneAssembly centerline_out;
    centerline.fill(centerline_out);
    ASSERT_EQ(centerline_out.map_elements.size(), 1u);
    EXPECT_EQ(centerline_out.map_elements[0].point_count, 2u);
    EXPECT_EQ(centerline_out.map_elements[0].kind, mpviz::MapKind::CENTERLINE);
}

TEST(HdMapAdapter, CenterlineAndBoundaryKindAndLaneIdFromMarkerId)
{
    // Review finding (VM-036 Task 1 Step 3 / decision #4): shared decision
    // #4's core claim -- "the marker's own `id` field *is* `lane_id`
    // directly, no ns-suffix parsing" -- was asserted by NO test in either
    // suite before this one (test_scene_buffer.cpp:261's lane_id round-trip
    // only exercises SceneBuffer::assign(), never the adapter's own
    // extraction in hd_map.cpp). A centerline_934/id=934 marker and a
    // left_boundary_934/id=934 marker, same lane_id carried purely via the
    // marker's own `id`, not the ns string.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    auto make_marker = [](const char* ns, int32_t id) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.ns = ns;
        m.id = id;
        m.type = 4;
        m.action = 0;
        geometry_msgs::msg::Point p0;
        geometry_msgs::msg::Point p1;
        p1.x = 10.0;
        m.points = {p0, p1};
        return m;
    };
    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {make_marker("centerline_934", 934), make_marker("left_boundary_934", 934)};
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 2u);

    bool found_centerline = false, found_boundary = false;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::CENTERLINE)
        {
            found_centerline = true;
            EXPECT_EQ(e.lane_id, 934u);
        }
        // Road-edge detection (user directive 2026-09-08): this marker has
        // no opposite-side partner (no right_boundary_934 in this test's
        // scene), so IsRoadEdge() promotes it to ROAD_EDGE -- this test's
        // own point (lane_id carried via the marker's own `id`) doesn't
        // care which of the two kinds it ends up as.
        else if (e.kind == mpviz::MapKind::ROAD_EDGE)
        {
            found_boundary = true;
            EXPECT_EQ(e.lane_id, 934u);
        }
    }
    EXPECT_TRUE(found_centerline);
    EXPECT_TRUE(found_boundary);
}

TEST(HdMapAdapter, LaneWithOnlyOneBoundaryProducesNoRoadSurfaceElement)
{
    // Review finding (VM-036 Task 1 Step 5 / "Named fixture gaps" #2): the
    // committed hd_map_local_elements_0.yaml has all 16 boundary-bearing
    // lanes fully paired, so fill()'s `if (it == right_by_lane.end())
    // continue;` branch has no real-fixture instance and is otherwise
    // unexecuted by any test. Hand-built: only a left_boundary_ marker, no
    // right_boundary_ counterpart for the same lane_id -> zero ROAD_SURFACE
    // elements, and this is "missing data," not "bad data" (spec §9), so no
    // dropped_malformed bump either.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);
    a.ingest(StraightTenMeterMarker("left_boundary_x"), 1.0);  // marker id 1 -> lane_id 1

    SceneAssembly out;
    a.fill(out);
    const auto road_count =
        std::count_if(out.map_elements.begin(), out.map_elements.end(),
                       [](const mpviz::MapElement& m) { return m.kind == mpviz::MapKind::ROAD_SURFACE; });
    EXPECT_EQ(road_count, 0);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HdMapAdapter, MismatchedRailPointCountsResampledStationsSpanRecordedEndpoints)
{
    // Named gap, Epic 3 Task 1 Step 5's own SECOND CORRECTION (epic3 plan):
    // LocalElementsFixtureYieldsLanesAndCrosswalks's aggregate (road_count
    // ==16, every point_count==32) proves resampling produced the right
    // SHAPE but never proved the resampled stations actually SPAN the
    // recorded rail's own endpoints -- a resampler that silently clamped to
    // a sub-range of the rail (e.g. an off-by-one in ResampleByArcLength's
    // arc-length walk) would pass that aggregate too. Lane 955 (left 8 /
    // right 9 recorded points, the real mismatched-count case, verified
    // against the committed fixture) is the one this gap names directly.
    auto msg = mpviz_node::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(mpviz_node::testing::urban_row("/hd_map_local_elements"), kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);
    SceneAssembly out;
    a.fill(out);

    // Recorded endpoints straight from the fixture message -- frame "map",
    // identity pose (verified: pose.position=(0,0,0), pose.orientation=
    // identity for both markers), so these are exactly what reaches
    // storage_ too; comparing against the raw message rather than another
    // emitted MapElement sidesteps the road-edge promotion's kind ambiguity
    // (a lane-955 boundary may or may not have promoted to ROAD_EDGE; the
    // raw marker ns is unambiguous either way).
    const geometry_msgs::msg::Point* left_first = nullptr;
    const geometry_msgs::msg::Point* left_last = nullptr;
    const geometry_msgs::msg::Point* right_first = nullptr;
    const geometry_msgs::msg::Point* right_last = nullptr;
    for (const auto& m : msg.markers)
    {
        if (m.ns == "left_boundary_955" && !m.points.empty())
        {
            left_first = &m.points.front();
            left_last = &m.points.back();
        }
        else if (m.ns == "right_boundary_955" && !m.points.empty())
        {
            right_first = &m.points.front();
            right_last = &m.points.back();
        }
    }
    ASSERT_NE(left_first, nullptr) << "fixture no longer carries left_boundary_955";
    ASSERT_NE(right_first, nullptr) << "fixture no longer carries right_boundary_955";

    const mpviz::MapElement* road = nullptr;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_SURFACE && e.lane_id == 955u)
        {
            road = &e;
            break;
        }
    }
    ASSERT_NE(road, nullptr) << "lane 955 produced no ROAD_SURFACE element";
    ASSERT_EQ(road->point_count, 32u);

    // points[0..15] = left rail, points[16..31] = right rail, index-parallel
    // by normalized station (map_elements.hpp's own two-rail encoding note).
    constexpr double kEps = 1e-6;
    EXPECT_NEAR(road->points[0].x, left_first->x, kEps);
    EXPECT_NEAR(road->points[0].y, left_first->y, kEps);
    EXPECT_NEAR(road->points[15].x, left_last->x, kEps);
    EXPECT_NEAR(road->points[15].y, left_last->y, kEps);
    EXPECT_NEAR(road->points[16].x, right_first->x, kEps);
    EXPECT_NEAR(road->points[16].y, right_first->y, kEps);
    EXPECT_NEAR(road->points[31].x, right_last->x, kEps);
    EXPECT_NEAR(road->points[31].y, right_last->y, kEps);
}

// ── Road-edge detection (user directive 2026-09-08): "the boundary of the
//    road (most left and most right lines) should not be dashed and should
//    be colored differently" ────────────────────────────────────────────

TEST(HdMapAdapter, OuterBoundariesOfThreeAdjacentLanesPromoteToRoadEdge)
{
    // Three adjacent lanes sharing painted lines (A|B|C, each 3.2 m wide --
    // RailMarker's own comment): A's right rail == B's left rail (x=3.2),
    // B's right rail == C's left rail (x=6.4). Only the outermost two rails
    // (A's left, x=0; C's right, x=9.6) have no coincident opposite-side
    // twin from another lane -- everything between stays a shared interior
    // divider.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        RailMarker("left_boundary_a", 1, 0.0),
        RailMarker("right_boundary_a", 1, 3.2),
        RailMarker("left_boundary_b", 2, 3.2),
        RailMarker("right_boundary_b", 2, 6.4),
        RailMarker("left_boundary_c", 3, 6.4),
        RailMarker("right_boundary_c", 3, 9.6),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    // Direct scan by recorded X (simpler and unambiguous than re-deriving
    // "which side" from kind alone, since kind is exactly what's under test).
    auto kind_at_x = [&](double x) -> mpviz::MapKind {
        for (const auto& e : out.map_elements)
        {
            if (e.kind == mpviz::MapKind::ROAD_SURFACE) continue;
            if (std::abs(e.points[0].x - x) < 1e-9) return e.kind;
        }
        ADD_FAILURE() << "no element found at x=" << x;
        return mpviz::MapKind::OTHER;
    };
    EXPECT_EQ(kind_at_x(0.0), mpviz::MapKind::ROAD_EDGE) << "lane A's left rail: outer edge";
    EXPECT_EQ(kind_at_x(3.2), mpviz::MapKind::LEFT_BOUNDARY)
        << "lane B's left rail == lane A's right rail: shared interior divider";
    EXPECT_EQ(kind_at_x(6.4), mpviz::MapKind::LEFT_BOUNDARY)
        << "lane C's left rail == lane B's right rail: shared interior divider";
    EXPECT_EQ(kind_at_x(9.6), mpviz::MapKind::ROAD_EDGE) << "lane C's right rail: outer edge";

    const auto road_edge_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const mpviz::MapElement& m) { return m.kind == mpviz::MapKind::ROAD_EDGE; });
    EXPECT_EQ(road_edge_count, 2);
}

TEST(HdMapAdapter, SingleIsolatedLaneHasBothBoundariesPromotedToRoadEdge)
{
    // A lane with no neighbour on either side: BOTH its boundaries are road
    // edges (there is nothing to be interior to).
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {RailMarker("left_boundary_solo", 1, 0.0),
                    RailMarker("right_boundary_solo", 1, 3.2)};
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    int road_edge_count = 0;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_SURFACE) continue;
        EXPECT_EQ(e.kind, mpviz::MapKind::ROAD_EDGE);
        ++road_edge_count;
    }
    EXPECT_EQ(road_edge_count, 2);
}

// ---- Junction cleanup (user directive 2026-09-08 + same-day refinement) --

TEST(HdMapAdapter, JunctionPolygonClipSplitsRoadEdgeIntoTwoSubElementsWithInterpolatedCuts)
{
    // A single isolated (hence ROAD_EDGE-promoted) rail running straight
    // through a 6x6 m JUNCTION box centered on the origin: the directive's
    // own "cut them off in these areas and continue along the road after
    // the junction."
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(JunctionRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_edge", 1, {{0, -10}, {0, -5}, {0, 0}, {0, 5}, {0, 10}}),
        LineMarker("junction", 900, {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}, {-3, -3}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> edges;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE) edges.push_back(e);
    }
    ASSERT_EQ(edges.size(), 2u) << "one edge crossing the junction box splits into two";
    std::sort(edges.begin(), edges.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].y < b.points[0].y;
              });

    // First sub-element: y=-10 up to the box's entry edge (y=-3) -- the
    // last point is INTERPOLATED (the box boundary), not one of the
    // original recorded vertices (-5, 0, 5, 10).
    EXPECT_DOUBLE_EQ(edges[0].points[0].y, -10.0);
    EXPECT_NEAR(edges[0].points[edges[0].point_count - 1].y, -3.0, 1e-6);
    EXPECT_NE(edges[0].points[edges[0].point_count - 1].y, -5.0);

    // Second sub-element: the box's exit edge (y=3) resuming out to y=10.
    EXPECT_NEAR(edges[1].points[0].y, 3.0, 1e-6);
    EXPECT_DOUBLE_EQ(edges[1].points[edges[1].point_count - 1].y, 10.0);
    EXPECT_NE(edges[1].points[0].y, 5.0);
}

TEST(HdMapAdapter, MutualCrossingCutSplitsBothRoadEdgesWithBackoff)
{
    // Two isolated (ROAD_EDGE-promoted) rails from DIFFERENT lane_ids
    // crossing at the origin, no JUNCTION geometry at all -- the mechanism
    // that covers urban's real feed (verified: no `junction` namespace
    // rule anywhere in urban_profile.yaml).
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-10, 0}, {10, 0}}),
        LineMarker("left_boundary_b", 2, {{0, -10}, {0, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a, lane_b;
    for (const auto& e : out.map_elements)
    {
        if (e.kind != mpviz::MapKind::ROAD_EDGE) continue;
        (e.lane_id == 1u ? lane_a : lane_b).push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    ASSERT_EQ(lane_b.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    std::sort(lane_b.begin(), lane_b.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].y < b.points[0].y;
              });

    // kJunctionCutBackoffM = 2.0 m either side of the crossing at the origin.
    EXPECT_EQ(lane_a[0].point_count, 2u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -10.0);
    EXPECT_NEAR(lane_a[0].points[1].x, -2.0, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, 2.0, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[1].points[1].x, 10.0);

    EXPECT_NEAR(lane_b[0].points[1].y, -2.0, 1e-9);
    EXPECT_NEAR(lane_b[1].points[0].y, 2.0, 1e-9);
}

TEST(HdMapAdapter, ParallelRoadEdgesAreNeverCut)
{
    // A real road edge never legitimately crosses another -- two parallel
    // rails must render whole, unsplit, regardless of how close together
    // they run.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{0, -10}, {0, 10}}),
        LineMarker("left_boundary_b", 2, {{5, -10}, {5, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    int road_edge_count = 0;
    for (const auto& e : out.map_elements)
    {
        if (e.kind != mpviz::MapKind::ROAD_EDGE) continue;
        ++road_edge_count;
        EXPECT_EQ(e.point_count, 2u) << "untouched -- no cut introduced";
    }
    EXPECT_EQ(road_edge_count, 2);
}

TEST(HdMapAdapter, AbuttingNearCollinearRoadEdgesAreNeverCut)
{
    // Code-review finding (blocking): a lanelet-chain node where two
    // ROAD_EDGE rails from DIFFERENT lane_ids share an endpoint with only a
    // ~1 deg kink is NOT a junction crossing -- it is the ordinary case of
    // one lanelet boundary handing off to the next, re-derived against
    // hd_map_local_elements_0.yaml (21 of 24 raw SegSegIntersect2D
    // detections on that fixture were exactly this: shared-endpoint
    // abutments at 175.2-179.8 deg or 0.5-2.8 deg, not genuine crossings).
    // Before the fix, SegSegIntersect2D's bare |denom| < 1e-12 guard
    // happily accepted this (t==1, u==0 exactly, denom far above 1e-12)
    // and punched a kJunctionCutBackoffM hole into a continuous rail.
    // ParallelRoadEdgesAreNeverCut above can't pin this: its rails never
    // touch at all, so it never reaches the t/u-at-an-endpoint case this
    // guards.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    const double kink_rad = 1.0 * M_PI / 180.0;
    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-10, 0}, {0, 0}}),
        LineMarker("left_boundary_b", 2, {{0, 0}, {10, 10 * std::tan(kink_rad)}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    int road_edge_count = 0;
    for (const auto& e : out.map_elements)
    {
        if (e.kind != mpviz::MapKind::ROAD_EDGE) continue;
        ++road_edge_count;
        EXPECT_EQ(e.point_count, 2u) << "untouched -- a ~1 deg kink is not a crossing";
    }
    EXPECT_EQ(road_edge_count, 2);
}

// ---- Junction gap-merge (measurement pass 2026-09-08, user report "yellow
// boundaries left overs (check junction corners) that looks messy") -- a
// real multi-lane junction crosses one through-edge SEVERAL times close
// together; each crossing's own kJunctionCutBackoffM=2.0 window needs to
// merge with its neighbours' when the gap between them is small, or the
// small real gap survives as its own tiny "leftover" rendered piece (the
// bug). All three tests below share one long through-rail (lane 1, x=-20
// to 20 along y=0) crossed by two other lanes (2, 3) at two x positions;
// only lane 1's own resulting ROAD_EDGE pieces are asserted on.

TEST(HdMapAdapter, JunctionGapMergeCutsSliverBetweenCloseCrossings)
{
    // Crossings at x=-4.5 and x=4.5 (separation 9.0 m): each gets its own
    // [-2,+2] backoff window, [-6.5,-2.5] and [2.5,6.5] -- a 5.0 m gap
    // between them, UNDER kJunctionGapMergeM=6.6 -- so MergeWindows folds
    // them into one [-6.5,6.5] cut. Before this fix, a 5.0 m gap did not
    // touch/overlap under the old bare-overlap test, so it would have
    // survived as its own emitted sliver between the two crossings --
    // exactly the "leftovers at junction corners" the user reported.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-20, 0}, {20, 0}}),
        LineMarker("left_boundary_b", 2, {{-4.5, -10}, {-4.5, 10}}),
        LineMarker("left_boundary_c", 3, {{4.5, -10}, {4.5, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "the two close-together crossings merge into one cut -- "
                                    "no sliver piece survives between them";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -20.0);
    EXPECT_NEAR(lane_a[0].points[lane_a[0].point_count - 1].x, -6.5, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, 6.5, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[1].points[lane_a[1].point_count - 1].x, 20.0);
}

TEST(HdMapAdapter, JunctionGapMergeAtThresholdStillMerges)
{
    // Crossings at x=-5.3/+5.3 (separation 10.6 m): windows [-7.3,-3.3] and
    // [3.3,7.3] -- the inter-window gap is kJunctionGapMergeM=6.6 m to
    // within floating-point noise (6.599999999999998 as the arc-length walk
    // computes it, ~2 ulp BELOW 6.6 -- review 2026-09-08), so this pins the
    // constant's boundary from just below it: still one merged cut. The
    // merge test is <=, and nothing here depends on the exact-equality case.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-20, 0}, {20, 0}}),
        LineMarker("left_boundary_b", 2, {{-5.3, -10}, {-5.3, 10}}),
        LineMarker("left_boundary_c", 3, {{5.3, -10}, {5.3, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "gap exactly at the threshold still merges (inclusive test)";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    EXPECT_NEAR(lane_a[0].points[lane_a[0].point_count - 1].x, -7.3, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, 7.3, 1e-9);
}

TEST(HdMapAdapter, JunctionGapMergeKeepsLegitInteriorSpanAboveThreshold)
{
    // Crossings at x=-5.5/+5.5 (separation 11.0 m): windows [-7.5,-3.5] and
    // [3.5,7.5] -- a 7.0 m gap, ABOVE kJunctionGapMergeM=6.6, so the two
    // windows stay separate and the 7.0 m interior span between them
    // survives as its own legit piece (this is the "just above threshold
    // survives" case -- a real stub is never shorter than 7.027 m per the
    // measurement pass, so a mechanism that ate anything above 6.0 m would
    // risk eating real road; this pins that it does not).
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-20, 0}, {20, 0}}),
        LineMarker("left_boundary_b", 2, {{-5.5, -10}, {-5.5, 10}}),
        LineMarker("left_boundary_c", 3, {{5.5, -10}, {5.5, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 3u) << "head, kept interior span, tail -- the gap is too wide to merge";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -20.0);
    EXPECT_NEAR(lane_a[0].points[lane_a[0].point_count - 1].x, -7.5, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, -3.5, 1e-9);
    EXPECT_NEAR(lane_a[1].points[lane_a[1].point_count - 1].x, 3.5, 1e-9);
    EXPECT_NEAR(lane_a[2].points[0].x, 7.5, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[2].points[lane_a[2].point_count - 1].x, 20.0);
}

TEST(HdMapAdapter, BoundariesUntouchedByJunctionCutsAtDefaultFlag)
{
    // junction_interior_boundaries defaults to true: an INTERIOR boundary
    // -- lane 2's left rail coincides with lane 1's right rail (the shared
    // painted line between two adjacent lanes, same coincidence check
    // OuterBoundariesOfThreeAdjacentLanesPromoteToRoadEdge exercises), so
    // neither promotes to ROAD_EDGE -- running straight through a JUNCTION
    // box renders whole, unclipped: the refinement's own "enable them by
    // default".
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(JunctionRuleRow(/*junction_interior_boundaries=*/true), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_b", 2, {{0, -10}, {0, -5}, {0, 0}, {0, 5}, {0, 10}}),
        LineMarker("right_boundary_a", 1, {{0, -10}, {0, 10}}),
        LineMarker("junction", 900, {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}, {-3, -3}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> left;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::LEFT_BOUNDARY) left.push_back(e);
    }
    ASSERT_EQ(left.size(), 1u) << "not split -- the junction polygon clip never ran on it";
    ASSERT_EQ(left[0].point_count, 5u) << "every original point survives, untouched";
    EXPECT_DOUBLE_EQ(left[0].points[0].y, -10.0);
    EXPECT_DOUBLE_EQ(left[0].points[2].y, 0.0);
    EXPECT_DOUBLE_EQ(left[0].points[4].y, 10.0);
}

TEST(HdMapAdapter, JunctionInteriorBoundariesFalseDropsSegmentsInsideJunctionPolygon)
{
    // Same interior (coincident, non-promoted) boundary + junction box as
    // above, but with junction_interior_boundaries: false -- the boundary
    // now gets the SAME polygon clip ROAD_EDGE always gets (never the
    // crossing-cut's back-off: the cut point sits exactly on the box edge,
    // y=+/-3, not offset by kJunctionCutBackoffM).
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(JunctionRuleRow(/*junction_interior_boundaries=*/false), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_b", 2, {{0, -10}, {0, -5}, {0, 0}, {0, 5}, {0, 10}}),
        LineMarker("right_boundary_a", 1, {{0, -10}, {0, 10}}),
        LineMarker("junction", 900, {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}, {-3, -3}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> left;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::LEFT_BOUNDARY) left.push_back(e);
    }
    ASSERT_EQ(left.size(), 2u) << "split by the polygon clip, same as ROAD_EDGE would be";
    std::sort(left.begin(), left.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].y < b.points[0].y;
              });
    EXPECT_DOUBLE_EQ(left[0].points[0].y, -10.0);
    EXPECT_NEAR(left[0].points[left[0].point_count - 1].y, -3.0, 1e-6);
    EXPECT_NEAR(left[1].points[0].y, 3.0, 1e-6);
    EXPECT_DOUBLE_EQ(left[1].points[left[1].point_count - 1].y, 10.0);
}

// ---- Arc-aware cut refinement (user directive 2026-09-08, follow-up:
// "leftover still exist, I suggest that as there are arcs on the inner
// coreners of the junction, start cutting of from the poin the arc starts,
// and if you drive through further another adjecent arc joins there we stop
// cutting off"). Every polyline below is built from the measurement pass's
// own numbers (kArcRadiusThresholdM=20.0, kArcMinTotalTurnDeg=15.0,
// kArcSearchMarginM=6.0 -- see hd_map.cpp's own comment for the measured
// margins that chose them) so each test pins a specific, checkable
// arc-length station, not just "some point past the old fixed backoff".

TEST(HdMapAdapter, ArcSnapExtendsCutPastFixedBackoffToTheCornerArcsFarEdge)
{
    // lane 1 crosses lane 2 near the origin; well past the OLD fixed
    // kJunctionCutBackoffM=2.0 m cut point (x=2), lane 1's own recorded
    // curb geometry curves into a real corner arc (radius 8 m, within the
    // measured 2.29-12.44 m corner range) -- the old mechanism would stop
    // mid-curve at (1.99, 0.13). The arc-snap must extend the cut out to a
    // REAL recorded vertex on that curve, never an interpolated mid-curve
    // point, and (code-review finding 2026-09-08, blocking) all the way to
    // the arc's own true far edge (v6, the run v2..v6 all measuring R=8.0 m,
    // 75 deg total turn) even though that lies farther than
    // kArcSearchMarginM=6.0 m from the fixed-backoff boundary -- not just
    // the farthest vertex the OLD search margin alone could reach (v4).
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-30.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834},
                    {9.0, 7.999999999999999},
                    {9.0, 20.0}}),
        LineMarker("left_boundary_b", 2, {{0.0, -10.0}, {0.0, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "one crossing -- head piece and tail piece";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    // Head piece: untouched by the arc (it's on the tail side only) --
    // still the OLD fixed-backoff point (x=-2).
    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -30.0);
    EXPECT_NEAR(lane_a[0].points[1].x, -2.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[1].y, 0.0, 1e-9);

    // Tail piece: the cut resumes at (8.727406610312546, 5.929447639179834)
    // -- the arc's own TRUE far recorded vertex (v6), not an interpolated
    // point. CORRECTED (code-review finding 2026-09-08, blocking): an
    // earlier version of this test pinned (6.65685424949238,
    // 2.3431457505076194) (v4) here -- the farthest vertex the OLD
    // +/-6.0 m search margin could reach from the fixed-backoff boundary,
    // not the arc's own far edge the run actually reaches (v2..v6, all
    // R=8.0 m, 75 deg total turn) -- under a test name asserting exactly
    // that far-edge behaviour. FindArcSpanNear now extends a run past the
    // search margin while curvature keeps clearing kArcRadiusThresholdM, so
    // the cut reaches v6 in full.
    ASSERT_EQ(lane_a[1].point_count, 3u);
    EXPECT_NEAR(lane_a[1].points[0].x, 8.727406610312546, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].y, 5.929447639179834, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].x, 9.0, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].y, 20.0, 1e-9);
}

TEST(HdMapAdapter, ArcSnapDoesNotFireOnTheMeasuredWorstCaseGentleOpenRoadCurve)
{
    // CORRECTED (code-review finding 2026-09-08, blocking): the ORIGINAL
    // version of this test pinned a single-vertex 2.8 deg survey kink as
    // "the measured worst case" -- the wrong population (see
    // kArcMinTotalTurnDeg's corrected comment in hd_map.cpp). This gate
    // gates a maximal contiguous R<kArcRadiusThresholdM RUN's own
    // accumulated |TurnAngleDeg|, not one vertex's own kink -- and
    // TurnAngleDeg sums absolute per-vertex turns, so a multi-vertex
    // near-straight run is the false-positive shape a single-vertex test
    // can never exercise. Re-pointed at the real worst case: three interior
    // vertices, each on a true R=15 m circular arc (well under the 20 m
    // radius gate), each contributing 3.5267 deg, for an accumulated total
    // of 10.58 deg -- the largest REJECTED R<threshold run measured in the
    // bag-wide scan, still comfortably under the 15 deg gate (smallest
    // ACCEPTED run measured: 17.05 deg). Sits well within kArcSearchMarginM
    // of the crossing-cut boundary -- exactly where a false positive would
    // show up -- and must NOT move the cut at all: both boundaries stay at
    // the OLD fixed-backoff points.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-30.0, 0.0},
                    {2.0, 0.0},
                    {2.9231334322826161, 0.0},
                    {3.8445187055856869, 0.056784786150068864},
                    {4.76066612300773, 0.1701392891431307},
                    {5.668105825743191, 0.339634184928405}}),
        LineMarker("left_boundary_b", 2, {{0.0, -10.0}, {0.0, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_NEAR(lane_a[0].points[1].x, -2.0, 1e-9) << "unchanged -- the run never qualifies";
    EXPECT_NEAR(lane_a[0].points[1].y, 0.0, 1e-9);

    ASSERT_EQ(lane_a[1].point_count, 5u) << "the whole near-straight run still renders, unmoved";
    EXPECT_NEAR(lane_a[1].points[0].x, 2.0, 1e-9) << "unchanged -- still the old fixed backoff";
    EXPECT_NEAR(lane_a[1].points[1].x, 2.9231334322826161, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].x, 3.8445187055856869, 1e-9);
    EXPECT_NEAR(lane_a[1].points[3].x, 4.76066612300773, 1e-9);
    EXPECT_NEAR(lane_a[1].points[4].x, 5.668105825743191, 1e-9);
}

TEST(HdMapAdapter, ArcSnapExtendsBothWindowBoundariesToTheirOwnCornerArcs)
{
    // "start cutting off from the point the arc starts, and if you drive
    // through further another adjacent arc joins there we stop cutting
    // off": lane 1 approaches the crossing (near the origin) through an
    // entry-side corner arc AND leaves through an exit-side one (the same
    // corner geometry as the test above, mirrored) -- both directions of
    // travel through this one junction corner. Both window boundaries must
    // snap outward independently.
    // CORRECTED (code-review finding 2026-09-08, blocking): this lane's own
    // entry lead-in (v0->v1, 7.0711 m) is short enough that CircumradiusXY
    // AT v1 itself (R=7.246 m) also clears kArcRadiusThresholdM -- unlike
    // the longer straight lead-ins in the tests above, where the joint
    // vertex reads as high-radius and stops the run there. CircumradiusXY
    // takes the curvature's MAGNITUDE only (std::abs), so the inflection at
    // v7/v8 (where the entry arc's curve direction reverses into the exit
    // arc, still R=15.63 m, comfortably under threshold) never breaks the
    // run either -- by this run's own definition, v1..v13 is ONE continuous
    // qualifying corridor, not two independent corners. Both boundaries
    // therefore snap to that ONE run's own two true ends (v1 and v13), not
    // to two separate nearer corners -- still "each boundary snaps outward
    // independently to its own nearby geometry" (no cross-polyline pairing
    // logic was added), just with a farther reach than this test originally
    // assumed under the old, margin-bounded search.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-14.0, 13.0},
                    {-9.0, 8.0},
                    {-8.727406610312546, 5.929447639179834},
                    {-7.928203230275509, 3.999999999999999},
                    {-6.65685424949238, 2.3431457505076194},
                    {-5.0, 1.0717967697244903},
                    {-3.070552360820166, 0.2725933896874535},
                    {-1.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834},
                    {9.0, 7.999999999999999},
                    {9.0, 20.0}}),
        LineMarker("left_boundary_b", 2, {{0.0, -10.0}, {0.0, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    // Entry side: cut starts (the kept head ends) at v1 (-9.0, 8.0) -- the
    // one continuous run's own true near end, reached even though it lies
    // past the OLD +/-6.0 m search margin from the fixed-backoff boundary.
    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_NEAR(lane_a[0].points[0].x, -14.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[1].x, -9.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[1].y, 8.0, 1e-9);

    // Exit side: cutting resumes at v13 (8.727406610312546, 5.929447639179834)
    // -- the same one continuous run's own true far end.
    ASSERT_EQ(lane_a[1].point_count, 3u);
    EXPECT_NEAR(lane_a[1].points[0].x, 8.727406610312546, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].y, 5.929447639179834, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].x, 9.0, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].y, 20.0, 1e-9);
}

TEST(HdMapAdapter, ArcSnapAdjacentWindowsFromTwoCrossingsProduceOneContinuousCut)
{
    // Same crossing positions as JunctionGapMergeKeepsLegitInteriorSpanAboveThreshold
    // above (gap 7.0 m, ABOVE kJunctionGapMergeM=6.6 -- MergeWindows alone
    // leaves a legit interior span there). lane 1's own recorded geometry
    // now carries a small real corner kink facing EACH crossing from the
    // inside (turn ~25 deg, radius ~3.5 m -- both comfortably inside the
    // measured corner range). This models the user's own "another adjacent
    // arc joins there" case: TWO DIFFERENT crossings, each with its own
    // corner, close enough together that once each window snaps outward to
    // its own arc, they meet -- and the interior that used to survive
    // whole is correctly consumed once each window snaps outward to its own
    // arc. fill() re-runs MergeWindows after SnapWindowsToArcs (see that
    // call site's own comment); this test does not depend on which of the
    // two MergeWindows calls is the one that folds these windows together.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-50.0, 0.0},
                    {-4.0, 0.0},
                    {-2.0, 0.3},
                    {-1.0, 0.0},
                    {1.0, 0.0},
                    {2.0, 0.3},
                    {4.0, 0.0},
                    {50.0, 0.0}}),
        LineMarker("left_boundary_b", 2, {{-5.5, -10.0}, {-5.5, 10.0}}),
        LineMarker("left_boundary_c", 3, {{5.5, -10.0}, {5.5, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u)
        << "the two corners' arcs meet in the middle -- no legit interior survives "
           "here any more, unlike the plain-straight version of this same gap above";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    // The OUTER bounds are exactly where the plain fixed backoff always put
    // them (+/-2.0 m off the +/-5.5 m crossings) -- neither kink is close
    // enough to the OUTER edge to move it; only the (now-vanished) interior
    // moved.
    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -50.0);
    EXPECT_NEAR(lane_a[0].points[1].x, -7.5, 1e-9);
    ASSERT_EQ(lane_a[1].point_count, 2u);
    EXPECT_NEAR(lane_a[1].points[0].x, 7.5, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[1].points[1].x, 50.0);
}

TEST(HdMapAdapter, PureArcCornerConnectorStillFramesItsOwnTwoRecordedEndpoints)
{
    // "No edge in this data is 'mostly arc'" (measurement pass) -- this is
    // the hypothetical the report flags but never observed: a SHORT
    // connector whose entire length between its own two recorded endpoints
    // is one continuous corner arc (radius 6 m, ~45 deg total turn),
    // crossed once. Stated behaviour, by design: the arc-snap can never
    // swallow a polyline's own literal first/last vertex (FindArcSpanNear
    // only tests INTERIOR vertices -- an endpoint has no far neighbour to
    // compute curvature from), so even a 100%-arc connector still emits two
    // short slivers framing its own two recorded ends, not zero pieces.
    // This is the outward-only design's own stated ceiling, not a bug.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{0.0, 0.0},
                    {1.5529142706151244, 0.2044450422655899},
                    {2.9999999999999996, 0.803847577293368},
                    {4.242640687119285, 1.7573593128807143},
                    {5.196152422706632, 2.999999999999999}}),
        LineMarker("left_boundary_b", 2, {{2.2, -10.0}, {2.2, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "two minimal end-slivers, never zero pieces";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    // Head sliver: the connector's own literal FIRST point, plus the
    // (unmoved -- the fixed backoff already reached further in than the
    // arc's own near edge here) cut boundary.
    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_NEAR(lane_a[0].points[0].x, 0.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[0].y, 0.0, 1e-9);

    // Tail sliver: starts exactly at the arc's own far recorded vertex
    // (4.242640687119285, 1.7573593128807143), not mid-curve, and still
    // ends at the connector's own literal LAST point -- never dropped.
    ASSERT_EQ(lane_a[1].point_count, 2u);
    EXPECT_NEAR(lane_a[1].points[0].x, 4.242640687119285, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].y, 1.7573593128807143, 1e-9);
    EXPECT_NEAR(lane_a[1].points[1].x, 5.196152422706632, 1e-9);
    EXPECT_NEAR(lane_a[1].points[1].y, 2.999999999999999, 1e-9);
}

TEST(HdMapAdapter, TerminalArcStubIsRemovedAndArcSnapStaysSafeAtTheClamp)
{
    // A crossing lands close enough to lane 1's own recorded END that the
    // PLAIN fixed kJunctionCutBackoffM=2.0 m window already clamps past
    // its own total length (a terminal cut -- H1 from the original
    // measurement pass, checked and found absent in the real bag, but the
    // mechanism must still behave when it happens): the tail stub must be
    // REMOVED ENTIRELY (zero pieces on that side), not left as a tiny
    // leftover. There IS a real corner arc near that same boundary (the
    // crossing lands inside it) -- this pins that FindArcSpanNear finding
    // one there does NOT resurrect the already-clamped tail (the max()
    // snap in SnapWindowsToArcs is a no-op whenever the arc's own far edge
    // sits BEFORE the already-larger clamped boundary, exactly like here).
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-30.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834},
                    {9.0, 7.999999999999999}}),
        // Crosses the final segment (index6->7) at arc-length 42.5, i.e.
        // (8.865490872405871, 6.978301740877824) -- 1.03 m from lane 1's
        // own recorded end (total length 43.53051445312495), well inside
        // kJunctionCutBackoffM=2.0 m of it.
        LineMarker("left_boundary_b", 2, {{-10.0, 6.978301740877824}, {20.0, 6.978301740877824}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u) << "the tail stub is gone -- only the head piece survives";

    // Head piece: the arc-snap DOES fire here (the fixed backoff at 40.5 m
    // landed mid-curve too) and extends it out to a real recorded vertex,
    // exactly as ArcSnapExtendsCutPastFixedBackoffToTheCornerArcsFarEdge
    // pins on the equivalent entry-side geometry. CORRECTED (code-review
    // finding 2026-09-08, blocking): the extension now reaches all the way
    // back to v2, the run's own true near end (v2..v6, all R=8.0 m) -- an
    // earlier version of this test pinned v3, the farthest vertex the OLD
    // +/-6.0 m search margin alone could reach from this boundary.
    ASSERT_EQ(lane_a[0].point_count, 3u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -30.0);
    EXPECT_NEAR(lane_a[0].points[2].x, 3.070552360820166, 1e-9);
    EXPECT_NEAR(lane_a[0].points[2].y, 0.2725933896874535, 1e-9);
}

TEST(HdMapAdapter, ArcSnapReachesTheArcsTrueStartEvenBeyondTheOldSearchMargin)
{
    // CORRECTED (code-review finding 2026-09-08, blocking): this test used
    // to be named ArcSnapKnownCeilingStopsWithinSearchMarginNotAtTheArcsTrue
    // Start and pinned a DELIBERATE, ACCEPTED ceiling -- when a qualifying
    // arc run's own true start lay more than kArcSearchMarginM=6.0 m before
    // the fixed-backoff boundary, the snap used to reach only the farthest
    // vertex the search margin could see (vertex 6), not the run's own true
    // first vertex (vertex 4), a 5.065 m shortfall. That ceiling is now
    // removed: FindArcSpanNear extends a run outward past the search margin
    // while curvature keeps clearing kArcRadiusThresholdM (see that
    // function's own comment), so the snap reaches vertex 4 in full. This is
    // a real, committed instance, not a hypothetical: lane_a's 10 points
    // below are LocalElementsFixtureYieldsLanesAndCrosswalks' own real
    // recorded lane 792 vertices (hd_map_local_elements_0.yaml), and lane_b
    // crosses at that same fixture's own one genuine crossing point,
    // (-40.36,-21.90) -- reproduced standalone here so this specific case is
    // pinned by a dedicated, self-contained test, not just implicitly by the
    // fixture test's own aggregate assertions.
    //
    // Lane 792's real contiguous R<kArcRadiusThresholdM run is vertices
    // 4..7 (station 17.114 to 24.822, 73.96 deg total turn) -- by
    // FindArcSpanNear's own definition of an arc, THIS is where the fillet
    // actually starts, and it is now where the snap actually lands.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 792,
                   {{-46.980491978360725, 2.12625044165753},
                    {-46.771308183612035, -6.249399325630099},
                    {-46.78021262964802, -9.742296952272927},
                    {-46.547218708994116, -12.837935483912194},
                    {-46.4658864625989, -14.974389719737527},   // vertex 4, station 17.114 --
                                                                 // the arc's OWN true start
                    {-46.0651262769608, -17.148459112710047},   // vertex 5, station 19.324
                    {-44.95378149894339, -19.77721103359962},   // vertex 6, station 22.178 --
                                                                 // the search margin's own reach
                    {-42.79181672520813, -21.299520261110413},  // vertex 7, station 24.822
                    {-38.17917117284436, -22.437475517352055},
                    {-30.297919317257183, -23.46468621338738}}),
        LineMarker("left_boundary_b", 685, {{-40.35957131078806, -50.0}, {-40.35957131078806, 50.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 792u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const mpviz::MapElement& a, const mpviz::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    // Head piece: stops at vertex 4, the arc's own TRUE start -- reached in
    // full even though it lies 5.065 m past the OLD +/-6.0 m search margin
    // from the fixed-backoff boundary (25.3276 m).
    const auto& head = lane_a[0];
    ASSERT_EQ(head.point_count, 5u);
    EXPECT_NEAR(head.points[0].x, -46.980491978360725, 1e-9) << "lane's own literal first vertex";
    EXPECT_NEAR(head.points[4].x, -46.4658864625989, 1e-6) << "vertex 4 -- the arc's own true start";
    EXPECT_NEAR(head.points[4].y, -14.974389719737527, 1e-6);

    // Tail piece: untouched -- the arc-snap's own outward-only max() never
    // moves this boundary, since the far side's own reachable run (the same
    // vertex 4..7 run, extended the same way from this side) never exceeds
    // the fixed-backoff boundary already at 29.3276.
    const auto& tail = lane_a[1];
    ASSERT_EQ(tail.point_count, 3u);
    EXPECT_NEAR(tail.points[2].x, -30.297919317257183, 1e-9) << "lane's own literal last vertex";
    EXPECT_NEAR(tail.points[2].y, -23.46468621338738, 1e-9);
}

TEST(HdMapAdapter, RedundantArcTailTrimCutsBackToTheCornerArcsDepartureVertex)
{
    // USER CORRECTION, verbatim, to an earlier wrong diagnosis of this same
    // complaint: "you got the leftovers wrongly! Look at the right boundary
    // of a road that has a junction with right exit, you'll find the arc
    // that goes to the right exit road but also a stray straight line for
    // couple meters continuing the boundary line, that's the unwanted
    // leftover I'm talking about."
    //
    // lane 1's own recorded polyline: a straight lead-in, then the SAME
    // R=8 m/75 deg corner arc ArcSnapExtendsCutPastFixedBackoffToTheCorner
    // ArcsFarEdge above uses (the "arc that goes to the right exit road"),
    // then a straight tail continuing 12 m past the arc's own rejoin vertex
    // (v6) -- that tail is the "stray straight line" leftover. lane 2 is a
    // SEPARATE, independently-promoted ROAD_EDGE piece that is what this
    // tail actually duplicates: it shares lane 1's own exact final vertex
    // (9.0, 20.0) and runs within 0.30 m of lane 1's own tail the whole way
    // -- the measured discriminator (kSharedNodeEpsM at the shared vertex,
    // kRoadEdgeCoincidenceThresholdM at the vertex just before it). No
    // crossing exists anywhere in this scene at all (unlike the ArcSnap*
    // tests above) -- this mechanism does not need one, matching the real
    // confirmed instance (lane 955) the measurement pass found, which has
    // no crossing anywhere near its own tail either.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-5.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834},  // v6 -- the arc's own rejoin vertex
                    {9.0, 8.0},                               // redundant tail starts
                    {9.0, 20.0}}),                             // redundant tail ends, shared vertex
        LineMarker("left_boundary_b", 2, {{9.3, 8.0}, {9.0, 20.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a, lane_b;
    for (const auto& e : out.map_elements)
    {
        if (e.kind != mpviz::MapKind::ROAD_EDGE) continue;
        if (e.lane_id == 1u) lane_a.push_back(e);
        if (e.lane_id == 2u) lane_b.push_back(e);
    }

    ASSERT_EQ(lane_a.size(), 1u) << "no crossing exists -- the trim is the only cut on this piece";
    ASSERT_EQ(lane_a[0].point_count, 7u) << "v0..v6 kept -- the redundant tail (v7,v8) is gone";
    // Upstream approach AND the arc itself are untouched -- the trim only
    // ever looks at vertices past the arc's own rejoin vertex.
    EXPECT_NEAR(lane_a[0].points[0].x, -5.0, 1e-9) << "lane's own literal first vertex, unmoved";
    EXPECT_NEAR(lane_a[0].points[0].y, 0.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[6].x, 8.727406610312546, 1e-9)
        << "v6 -- the arc's own true rejoin vertex, a real recorded point, never interpolated";
    EXPECT_NEAR(lane_a[0].points[6].y, 5.929447639179834, 1e-9);

    ASSERT_EQ(lane_b.size(), 1u) << "lane 2 is its own independently-promoted piece, untouched";
    ASSERT_EQ(lane_b[0].point_count, 2u);
}

TEST(HdMapAdapter, RedundantArcTailTrimDoesNotFireOnAnOpenElbowConnector)
{
    // Measured false-positive class (5 of the 19 bag-wide arc-tail
    // instances -- re-derived per-message, order-independent discriminator:
    // lanes 232 and 1029 sit in this coincident-endpoint sub-class, plus 3
    // more with no coincident endpoint at all; lanes 644, 1305, and
    // 14605@hi=8 were misfiled here by an earlier order-dependent
    // measurement but are real true cases -- see hd_map.cpp's own corrected
    // comment on kSharedNodeEpsM): a tail's own last vertex CAN coincide
    // exactly with another piece's own endpoint (kSharedNodeEpsM does NOT
    // discriminate this case -- see hd_map.cpp's own corrected comment on
    // that constant) while the vertex just before it sits outside
    // kRoadEdgeCoincidenceThresholdM of that other piece's polyline, so
    // the trim correctly does not fire. Pinned here at lane 232's own
    // measured value -- the nearest legitimate straight edge sits at
    // 1.5366 m, only a 1.73x margin past kRoadEdgeCoincidenceThresholdM=
    // 1.0 m (NOT the ~30x this file used to attribute to kSharedNodeEpsM)
    // -- rather than an arbitrary far distance, so a future change eroding
    // that real margin fails this test. Same lane-1 arc geometry as the
    // test above; lane 2 now reaches the shared vertex from a direction
    // whose penultimate-vertex distance sits just past the real measured
    // boundary instead of far past it.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-5.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834},
                    {9.0, 8.0},
                    {9.0, 20.0}}),
        LineMarker("left_boundary_b", 2, {{9.0, 20.0}, {10.549, 8.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u);
    ASSERT_EQ(lane_a[0].point_count, 9u)
        << "penultimate vertex ~1.5366 m from lane 2 (lane 232's own measured false-case "
           "distance) -- just past kRoadEdgeCoincidenceThresholdM, condition (b) fails, whole "
           "lane renders unmoved";
}

TEST(HdMapAdapter, RedundantArcTailTrimFiresOnTheWorstMeasuredTrueCase)
{
    // Companion to the false-case test above: pins the OTHER side of the
    // real discriminating band. Lane 792 was the worst (largest lateral
    // offset) of the 14 real bag-wide true cases (re-derived per-message,
    // order-independent discriminator -- see hd_map.cpp's own corrected
    // comment; three instances an earlier order-dependent measurement had
    // misfiled as false, lanes 644/1305/14605@hi=8, are true cases too, but
    // lane 792 remains the worst of all 14), at 0.8883 m -- still
    // comfortably inside kRoadEdgeCoincidenceThresholdM=1.0 m, but only a
    // 1.73x margin from lane 232's 1.5366 m false case above (not the
    // ~30x this file used to wrongly attribute to kSharedNodeEpsM).
    // Bracketing both sides of that measured 0.8883/1.5366 m band around
    // the 1.0 m gate means a future change to that constant that erodes
    // either margin fails a test. Same lane-1 arc geometry as both tests
    // above; lane 2's penultimate-vertex distance sits just inside the
    // measured true-case boundary instead of deep inside it.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-5.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834},
                    {9.0, 8.0},
                    {9.0, 20.0}}),
        LineMarker("left_boundary_b", 2, {{9.0, 20.0}, {9.891, 8.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u);
    ASSERT_EQ(lane_a[0].point_count, 7u)
        << "penultimate vertex ~0.8885 m from lane 2 (lane 792's own measured worst true-case "
           "distance) -- just inside kRoadEdgeCoincidenceThresholdM, condition (b) passes, the "
           "redundant tail is trimmed";
}

TEST(HdMapAdapter, RedundantArcTailTrimDoesNotFireJustOutsideTheSharedNodeEpsilon)
{
    // Pins condition (a), kSharedNodeEpsM, for the piece that never shares
    // an endpoint at all. Measured bag-wide, this condition alone does NO
    // discriminating work against the coincident-endpoint false class --
    // those all sit at 0.0000 m, indistinguishable from the true matches
    // on this condition by itself (see hd_map.cpp's own corrected comment
    // on kSharedNodeEpsM); its only real job is excluding the 3 genuinely
    // free-floating tails, nearest 3.43 m -- NOT 1.537 m, which is a
    // *different* false case's condition-(b) penultimate distance, not a
    // nearest-false-match margin on this condition. Here lane 2's near
    // endpoint sits 0.20 m from lane 1's own tail end -- outside
    // kSharedNodeEpsM=0.10 m -- even though it otherwise runs close
    // alongside lane 1's tail exactly like the confirmed-duplicate test
    // above -- proximity along the way is not enough without the
    // shared-endpoint condition too.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-5.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834},
                    {9.0, 8.0},
                    {9.0, 20.0}}),
        LineMarker("left_boundary_b", 2, {{9.3, 8.0}, {9.2, 20.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u);
    ASSERT_EQ(lane_a[0].point_count, 9u)
        << "0.2 m gap at the tail's own far vertex is outside kSharedNodeEpsM -- no trim";
}

TEST(HdMapAdapter, NeighborArcDepartureSnapCutsBackAStraightEdgeOvershootingTheCorner)
{
    // Code-review finding 2026-09-09, blocking, re-targeting
    // TrimRedundantArcTails at the class criterion 1 and the user's own
    // words actually describe: "the arc that goes to the right exit road
    // but also a stray straight line for couple meters continuing the
    // boundary line" is, in the finding's own verified frame (msg 452, lane
    // 955 x lane 12), NOT lane 955's own tail -- it is lane 12, the through
    // road's own straight boundary, sharing lane 955's start node and
    // continuing straight past lane 955's own corner-arc departure vertex,
    // because lane 12 carries no arc of its own to snap to.
    //
    // lane 1 here plays lane 955: the SAME R=8 m/75 deg corner arc the
    // ArcSnap*/RedundantArcTailTrim* tests above use (v0 = shared node, v2 =
    // the corner's own departure vertex, v6 = rejoin -- unused here, no tail
    // needed). lane 3 plays lane 12: a plain straight boundary sharing lane
    // 1's own start node (-5,0), crossed by lane 4 (an unrelated crossing,
    // playing lane 1591/1320's role) far past the corner at x=15 -- the OLD
    // fixed kJunctionCutBackoffM=2.0 m back-off alone puts lane 3's own kept
    // head at x=13, station 18, well past v2 at station 8.0706 (matching
    // the real corner's own 3.24 m overshoot in kind, not magnitude). lane 4
    // is placed at x=15 (far past lane 1's own x<=8.73 extent) specifically
    // so it crosses ONLY lane 3, not lane 1 -- isolating this mechanism from
    // the ordinary crossing-cut.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-5.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},  // v2 -- the corner's own departure
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834}}),
        LineMarker("left_boundary_c", 3, {{-5.0, 0.0}, {30.0, 0.0}}),
        LineMarker("left_boundary_d", 4, {{15.0, -10.0}, {15.0, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_a, lane_c;
    for (const auto& e : out.map_elements)
    {
        if (e.kind != mpviz::MapKind::ROAD_EDGE) continue;
        if (e.lane_id == 1u) lane_a.push_back(e);
        if (e.lane_id == 3u) lane_c.push_back(e);
    }

    ASSERT_EQ(lane_a.size(), 1u) << "lane 1 crosses nothing in this scene -- untouched, whole";
    ASSERT_EQ(lane_a[0].point_count, 7u);

    ASSERT_EQ(lane_c.size(), 2u) << "one crossing on lane 3 -- head piece and tail piece";
    std::sort(lane_c.begin(), lane_c.end(),
              [](const mpviz::MapElement& x, const mpviz::MapElement& y) {
                  return x.points[0].x < y.points[0].x;
              });

    // Head piece: WITHOUT this fix, the fixed backoff alone would keep this
    // at x=13 (station 18). With it, the boundary is pulled back to v2's
    // own station projected onto lane 3's straight line -- the exact
    // physical point where lane 1's own curb starts curving away.
    ASSERT_EQ(lane_c[0].point_count, 2u);
    EXPECT_NEAR(lane_c[0].points[0].x, -5.0, 1e-9) << "lane 3's own literal start, unmoved";
    EXPECT_NEAR(lane_c[0].points[1].x, 3.070552360820166, 1e-9)
        << "pulled back to v2's own station -- not left at the old fixed-backoff x=13";
    EXPECT_NEAR(lane_c[0].points[1].y, 0.0, 1e-9);

    // Tail piece: lane 3's own end (30,0) shares no node with anything --
    // untouched, still at the old fixed-backoff point (x=17).
    ASSERT_EQ(lane_c[1].point_count, 2u);
    EXPECT_NEAR(lane_c[1].points[0].x, 17.0, 1e-9);
    EXPECT_NEAR(lane_c[1].points[1].x, 30.0, 1e-9);
}

TEST(HdMapAdapter, NeighborArcDepartureSnapDoesNotFireWithoutASharedNode)
{
    // Pins the scoping condition: moving lane 3's own start 0.2 m away from
    // lane 1's start -- outside kSharedNodeEpsM=0.10 m, the SAME
    // node-coincidence gate TrimRedundantArcTails's own condition (a) uses
    // -- means no neighbour is found to snap to at all, so the ordinary
    // fixed-backoff crossing-cut boundary is left exactly where it was.
    // Same scene as the test above otherwise.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1,
                   {{-5.0, 0.0},
                    {1.0, 0.0},
                    {3.070552360820166, 0.2725933896874535},
                    {5.0, 1.0717967697244903},
                    {6.65685424949238, 2.3431457505076194},
                    {7.928203230275509, 3.999999999999999},
                    {8.727406610312546, 5.929447639179834}}),
        LineMarker("left_boundary_c", 3, {{-5.2, 0.0}, {30.0, 0.0}}),
        LineMarker("left_boundary_d", 4, {{15.0, -10.0}, {15.0, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<mpviz::MapElement> lane_c;
    for (const auto& e : out.map_elements)
    {
        if (e.kind == mpviz::MapKind::ROAD_EDGE && e.lane_id == 3u) lane_c.push_back(e);
    }
    ASSERT_EQ(lane_c.size(), 2u);
    std::sort(lane_c.begin(), lane_c.end(),
              [](const mpviz::MapElement& x, const mpviz::MapElement& y) {
                  return x.points[0].x < y.points[0].x;
              });
    ASSERT_EQ(lane_c[0].point_count, 2u);
    EXPECT_NEAR(lane_c[0].points[1].x, 13.0, 1e-9)
        << "0.2 m outside kSharedNodeEpsM -- no neighbour found, old fixed-backoff point unmoved "
           "(2.0 m back off the x=15 crossing, same physical point regardless of this lane's own "
           "start offset)";
}

TEST(HdMapAdapter, OneMarkerCountsAsOneIngestedMarkerForStats)
{
    // A centerline marker is one MARKER on the wire and one ingest() call,
    // producing one MapElement (dashing moved renderer-side, decision #3)
    // -- msgs and the dropped_* counters reflect that one marker.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);
    a.ingest(StraightTenMeterMarker("centerline_x"), 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);

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
    // tf2::Transform's own point-multiply composition order. Single marker,
    // so it stays exactly one MapElement.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

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
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);
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
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

// DashChopHappensAfterPoseComposition (pre-Epic3): DELETED. Its premise --
// an adapter-side chop that could theoretically run before vs. after pose
// composition -- no longer exists (decision #3: chopping is entirely
// renderer-side now and only ever sees already-posed points crossing the
// ABI boundary). No adapter-side successor is needed.

TEST(HdMapAdapter, ZeroQuaternionPoseIsTreatedAsIdentityRotationNotNan)
{
    // rviz renders a zero-filled orientation as identity (with a console
    // warning); handing it to tf2 NaNs every point and silently voids the
    // whole marker as dropped_malformed. Zero quat + translation -> points
    // still come through translated, nothing counted malformed.
    TfFixture kTf;
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);
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
    mpviz_node::HdMapAdapter a(MapRuleRow(), kTf.tf);
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
    mpviz_node::HdMapAdapter a(MapRuleRow(), tf3d);
    auto arr = StraightTenMeterMarker("centerline_x");
    for (auto& p : arr.markers[0].points) p.z = 3.0;
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[0].z, 3.0);
}
