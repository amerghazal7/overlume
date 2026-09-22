// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/hd_map.hpp"

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

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

}  // namespace

TEST(HdMapAdapter, LocalElementsFixtureYieldsLanesAndCrosswalks) {
    auto msg = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(overlume::ros::testing::urban_row("/hd_map_local_elements"),
                                  kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    EXPECT_GT(out.map_elements.size(), 0u);
    EXPECT_EQ(out.map_elements.size(), 60u);
    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                            [](const overlume::MapElement& m) { return m.is_polygon == 1; }));
    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                            [](const overlume::MapElement& m) { return m.is_polygon == 0; }));
    EXPECT_EQ(std::count_if(out.map_elements.begin(), out.map_elements.end(),
                            [](const overlume::MapElement& m) {
                                return m.kind == overlume::MapKind::CENTERLINE;
                            }),
              0);

    const auto road_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const overlume::MapElement& m) { return m.kind == overlume::MapKind::ROAD_SURFACE; });
    EXPECT_EQ(road_count, 16);
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_SURFACE) EXPECT_EQ(e.point_count, 32u);
    }

    const auto left_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const overlume::MapElement& m) { return m.kind == overlume::MapKind::LEFT_BOUNDARY; });
    const auto right_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const overlume::MapElement& m) { return m.kind == overlume::MapKind::RIGHT_BOUNDARY; });
    const auto road_edge_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const overlume::MapElement& m) { return m.kind == overlume::MapKind::ROAD_EDGE; });
    EXPECT_EQ(left_count, 9);
    EXPECT_EQ(right_count, 8);
    EXPECT_EQ(road_edge_count, 17);
    EXPECT_EQ(left_count + right_count + road_edge_count, 34);
    for (const auto& e : out.map_elements) {
        if (e.lane_id != 934u) continue;
        if (e.kind == overlume::MapKind::LEFT_BOUNDARY ||
            e.kind == overlume::MapKind::RIGHT_BOUNDARY ||
            e.kind == overlume::MapKind::ROAD_SURFACE) {
            continue;
        }
        ADD_FAILURE() << "lane 934 has an unexpected kind " << static_cast<int>(e.kind)
                      << " -- it is measured fully-interior and should never promote to ROAD_EDGE";
    }

    {
        std::vector<overlume::MapElement> lane792;
        for (const auto& e : out.map_elements) {
            if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 792u) lane792.push_back(e);
        }
        ASSERT_EQ(lane792.size(), 2u);
        std::sort(lane792.begin(), lane792.end(),
                  [](const overlume::MapElement& a, const overlume::MapElement& b) {
                      return a.points[0].y > b.points[0].y;
                  });
        const auto& head = lane792[0];
        EXPECT_NEAR(head.points[head.point_count - 1].x, -46.4658864625989, 1e-6);
        EXPECT_NEAR(head.points[head.point_count - 1].y, -14.974389719737527, 1e-6);
    }

    if (const char* geom_path = std::getenv("OVERLUME_EMIT_GEOM")) {
        std::ofstream geom(geom_path);
        geom << std::setprecision(12);
        for (const auto& e : out.map_elements) {
            geom << static_cast<int>(e.is_polygon) << ' ' << static_cast<int>(e.kind) << ' '
                 << e.lane_id << ' ' << e.point_count;
            for (uint32_t i = 0; i < e.point_count; ++i) {
                geom << ' ' << e.points[i].x << ' ' << e.points[i].y << ' ' << e.points[i].z;
            }
            geom << '\n';
        }
    }
}

TEST(HdMapAdapter, SimProfileRowMakesCrosswalksPolygonsToo) {
    auto msg = overlume::ros::testing::load_marker_array("sim_hd_map_markers_0.yaml");
    TfFixture kTf;
    auto row = overlume::ros::testing::sim_row("/sim/hd_map/markers");
    overlume::ros::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    EXPECT_TRUE(std::any_of(out.map_elements.begin(), out.map_elements.end(),
                            [](const overlume::MapElement& m) { return m.is_polygon == 1; }));

    uint64_t expected_drops = 0;
    for (const auto& m : msg.markers) {
        if (m.action == 3) continue;
        if (overlume::ros::classify(row, m.ns) == overlume::ros::NsRender::kDrop) ++expected_drops;
    }
    EXPECT_GT(expected_drops, 0u);
    EXPECT_EQ(a.stats().dropped_by_rule, expected_drops);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HdMapAdapter, SimCrosswalksPluralUnsuffixedGetsCrosswalkKind) {
    auto msg = overlume::ros::testing::load_marker_array("sim_hd_map_markers_0.yaml");
    TfFixture kTf;
    auto row = overlume::ros::testing::sim_row("/sim/hd_map/markers");
    overlume::ros::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    bool found = false;
    for (const auto& e : out.map_elements) {
        if (e.is_polygon != 1) continue;
        found = true;
        EXPECT_EQ(e.kind, overlume::MapKind::CROSSWALK);
    }
    ASSERT_TRUE(found) << "no polygon (crosswalk) element found in sim fixture output";
}

TEST(HdMapAdapter, CrosswalkTrailingDuplicateVertexIsDeduped) {
    auto msg = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(overlume::ros::testing::urban_row("/hd_map_local_elements"),
                                  kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    bool found = false;
    for (const auto& e : out.map_elements) {
        if (e.kind != overlume::MapKind::CROSSWALK || e.point_count == 0) continue;
        if (std::abs(e.points[0].x - (-39.50850289011474)) < 1e-6 &&
            std::abs(e.points[0].y - 45.33743457749722) < 1e-6) {
            found = true;
            EXPECT_EQ(e.point_count, 4u);
            EXPECT_EQ(e.lane_id, 0u);
        }
    }
    ASSERT_TRUE(found) << "crosswalk_8043 element not found in fill() output";
}

TEST(HdMapAdapter, ArrowNamespaceIsDroppedByLongestPrefixWins) {
    auto msg = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_by_rule, 80u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HdMapAdapter, NonMapFrameMessageIsTransformedNotCopied) {
    auto msg = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");
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
    buffer.setTransform(xf, "test_authority", true);
    FrameTransformer ft(buffer);

    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, ft);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);
    ASSERT_GT(out.map_elements.size(), 0u);

    bool any_far = false;
    for (const auto& e : out.map_elements) {
        for (uint32_t i = 0; i < e.point_count; ++i) {
            if (e.points[i].x > 900.0 && e.points[i].y > 1900.0) any_far = true;
        }
    }
    EXPECT_TRUE(any_far);
}

TEST(HdMapAdapter, TfLookupFailureDropsTheMessageAndCounts) {
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, kTf.tf);

    auto good = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");
    a.ingest(good, 1.0);
    SceneAssembly before;
    a.fill(before);
    ASSERT_GT(before.map_elements.size(), 0u);

    auto bad = good;
    for (auto& m : bad.markers) m.header.frame_id = "sensor_frame_nobody_ever_published";
    a.ingest(bad, 2.0);
    EXPECT_EQ(a.stats().dropped_no_tf, 1u);

    SceneAssembly after;
    a.fill(after);
    EXPECT_EQ(after.map_elements.size(), before.map_elements.size());
}

TEST(HdMapAdapter, DeleteAllClearsPreviousElements) {
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, kTf.tf);
    auto msg = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");

    a.ingest(msg, 1.0);
    SceneAssembly first;
    a.fill(first);
    ASSERT_GT(first.map_elements.size(), 0u);

    a.ingest(msg, 3.0);
    SceneAssembly second;
    a.fill(second);
    EXPECT_EQ(second.map_elements.size(), first.map_elements.size());
}

TEST(HdMapAdapter, MalformedMarkersAreDroppedAndCounted) {
    auto msg = overlume::ros::testing::load_marker_array("hd_map_malformed_0.yaml");
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);

    EXPECT_EQ(a.stats().dropped_malformed, 3u);
    EXPECT_EQ(a.stats().dropped_by_rule, 0u);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    for (const auto& e : out.map_elements) EXPECT_EQ(e.point_count, 2u);
}

TEST(HdMapAdapter, FillStampsLastUpdateSecOnEveryElementIncludingRoadSurface) {
    auto msg = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, kTf.tf);
    constexpr double kSimTime = 42.0;
    a.ingest(msg, kSimTime);
    SceneAssembly out;
    a.fill(out);

    constexpr double kMapFadeWindowSec = 1.0;
    const double expected_stamp = kSimTime + (row.timeout_sec - kMapFadeWindowSec);
    ASSERT_GT(out.map_elements.size(), 0u);
    bool saw_road_surface = false;
    for (const auto& e : out.map_elements) {
        EXPECT_DOUBLE_EQ(e.last_update_sec, expected_stamp)
            << "element kind=" << static_cast<int>(e.kind) << " lane_id=" << e.lane_id
            << " was not stamped from the ingest sim time offset by (timeout_sec - "
               "kMapFadeWindowSec)";
        if (e.kind == overlume::MapKind::ROAD_SURFACE) saw_road_surface = true;
    }
    EXPECT_TRUE(saw_road_surface) << "fixture no longer synthesizes a ROAD_SURFACE element";
}

TEST(HdMapAdapter, ThrottledRowStaysFreshOnReceiptNotOnAcceptedRebuildCadence) {
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_global_elements");
    ASSERT_DOUBLE_EQ(row.max_rate_hz, 0.5)
        << "urban_profile.yaml's row no longer matches this test's premise";
    overlume::ros::HdMapAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "left_boundary_x";
    m.id = 1;
    m.type = 4;
    m.action = 0;
    geometry_msgs::msg::Point p0, p1;
    p1.x = 10.0;
    m.points = {p0, p1};
    msg.markers = {m};

    for (double t = 0.0; t <= 1.5 + 1e-9; t += 0.3) {
        a.ingest(msg, t);
    }

    SceneAssembly out;
    a.fill(out);
    constexpr double kMapFadeWindowSec = 1.0;
    const double expected_stamp = 1.5 + (row.timeout_sec - kMapFadeWindowSec);
    ASSERT_GT(out.map_elements.size(), 0u);
    for (const auto& e : out.map_elements) {
        EXPECT_NEAR(e.last_update_sec, expected_stamp, 1e-9)
            << "stamp tracked the throttled rebuild cadence instead of topic liveness -- "
               "the map layer would blink dark between rebuilds";
    }
}

TEST(HdMapAdapter, LowRateReceiptDoesNotSawtoothBetweenReceipts) {
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, kTf.tf);

    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "left_boundary_x";
    m.id = 1;
    m.type = 4;
    m.action = 0;
    geometry_msgs::msg::Point p0, p1;
    p1.x = 10.0;
    m.points = {p0, p1};
    msg.markers = {m};

    constexpr double kStaleFadeStartSec = 0.5;
    constexpr double kMapFadeWindowSec = 1.0;
    constexpr double kReceiptPeriodSec = 1.0;
    for (int receipt = 0; receipt < 5; ++receipt) {
        const double t_recv = receipt * kReceiptPeriodSec;
        a.ingest(msg, t_recv);

        SceneAssembly out;
        a.fill(out);
        ASSERT_GT(out.map_elements.size(), 0u);

        const double now_just_before_next_receipt = t_recv + kReceiptPeriodSec - 1e-3;
        const double expected_stamp = t_recv + (row.timeout_sec - kMapFadeWindowSec);
        for (const auto& e : out.map_elements) {
            EXPECT_DOUBLE_EQ(e.last_update_sec, expected_stamp)
                << "receipt #" << receipt << ": stamp is not the offset-into-timeout_sec value";
            const double age = now_just_before_next_receipt - e.last_update_sec;
            EXPECT_LT(age, kStaleFadeStartSec)
                << "receipt #" << receipt
                << ": a 1 Hz-received row has already started "
                   "fading (age >= kStaleFadeStartSec) just before its next receipt -- "
                   "the fix does not hold in the 1-2 Hz regime";
        }
    }
}

TEST(HdMapAdapter, PublishOnceTransientLocalRowStaysOpaqueWellPastOldOneSecondFadeFloor) {
    TfFixture kTf;
    auto row = overlume::ros::testing::sim_row("/sim/hd_map/markers");
    ASSERT_DOUBLE_EQ(row.timeout_sec, 5.0)
        << "sim_profile.yaml's row no longer matches this test's premise";
    overlume::ros::HdMapAdapter a(row, kTf.tf);

    auto msg = overlume::ros::testing::load_marker_array("sim_hd_map_markers_0.yaml");
    a.ingest(msg, 0.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_GT(out.map_elements.size(), 0u);

    constexpr double kStaleFadeStartSec = 0.5;
    constexpr double kSimTimeNow = 2.0;
    for (const auto& e : out.map_elements) {
        const double age = kSimTimeNow - e.last_update_sec;
        EXPECT_LT(age, kStaleFadeStartSec)
            << "the publish-once/transient_local row faded before its own timeout_sec cutoff -- "
               "this is exactly the 'map never appears' regression the fix must close";
    }
}

TEST(HdMapAdapter, RateLimitHonoursMaxRateHz) {
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/hd_map_local_elements");
    overlume::ros::HdMapAdapter a(row, kTf.tf);

    auto make_one_marker = [](const char* ns, int32_t id, double x0) {
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

    a.ingest(make_one_marker("left_boundary_a", 1, 0.0), 1.0);
    a.ingest(make_one_marker("left_boundary_b", 2, 10.0), 1.1);
    a.ingest(make_one_marker("left_boundary_c", 3, 20.0), 1.2);
    a.ingest(make_one_marker("left_boundary_d", 4, 30.0), 1.6);

    SceneAssembly out;
    a.fill(out);
    EXPECT_EQ(out.map_elements.size(), 2u);
}

namespace {

overlume::ros::ProfileRow MapRuleRow() {
    std::vector<std::string> errs;
    const std::string yaml =
        "name: t\nrows:\n  - {topic: /hd_map, type: visualization_msgs/msg/MarkerArray,"
        " adapter: hd_map, role: lane, ns_default: drop, namespaces:"
        " [{prefix: centerline_,     render: polyline, kind: centerline},"
        "  {prefix: left_boundary_,  render: polyline, kind: left_boundary},"
        "  {prefix: right_boundary_, render: polyline, kind: right_boundary}]}\n";
    auto p = overlume::ros::load_profile_string(yaml, errs);
    if (!p) throw std::runtime_error("MapRuleRow: profile failed to parse");
    return p->rows[0];
}

visualization_msgs::msg::Marker RailMarker(const char* ns, int32_t id, double x) {
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

overlume::ros::ProfileRow JunctionRuleRow(bool junction_interior_boundaries = true) {
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
    auto p = overlume::ros::load_profile_string(yaml, errs);
    if (!p)
        throw std::runtime_error("JunctionRuleRow: profile failed to parse: " +
                                 (errs.empty() ? "" : errs[0]));
    return p->rows[0];
}

visualization_msgs::msg::Marker LineMarker(const char* ns, int32_t id,
                                           const std::vector<std::pair<double, double>>& xy) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = id;
    m.type = 4;
    m.action = 0;
    for (const auto& [x, y] : xy) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        m.points.push_back(p);
    }
    return m;
}

visualization_msgs::msg::MarkerArray StraightTenMeterMarker(const char* ns) {
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

TEST(HdMapAdapter, MarkerOfAnyKindStaysOneElementOutOfTheAdapter) {
    TfFixture kTf;

    overlume::ros::HdMapAdapter boundary(MapRuleRow(), kTf.tf);
    boundary.ingest(StraightTenMeterMarker("left_boundary_x"), 1.0);
    SceneAssembly boundary_out;
    boundary.fill(boundary_out);
    ASSERT_EQ(boundary_out.map_elements.size(), 1u);
    EXPECT_EQ(boundary_out.map_elements[0].point_count, 2u);
    EXPECT_EQ(boundary_out.map_elements[0].kind, overlume::MapKind::ROAD_EDGE);

    overlume::ros::HdMapAdapter centerline(MapRuleRow(), kTf.tf);
    centerline.ingest(StraightTenMeterMarker("centerline_x"), 1.0);
    SceneAssembly centerline_out;
    centerline.fill(centerline_out);
    ASSERT_EQ(centerline_out.map_elements.size(), 1u);
    EXPECT_EQ(centerline_out.map_elements[0].point_count, 2u);
    EXPECT_EQ(centerline_out.map_elements[0].kind, overlume::MapKind::CENTERLINE);
}

TEST(HdMapAdapter, CenterlineAndBoundaryKindAndLaneIdFromMarkerId) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::CENTERLINE) {
            found_centerline = true;
            EXPECT_EQ(e.lane_id, 934u);
        } else if (e.kind == overlume::MapKind::ROAD_EDGE) {
            found_boundary = true;
            EXPECT_EQ(e.lane_id, 934u);
        }
    }
    EXPECT_TRUE(found_centerline);
    EXPECT_TRUE(found_boundary);
}

TEST(HdMapAdapter, LaneWithOnlyOneBoundaryProducesNoRoadSurfaceElement) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);
    a.ingest(StraightTenMeterMarker("left_boundary_x"), 1.0);

    SceneAssembly out;
    a.fill(out);
    const auto road_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const overlume::MapElement& m) { return m.kind == overlume::MapKind::ROAD_SURFACE; });
    EXPECT_EQ(road_count, 0);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HdMapAdapter, RoadSurfaceFillPersistsFromCacheWhenARailDropsForOneTick) {
    TfFixture kTf;
    auto row = MapRuleRow();
    overlume::ros::HdMapAdapter a(row, kTf.tf);

    auto rail_marker = [](const char* ns, int32_t id, double x) {
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
    };
    auto delete_all_marker = []() {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.ns = "";
        m.id = 0;
        m.action = 3;
        return m;
    };
    auto lane42_both_rails = [&]() {
        visualization_msgs::msg::MarkerArray arr;
        arr.markers = {delete_all_marker(), rail_marker("left_boundary_42", 42, 0.0),
                       rail_marker("right_boundary_42", 42, 3.2)};
        return arr;
    };
    auto lane42_left_rail_only = [&]() {
        visualization_msgs::msg::MarkerArray arr;
        arr.markers = {delete_all_marker(), rail_marker("left_boundary_42", 42, 0.0)};
        return arr;
    };
    auto find_lane42_fill = [](const SceneAssembly& out) -> const overlume::MapElement* {
        for (const auto& e : out.map_elements) {
            if (e.kind == overlume::MapKind::ROAD_SURFACE && e.lane_id == 42u) return &e;
        }
        return nullptr;
    };

    a.ingest(lane42_both_rails(), 1.0);
    SceneAssembly out1;
    a.fill(out1);
    const overlume::MapElement* fill1 = find_lane42_fill(out1);
    ASSERT_NE(fill1, nullptr)
        << "lane 42 did not synthesize a ROAD_SURFACE element with both rails present";
    const uint32_t point_count = fill1->point_count;

    a.ingest(lane42_left_rail_only(), 1.5);
    SceneAssembly out2;
    a.fill(out2);
    const overlume::MapElement* fill2 = find_lane42_fill(out2);
    ASSERT_NE(fill2, nullptr)
        << "lane 42's ROAD_SURFACE fill vanished for one tick instead of holding its cached pair "
           "-- this is the flicker VM/2026-09-22 root-caused on the real-robot replay";
    EXPECT_EQ(fill2->point_count, point_count);

    a.ingest(lane42_left_rail_only(), 1.0 + row.timeout_sec + 0.1);
    SceneAssembly out3;
    a.fill(out3);
    EXPECT_EQ(find_lane42_fill(out3), nullptr)
        << "lane 42's stale ROAD_SURFACE cache entry outlived row.timeout_sec";
}

TEST(HdMapAdapter, RoadSurfaceFillCacheRetiresWhenTheClockRewinds) {
    TfFixture kTf;
    auto row = MapRuleRow();
    overlume::ros::HdMapAdapter a(row, kTf.tf);

    auto rail_marker = [](const char* ns, int32_t id, double x) {
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
    };
    auto delete_all_marker = []() {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.ns = "";
        m.id = 0;
        m.action = 3;
        return m;
    };
    auto snapshot = [&](bool with_right) {
        visualization_msgs::msg::MarkerArray arr;
        arr.markers = {delete_all_marker(), rail_marker("left_boundary_42", 42, 0.0)};
        if (with_right) arr.markers.push_back(rail_marker("right_boundary_42", 42, 3.2));
        return arr;
    };
    auto find_lane42_fill = [](const SceneAssembly& out) -> const overlume::MapElement* {
        for (const auto& e : out.map_elements) {
            if (e.kind == overlume::MapKind::ROAD_SURFACE && e.lane_id == 42u) return &e;
        }
        return nullptr;
    };

    a.ingest(snapshot(true), 400.0);
    SceneAssembly out1;
    a.fill(out1);
    ASSERT_NE(find_lane42_fill(out1), nullptr);

    a.ingest(snapshot(false), 1.0);
    SceneAssembly out2;
    a.fill(out2);
    EXPECT_EQ(find_lane42_fill(out2), nullptr)
        << "a pre-rewind ROAD_SURFACE cache entry survived the clock going backwards -- it would "
           "be re-emitted for the rest of the replay as a ghost carriageway";
}

TEST(HdMapAdapter, MismatchedRailPointCountsResampledStationsSpanRecordedEndpoints) {
    auto msg = overlume::ros::testing::load_marker_array("hd_map_local_elements_0.yaml");
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(overlume::ros::testing::urban_row("/hd_map_local_elements"),
                                  kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly out;
    a.fill(out);

    const geometry_msgs::msg::Point* left_first = nullptr;
    const geometry_msgs::msg::Point* left_last = nullptr;
    const geometry_msgs::msg::Point* right_first = nullptr;
    const geometry_msgs::msg::Point* right_last = nullptr;
    for (const auto& m : msg.markers) {
        if (m.ns == "left_boundary_955" && !m.points.empty()) {
            left_first = &m.points.front();
            left_last = &m.points.back();
        } else if (m.ns == "right_boundary_955" && !m.points.empty()) {
            right_first = &m.points.front();
            right_last = &m.points.back();
        }
    }
    ASSERT_NE(left_first, nullptr) << "fixture no longer carries left_boundary_955";
    ASSERT_NE(right_first, nullptr) << "fixture no longer carries right_boundary_955";

    const overlume::MapElement* road = nullptr;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_SURFACE && e.lane_id == 955u) {
            road = &e;
            break;
        }
    }
    ASSERT_NE(road, nullptr) << "lane 955 produced no ROAD_SURFACE element";
    ASSERT_EQ(road->point_count, 32u);

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

TEST(HdMapAdapter, OuterBoundariesOfThreeAdjacentLanesPromoteToRoadEdge) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        RailMarker("left_boundary_a", 1, 0.0), RailMarker("right_boundary_a", 1, 3.2),
        RailMarker("left_boundary_b", 2, 3.2), RailMarker("right_boundary_b", 2, 6.4),
        RailMarker("left_boundary_c", 3, 6.4), RailMarker("right_boundary_c", 3, 9.6),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    auto kind_at_x = [&](double x) -> overlume::MapKind {
        for (const auto& e : out.map_elements) {
            if (e.kind == overlume::MapKind::ROAD_SURFACE) continue;
            if (std::abs(e.points[0].x - x) < 1e-9) return e.kind;
        }
        ADD_FAILURE() << "no element found at x=" << x;
        return overlume::MapKind::OTHER;
    };
    EXPECT_EQ(kind_at_x(0.0), overlume::MapKind::ROAD_EDGE) << "lane A's left rail: outer edge";
    EXPECT_EQ(kind_at_x(3.2), overlume::MapKind::LEFT_BOUNDARY)
        << "lane B's left rail == lane A's right rail: shared interior divider";
    EXPECT_EQ(kind_at_x(6.4), overlume::MapKind::LEFT_BOUNDARY)
        << "lane C's left rail == lane B's right rail: shared interior divider";
    EXPECT_EQ(kind_at_x(9.6), overlume::MapKind::ROAD_EDGE) << "lane C's right rail: outer edge";

    const auto road_edge_count = std::count_if(
        out.map_elements.begin(), out.map_elements.end(),
        [](const overlume::MapElement& m) { return m.kind == overlume::MapKind::ROAD_EDGE; });
    EXPECT_EQ(road_edge_count, 2);
}

TEST(HdMapAdapter, SingleIsolatedLaneHasBothBoundariesPromotedToRoadEdge) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {RailMarker("left_boundary_solo", 1, 0.0),
                   RailMarker("right_boundary_solo", 1, 3.2)};
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    int road_edge_count = 0;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_SURFACE) continue;
        EXPECT_EQ(e.kind, overlume::MapKind::ROAD_EDGE);
        ++road_edge_count;
    }
    EXPECT_EQ(road_edge_count, 2);
}

TEST(HdMapAdapter, JunctionPolygonClipSplitsRoadEdgeIntoTwoSubElementsWithInterpolatedCuts) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(JunctionRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_edge", 1, {{0, -10}, {0, -5}, {0, 0}, {0, 5}, {0, 10}}),
        LineMarker("junction", 900, {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}, {-3, -3}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> edges;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE) edges.push_back(e);
    }
    ASSERT_EQ(edges.size(), 2u) << "one edge crossing the junction box splits into two";
    std::sort(edges.begin(), edges.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].y < b.points[0].y;
              });

    EXPECT_DOUBLE_EQ(edges[0].points[0].y, -10.0);
    EXPECT_NEAR(edges[0].points[edges[0].point_count - 1].y, -3.0, 1e-6);
    EXPECT_NE(edges[0].points[edges[0].point_count - 1].y, -5.0);

    EXPECT_NEAR(edges[1].points[0].y, 3.0, 1e-6);
    EXPECT_DOUBLE_EQ(edges[1].points[edges[1].point_count - 1].y, 10.0);
    EXPECT_NE(edges[1].points[0].y, 5.0);
}

TEST(HdMapAdapter, MutualCrossingCutSplitsBothRoadEdgesWithBackoff) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-10, 0}, {10, 0}}),
        LineMarker("left_boundary_b", 2, {{0, -10}, {0, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a, lane_b;
    for (const auto& e : out.map_elements) {
        if (e.kind != overlume::MapKind::ROAD_EDGE) continue;
        (e.lane_id == 1u ? lane_a : lane_b).push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    ASSERT_EQ(lane_b.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    std::sort(lane_b.begin(), lane_b.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].y < b.points[0].y;
              });

    EXPECT_EQ(lane_a[0].point_count, 2u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -10.0);
    EXPECT_NEAR(lane_a[0].points[1].x, -2.0, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, 2.0, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[1].points[1].x, 10.0);

    EXPECT_NEAR(lane_b[0].points[1].y, -2.0, 1e-9);
    EXPECT_NEAR(lane_b[1].points[0].y, 2.0, 1e-9);
}

TEST(HdMapAdapter, ParallelRoadEdgesAreNeverCut) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{0, -10}, {0, 10}}),
        LineMarker("left_boundary_b", 2, {{5, -10}, {5, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    int road_edge_count = 0;
    for (const auto& e : out.map_elements) {
        if (e.kind != overlume::MapKind::ROAD_EDGE) continue;
        ++road_edge_count;
        EXPECT_EQ(e.point_count, 2u) << "untouched -- no cut introduced";
    }
    EXPECT_EQ(road_edge_count, 2);
}

TEST(HdMapAdapter, AbuttingNearCollinearRoadEdgesAreNeverCut) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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
    for (const auto& e : out.map_elements) {
        if (e.kind != overlume::MapKind::ROAD_EDGE) continue;
        ++road_edge_count;
        EXPECT_EQ(e.point_count, 2u) << "untouched -- a ~1 deg kink is not a crossing";
    }
    EXPECT_EQ(road_edge_count, 2);
}

TEST(HdMapAdapter, JunctionGapMergeCutsSliverBetweenCloseCrossings) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-20, 0}, {20, 0}}),
        LineMarker("left_boundary_b", 2, {{-4.5, -10}, {-4.5, 10}}),
        LineMarker("left_boundary_c", 3, {{4.5, -10}, {4.5, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "the two close-together crossings merge into one cut -- "
                                    "no sliver piece survives between them";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -20.0);
    EXPECT_NEAR(lane_a[0].points[lane_a[0].point_count - 1].x, -6.5, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, 6.5, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[1].points[lane_a[1].point_count - 1].x, 20.0);
}

TEST(HdMapAdapter, JunctionGapMergeAtThresholdStillMerges) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-20, 0}, {20, 0}}),
        LineMarker("left_boundary_b", 2, {{-5.3, -10}, {-5.3, 10}}),
        LineMarker("left_boundary_c", 3, {{5.3, -10}, {5.3, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "gap exactly at the threshold still merges (inclusive test)";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    EXPECT_NEAR(lane_a[0].points[lane_a[0].point_count - 1].x, -7.3, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, 7.3, 1e-9);
}

TEST(HdMapAdapter, JunctionGapMergeKeepsLegitInteriorSpanAboveThreshold) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 1, {{-20, 0}, {20, 0}}),
        LineMarker("left_boundary_b", 2, {{-5.5, -10}, {-5.5, 10}}),
        LineMarker("left_boundary_c", 3, {{5.5, -10}, {5.5, 10}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 3u)
        << "head, kept interior span, tail -- the gap is too wide to merge";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -20.0);
    EXPECT_NEAR(lane_a[0].points[lane_a[0].point_count - 1].x, -7.5, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].x, -3.5, 1e-9);
    EXPECT_NEAR(lane_a[1].points[lane_a[1].point_count - 1].x, 3.5, 1e-9);
    EXPECT_NEAR(lane_a[2].points[0].x, 7.5, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[2].points[lane_a[2].point_count - 1].x, 20.0);
}

TEST(HdMapAdapter, BoundariesUntouchedByJunctionCutsAtDefaultFlag) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(JunctionRuleRow(true), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_b", 2, {{0, -10}, {0, -5}, {0, 0}, {0, 5}, {0, 10}}),
        LineMarker("right_boundary_a", 1, {{0, -10}, {0, 10}}),
        LineMarker("junction", 900, {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}, {-3, -3}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> left;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::LEFT_BOUNDARY) left.push_back(e);
    }
    ASSERT_EQ(left.size(), 1u) << "not split -- the junction polygon clip never ran on it";
    ASSERT_EQ(left[0].point_count, 5u) << "every original point survives, untouched";
    EXPECT_DOUBLE_EQ(left[0].points[0].y, -10.0);
    EXPECT_DOUBLE_EQ(left[0].points[2].y, 0.0);
    EXPECT_DOUBLE_EQ(left[0].points[4].y, 10.0);
}

TEST(HdMapAdapter, JunctionInteriorBoundariesFalseDropsSegmentsInsideJunctionPolygon) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(JunctionRuleRow(false), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_b", 2, {{0, -10}, {0, -5}, {0, 0}, {0, 5}, {0, 10}}),
        LineMarker("right_boundary_a", 1, {{0, -10}, {0, 10}}),
        LineMarker("junction", 900, {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}, {-3, -3}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> left;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::LEFT_BOUNDARY) left.push_back(e);
    }
    ASSERT_EQ(left.size(), 2u) << "split by the polygon clip, same as ROAD_EDGE would be";
    std::sort(left.begin(), left.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].y < b.points[0].y;
              });
    EXPECT_DOUBLE_EQ(left[0].points[0].y, -10.0);
    EXPECT_NEAR(left[0].points[left[0].point_count - 1].y, -3.0, 1e-6);
    EXPECT_NEAR(left[1].points[0].y, 3.0, 1e-6);
    EXPECT_DOUBLE_EQ(left[1].points[left[1].point_count - 1].y, 10.0);
}

TEST(HdMapAdapter, ArcSnapExtendsCutPastFixedBackoffToTheCornerArcsFarEdge) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "one crossing -- head piece and tail piece";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -30.0);
    EXPECT_NEAR(lane_a[0].points[1].x, -2.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[1].y, 0.0, 1e-9);

    ASSERT_EQ(lane_a[1].point_count, 3u);
    EXPECT_NEAR(lane_a[1].points[0].x, 8.727406610312546, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].y, 5.929447639179834, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].x, 9.0, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].y, 20.0, 1e-9);
}

TEST(HdMapAdapter, ArcSnapDoesNotFireOnTheMeasuredWorstCaseGentleOpenRoadCurve) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
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

TEST(HdMapAdapter, ArcSnapExtendsBothWindowBoundariesToTheirOwnCornerArcs) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_NEAR(lane_a[0].points[0].x, -14.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[1].x, -9.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[1].y, 8.0, 1e-9);

    ASSERT_EQ(lane_a[1].point_count, 3u);
    EXPECT_NEAR(lane_a[1].points[0].x, 8.727406610312546, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].y, 5.929447639179834, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].x, 9.0, 1e-9);
    EXPECT_NEAR(lane_a[1].points[2].y, 20.0, 1e-9);
}

TEST(HdMapAdapter, ArcSnapAdjacentWindowsFromTwoCrossingsProduceOneContinuousCut) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u)
        << "the two corners' arcs meet in the middle -- no legit interior survives "
           "here any more, unlike the plain-straight version of this same gap above";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -50.0);
    EXPECT_NEAR(lane_a[0].points[1].x, -7.5, 1e-9);
    ASSERT_EQ(lane_a[1].point_count, 2u);
    EXPECT_NEAR(lane_a[1].points[0].x, 7.5, 1e-9);
    EXPECT_DOUBLE_EQ(lane_a[1].points[1].x, 50.0);
}

TEST(HdMapAdapter, PureArcCornerConnectorStillFramesItsOwnTwoRecordedEndpoints) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u) << "two minimal end-slivers, never zero pieces";
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    ASSERT_EQ(lane_a[0].point_count, 2u);
    EXPECT_NEAR(lane_a[0].points[0].x, 0.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[0].y, 0.0, 1e-9);

    ASSERT_EQ(lane_a[1].point_count, 2u);
    EXPECT_NEAR(lane_a[1].points[0].x, 4.242640687119285, 1e-9);
    EXPECT_NEAR(lane_a[1].points[0].y, 1.7573593128807143, 1e-9);
    EXPECT_NEAR(lane_a[1].points[1].x, 5.196152422706632, 1e-9);
    EXPECT_NEAR(lane_a[1].points[1].y, 2.999999999999999, 1e-9);
}

TEST(HdMapAdapter, TerminalArcStubIsRemovedAndArcSnapStaysSafeAtTheClamp) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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
        LineMarker("left_boundary_b", 2, {{-10.0, 6.978301740877824}, {20.0, 6.978301740877824}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u) << "the tail stub is gone -- only the head piece survives";

    ASSERT_EQ(lane_a[0].point_count, 3u);
    EXPECT_DOUBLE_EQ(lane_a[0].points[0].x, -30.0);
    EXPECT_NEAR(lane_a[0].points[2].x, 3.070552360820166, 1e-9);
    EXPECT_NEAR(lane_a[0].points[2].y, 0.2725933896874535, 1e-9);
}

TEST(HdMapAdapter, ArcSnapReachesTheArcsTrueStartEvenBeyondTheOldSearchMargin) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    arr.markers = {
        LineMarker("left_boundary_a", 792,
                   {{-46.980491978360725, 2.12625044165753},
                    {-46.771308183612035, -6.249399325630099},
                    {-46.78021262964802, -9.742296952272927},
                    {-46.547218708994116, -12.837935483912194},
                    {-46.4658864625989, -14.974389719737527},
                    {-46.0651262769608, -17.148459112710047},
                    {-44.95378149894339, -19.77721103359962},
                    {-42.79181672520813, -21.299520261110413},
                    {-38.17917117284436, -22.437475517352055},
                    {-30.297919317257183, -23.46468621338738}}),
        LineMarker("left_boundary_b", 685,
                   {{-40.35957131078806, -50.0}, {-40.35957131078806, 50.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 792u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 2u);
    std::sort(lane_a.begin(), lane_a.end(),
              [](const overlume::MapElement& a, const overlume::MapElement& b) {
                  return a.points[0].x < b.points[0].x;
              });

    const auto& head = lane_a[0];
    ASSERT_EQ(head.point_count, 5u);
    EXPECT_NEAR(head.points[0].x, -46.980491978360725, 1e-9) << "lane's own literal first vertex";
    EXPECT_NEAR(head.points[4].x, -46.4658864625989, 1e-6)
        << "vertex 4 -- the arc's own true start";
    EXPECT_NEAR(head.points[4].y, -14.974389719737527, 1e-6);

    const auto& tail = lane_a[1];
    ASSERT_EQ(tail.point_count, 3u);
    EXPECT_NEAR(tail.points[2].x, -30.297919317257183, 1e-9) << "lane's own literal last vertex";
    EXPECT_NEAR(tail.points[2].y, -23.46468621338738, 1e-9);
}

TEST(HdMapAdapter, RedundantArcTailTrimCutsBackToTheCornerArcsDepartureVertex) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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
        LineMarker("left_boundary_b", 2, {{9.3, 8.0}, {9.0, 20.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a, lane_b;
    for (const auto& e : out.map_elements) {
        if (e.kind != overlume::MapKind::ROAD_EDGE) continue;
        if (e.lane_id == 1u) lane_a.push_back(e);
        if (e.lane_id == 2u) lane_b.push_back(e);
    }

    ASSERT_EQ(lane_a.size(), 1u) << "no crossing exists -- the trim is the only cut on this piece";
    ASSERT_EQ(lane_a[0].point_count, 7u) << "v0..v6 kept -- the redundant tail (v7,v8) is gone";
    EXPECT_NEAR(lane_a[0].points[0].x, -5.0, 1e-9) << "lane's own literal first vertex, unmoved";
    EXPECT_NEAR(lane_a[0].points[0].y, 0.0, 1e-9);
    EXPECT_NEAR(lane_a[0].points[6].x, 8.727406610312546, 1e-9)
        << "v6 -- the arc's own true rejoin vertex, a real recorded point, never interpolated";
    EXPECT_NEAR(lane_a[0].points[6].y, 5.929447639179834, 1e-9);

    ASSERT_EQ(lane_b.size(), 1u) << "lane 2 is its own independently-promoted piece, untouched";
    ASSERT_EQ(lane_b[0].point_count, 2u);
}

TEST(HdMapAdapter, RedundantArcTailTrimDoesNotFireOnAnOpenElbowConnector) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u);
    ASSERT_EQ(lane_a[0].point_count, 9u)
        << "penultimate vertex ~1.5366 m from lane 2 (lane 232's own measured false-case "
           "distance) -- just past kRoadEdgeCoincidenceThresholdM, condition (b) fails, whole "
           "lane renders unmoved";
}

TEST(HdMapAdapter, RedundantArcTailTrimFiresOnTheWorstMeasuredTrueCase) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u);
    ASSERT_EQ(lane_a[0].point_count, 7u)
        << "penultimate vertex ~0.8885 m from lane 2 (lane 792's own measured worst true-case "
           "distance) -- just inside kRoadEdgeCoincidenceThresholdM, condition (b) passes, the "
           "redundant tail is trimmed";
}

TEST(HdMapAdapter, RedundantArcTailTrimDoesNotFireJustOutsideTheSharedNodeEpsilon) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_a;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 1u) lane_a.push_back(e);
    }
    ASSERT_EQ(lane_a.size(), 1u);
    ASSERT_EQ(lane_a[0].point_count, 9u)
        << "0.2 m gap at the tail's own far vertex is outside kSharedNodeEpsM -- no trim";
}

TEST(HdMapAdapter, NeighborArcDepartureSnapCutsBackAStraightEdgeOvershootingTheCorner) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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
        LineMarker("left_boundary_c", 3, {{-5.0, 0.0}, {30.0, 0.0}}),
        LineMarker("left_boundary_d", 4, {{15.0, -10.0}, {15.0, 10.0}}),
    };
    a.ingest(arr, 1.0);
    SceneAssembly out;
    a.fill(out);

    std::vector<overlume::MapElement> lane_a, lane_c;
    for (const auto& e : out.map_elements) {
        if (e.kind != overlume::MapKind::ROAD_EDGE) continue;
        if (e.lane_id == 1u) lane_a.push_back(e);
        if (e.lane_id == 3u) lane_c.push_back(e);
    }

    ASSERT_EQ(lane_a.size(), 1u) << "lane 1 crosses nothing in this scene -- untouched, whole";
    ASSERT_EQ(lane_a[0].point_count, 7u);

    ASSERT_EQ(lane_c.size(), 2u) << "one crossing on lane 3 -- head piece and tail piece";
    std::sort(lane_c.begin(), lane_c.end(),
              [](const overlume::MapElement& x, const overlume::MapElement& y) {
                  return x.points[0].x < y.points[0].x;
              });

    ASSERT_EQ(lane_c[0].point_count, 2u);
    EXPECT_NEAR(lane_c[0].points[0].x, -5.0, 1e-9) << "lane 3's own literal start, unmoved";
    EXPECT_NEAR(lane_c[0].points[1].x, 3.070552360820166, 1e-9)
        << "pulled back to v2's own station -- not left at the old fixed-backoff x=13";
    EXPECT_NEAR(lane_c[0].points[1].y, 0.0, 1e-9);

    ASSERT_EQ(lane_c[1].point_count, 2u);
    EXPECT_NEAR(lane_c[1].points[0].x, 17.0, 1e-9);
    EXPECT_NEAR(lane_c[1].points[1].x, 30.0, 1e-9);
}

TEST(HdMapAdapter, NeighborArcDepartureSnapDoesNotFireWithoutASharedNode) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

    std::vector<overlume::MapElement> lane_c;
    for (const auto& e : out.map_elements) {
        if (e.kind == overlume::MapKind::ROAD_EDGE && e.lane_id == 3u) lane_c.push_back(e);
    }
    ASSERT_EQ(lane_c.size(), 2u);
    std::sort(lane_c.begin(), lane_c.end(),
              [](const overlume::MapElement& x, const overlume::MapElement& y) {
                  return x.points[0].x < y.points[0].x;
              });
    ASSERT_EQ(lane_c[0].point_count, 2u);
    EXPECT_NEAR(lane_c[0].points[1].x, 13.0, 1e-9)
        << "0.2 m outside kSharedNodeEpsM -- no neighbour found, old fixed-backoff point unmoved "
           "(2.0 m back off the x=15 crossing, same physical point regardless of this lane's own "
           "start offset)";
}

TEST(HdMapAdapter, OneMarkerCountsAsOneIngestedMarkerForStats) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);
    a.ingest(StraightTenMeterMarker("centerline_x"), 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);

    EXPECT_EQ(a.stats().msgs, 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
    EXPECT_EQ(a.stats().dropped_by_rule, 0u);
    EXPECT_EQ(a.stats().dropped_no_tf, 0u);
}

TEST(HdMapAdapter, MarkerPoseComposesRotationBeforeTranslation) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "centerline_x";
    m.id = 1;
    m.type = 4;
    m.action = 0;
    m.pose.position.x = 10.0;
    m.pose.position.y = 20.0;
    constexpr double kQuarterTurn = 0.70710678118654752440;
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

TEST(HdMapAdapter, IdentityMarkerPoseIsByteIdenticalToRawPoints) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);
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

TEST(HdMapAdapter, NanMarkerPoseIsDroppedAsMalformedNotAppliedRaw) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);

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

TEST(HdMapAdapter, ZeroQuaternionPoseIsTreatedAsIdentityRotationNotNan) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);
    auto arr = StraightTenMeterMarker("centerline_x");
    arr.markers[0].pose.position.y = 7.0;
    arr.markers[0].pose.orientation.w = 0.0;
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
    ASSERT_EQ(out.map_elements[0].point_count, 2u);
    EXPECT_NEAR(out.map_elements[0].points[0].y, 7.0, 1e-9);
    EXPECT_NEAR(out.map_elements[0].points[1].x, 10.0, 1e-9);
}

TEST(HdMapAdapter, NonZeroZPointsAreFlattenedToTheMapPlane) {
    TfFixture kTf;
    overlume::ros::HdMapAdapter a(MapRuleRow(), kTf.tf);
    auto arr = StraightTenMeterMarker("centerline_x");
    for (auto& p : arr.markers[0].points) p.z = 3.0;
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[0].z, 0.0);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[1].z, 0.0);
}

TEST(HdMapAdapter, FlattenZOffPreservesPublisherZ) {
    TfFixture kTf;
    overlume::ros::FrameTransformer tf3d(kTf.buffer, "map", false);
    overlume::ros::HdMapAdapter a(MapRuleRow(), tf3d);
    auto arr = StraightTenMeterMarker("centerline_x");
    for (auto& p : arr.markers[0].points) p.z = 3.0;
    a.ingest(arr, 1.0);

    SceneAssembly out;
    a.fill(out);
    ASSERT_EQ(out.map_elements.size(), 1u);
    EXPECT_DOUBLE_EQ(out.map_elements[0].points[0].z, 3.0);
}
