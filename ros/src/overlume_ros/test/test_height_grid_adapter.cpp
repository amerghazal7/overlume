// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/height_grid.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using overlume::ros::decode_height_cell;
using overlume::ros::FrameTransformer;
using overlume::ros::HeightGridAdapter;
using overlume::ros::ProfileRow;
using overlume::ros::SceneAssembly;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

ProfileRow HeightRow(const std::string& encoding = "height_linear") {
    ProfileRow r;
    r.topic = "/debug_ogm_2";
    r.type = "nav_msgs/msg/OccupancyGrid";
    r.adapter = "height_grid";
    r.role = "terrain";
    r.encoding = encoding;
    r.height_min_m = -2.0;
    r.height_max_m = 3.0;
    return r;
}

const overlume::HeightGridLayer* OnlyLayer(const SceneAssembly& asm_) {
    return asm_.height_grids.size() == 1 ? &asm_.height_grids[0] : nullptr;
}

}

TEST(DecodeHeightCell, LinearMapsZeroAndHundredToTheWindowEnds) {
    bool malformed = false;
    EXPECT_FLOAT_EQ(decode_height_cell(0, false, -2.0, 3.0, malformed), -2.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(100, false, -2.0, 3.0, malformed), 3.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(50, false, -2.0, 3.0, malformed), 0.5f);
    EXPECT_FALSE(malformed);
}

TEST(DecodeHeightCell, NormalizedMapsOneAndHundredToTheWindowEndsAndClampsZero) {
    bool malformed = false;
    EXPECT_FLOAT_EQ(decode_height_cell(1, true, -2.0, 3.0, malformed), -2.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(100, true, -2.0, 3.0, malformed), 3.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(0, true, -2.0, 3.0, malformed), -2.0f)
        << "(0-1)/99 is negative and must clamp to the window minimum";
    EXPECT_FLOAT_EQ(decode_height_cell(50, true, -2.0, 3.0, malformed),
                    static_cast<float>(-2.0 + 49.0 / 99.0 * 5.0));
    EXPECT_FALSE(malformed);
}

TEST(DecodeHeightCell, MinusOneIsUnknownNanAndNotMalformed) {
    for (const bool normalized : {false, true}) {
        bool malformed = false;
        EXPECT_TRUE(std::isnan(decode_height_cell(-1, normalized, -2.0, 3.0, malformed)));
        EXPECT_FALSE(malformed) << "-1 is the legal unknown marker";
    }
}

TEST(DecodeHeightCell, OutOfRangeValuesAreNanAndFlagMalformed) {
    for (const bool normalized : {false, true}) {
        for (const int v : {101, 127, -2, -128}) {
            bool malformed = false;
            EXPECT_TRUE(std::isnan(
                decode_height_cell(static_cast<int8_t>(v), normalized, -2.0, 3.0, malformed)))
                << v;
            EXPECT_TRUE(malformed) << v;
        }
    }
}

TEST(HeightGridAdapter, LinearFixturePopulatesLayerGeometryAndHeights) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    ASSERT_EQ(msg.info.width, 5u);
    ASSERT_EQ(msg.info.height, 2u);
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.x, 1.0);
    EXPECT_DOUBLE_EQ(g->origin.y, 2.0);
    EXPECT_DOUBLE_EQ(g->origin.z, 0.0) << "flatten_z (default true) zeroes the origin z";
    EXPECT_DOUBLE_EQ(g->yaw_rad, 0.0);
    EXPECT_NEAR(g->resolution_m, 0.2, 1e-6);
    EXPECT_EQ(g->width_cells, 5u);
    EXPECT_EQ(g->height_cells, 2u);
    EXPECT_DOUBLE_EQ(g->last_update_sec, 1.0);
    ASSERT_NE(g->heights_m, nullptr);

    EXPECT_TRUE(std::isnan(g->heights_m[0])) << "-1 is unknown";
    EXPECT_FLOAT_EQ(g->heights_m[1], -2.0f) << "v=0";
    EXPECT_FLOAT_EQ(g->heights_m[2], 0.5f) << "v=50";
    EXPECT_FLOAT_EQ(g->heights_m[3], 3.0f) << "v=100";
    EXPECT_TRUE(std::isnan(g->heights_m[4])) << "127 is illegal";
    EXPECT_FLOAT_EQ(g->heights_m[5], -1.95f) << "v=1";
    EXPECT_FLOAT_EQ(g->heights_m[6], 2.95f) << "v=99";
    EXPECT_FLOAT_EQ(g->heights_m[7], -0.75f) << "v=25";
    EXPECT_FLOAT_EQ(g->heights_m[8], 1.75f) << "v=75";
    EXPECT_TRUE(std::isnan(g->heights_m[9]));

    EXPECT_EQ(a.stats().msgs, 1u);
    EXPECT_DOUBLE_EQ(a.stats().last_msg_sec, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u)
        << "one illegal cell (127) -- counted once per message, and the message is still applied";
}

TEST(HeightGridAdapter, NormalizedEncodingUsesTheOneToHundredScale) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    TfFixture kTf;
    HeightGridAdapter a(HeightRow("height_normalized"), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_FLOAT_EQ(g->heights_m[1], -2.0f) << "v=0 clamps to min";
    EXPECT_FLOAT_EQ(g->heights_m[3], 3.0f) << "v=100";
    EXPECT_FLOAT_EQ(g->heights_m[5], -2.0f) << "v=1 is the window minimum";
    EXPECT_FLOAT_EQ(g->heights_m[2], static_cast<float>(-2.0 + 49.0 / 99.0 * 5.0)) << "v=50";
    EXPECT_TRUE(std::isnan(g->heights_m[0]));
}

TEST(HeightGridAdapter, SizeMismatchAndZeroDimsAreDroppedAndCounted) {
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);

    auto shortMsg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    shortMsg.data.pop_back();
    a.ingest(shortMsg, 1.0);

    auto zeroMsg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    zeroMsg.info.width = 0;
    zeroMsg.data.clear();
    a.ingest(zeroMsg, 1.1);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty()) << "nothing was accepted, so fill() emits nothing";
    EXPECT_EQ(a.stats().msgs, 2u);
    EXPECT_EQ(a.stats().dropped_malformed, 2u);
}

TEST(HeightGridAdapter, MissingTransformIsDroppedAsNoTf) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.header.frame_id = "frame_nobody_publishes";
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty());
    EXPECT_EQ(a.stats().dropped_no_tf, 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HeightGridAdapter, NonFiniteOriginIsDroppedAsMalformed) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.info.origin.position.x = std::numeric_limits<double>::quiet_NaN();
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty());
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
}

TEST(HeightGridAdapter, FlattenZOffKeepsTheOriginZ) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.info.origin.position.z = 2.5;
    TfFixture kTf;
    FrameTransformer keepZ(kTf.buffer, "map", false);
    HeightGridAdapter a(HeightRow(), keepZ);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.z, 2.5);
}

TEST(HeightGridAdapter, FlattenZOnZeroesAnOriginZThatIsNotZero) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.info.origin.position.z = 2.5;
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.z, 0.0);
}

TEST(HeightGridAdapter, BaseLinkGridTakesTheVehicleYawIntoTheMapFrame) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.header.frame_id = "base_link";
    msg.info.origin.position.x = -25.0;
    msg.info.origin.position.y = -25.0;
    msg.info.origin.orientation.w = 1.0;
    TfFixture kTf;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = 10.0;
    t.transform.rotation.z = std::sin(M_PI / 4.0);
    t.transform.rotation.w = std::cos(M_PI / 4.0);
    kTf.buffer.setTransform(t, "test", true);

    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_NEAR(g->yaw_rad, M_PI / 2.0, 1e-9) << "vehicle yaw must reach the layer";
    EXPECT_NEAR(g->origin.x, 35.0, 1e-9);
    EXPECT_NEAR(g->origin.y, -25.0, 1e-9);
}

TEST(HeightGridAdapter, FrameIdOverrideReplacesTheHeaderFrame) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.header.frame_id = "frame_nobody_publishes";
    TfFixture kTf;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "seyond";
    t.transform.translation.x = 3.0;
    t.transform.translation.y = 4.0;
    t.transform.rotation.w = 1.0;
    kTf.buffer.setTransform(t, "test", true);

    auto row = HeightRow();
    row.frame_id = "seyond";
    HeightGridAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr) << "the override frame must be looked up instead of the header's";
    EXPECT_NEAR(g->origin.x, 4.0, 1e-9);
    EXPECT_NEAR(g->origin.y, 6.0, 1e-9);
    EXPECT_EQ(a.stats().dropped_no_tf, 0u);
}

TEST(HeightGridAdapter, LaterMessageReplacesTheEarlierOneAndFillPointsAtOwnedHeights) {
    auto first = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    auto second = first;
    second.data.assign(10, 100);
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(first, 1.0);
    a.ingest(second, 2.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr) << "one adapter, one layer";
    EXPECT_DOUBLE_EQ(g->last_update_sec, 2.0);
    for (uint32_t i = 0; i < 10; ++i) EXPECT_FLOAT_EQ(g->heights_m[i], 3.0f) << i;
    EXPECT_EQ(a.stats().msgs, 2u);

    SceneAssembly again;
    a.fill(again);
    ASSERT_EQ(again.height_grids.size(), 1u);
    EXPECT_EQ(again.height_grids[0].heights_m, g->heights_m)
        << "fill() must point at the adapter's own buffer, not copy per call";
}

TEST(HeightGridAdapter, FillEmitsNothingBeforeTheFirstMessage) {
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty());
}

TEST(HeightGridAdapter, MarkStaleTickCountsDroppedStale) {
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.mark_stale_tick();
    a.mark_stale_tick();
    EXPECT_EQ(a.stats().dropped_stale, 2u);
}

TEST(HeightGridAdapter, OneRowYieldsExactlyOneSubscription) {
    const auto specs = overlume::ros::subscriptions_for(HeightRow());
    ASSERT_EQ(specs.size(), 1u);
    EXPECT_EQ(specs[0].topic, "/debug_ogm_2");
    EXPECT_EQ(specs[0].type, "nav_msgs/msg/OccupancyGrid");
}
