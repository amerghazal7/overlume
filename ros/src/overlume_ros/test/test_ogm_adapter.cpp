// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/ogm.hpp"

#include <memory>
#include <string>

#include <gtest/gtest.h>
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

const overlume::GroundGridLayer* OnlyGrid(const SceneAssembly& asm_) {
    return asm_.grids.size() == 1 ? &asm_.grids[0] : nullptr;
}

}

TEST(OgmAdapter, FullGridPopulatesLayerGeometryAndCells) {
    auto msg = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    ASSERT_EQ(msg.info.width, 5u);
    ASSERT_EQ(msg.info.height, 2u);
    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.x, 1.0);
    EXPECT_DOUBLE_EQ(g->origin.y, 2.0);
    EXPECT_DOUBLE_EQ(g->origin.z, 0.0);
    EXPECT_DOUBLE_EQ(g->resolution_m, 0.5);
    EXPECT_EQ(g->width_cells, 5u);
    EXPECT_EQ(g->height_cells, 2u);
    ASSERT_NE(g->cells, nullptr);
    for (uint32_t i = 5; i < 10; ++i) EXPECT_EQ(g->cells[i], 0u) << "cell " << i;
    EXPECT_EQ(a.stats().msgs, 1u);
}

TEST(OgmAdapter, OriginZFlattenedToZeroByDefault) {
    auto msg = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    msg.info.origin.position.z = 2.5;
    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.z, 0.0)
        << "flatten_z (default true) must zero the stored grid origin z";
}

TEST(OgmAdapter, UnknownCellsBecomeTheSentinelNotTwoFiftyFive) {
    auto msg = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_EQ(g->cells[0], overlume::ros::kUnknownCell) << "-1 (unknown) must become the sentinel";
    EXPECT_EQ(g->cells[1], 0u);
    EXPECT_EQ(g->cells[2], 50u);
    EXPECT_EQ(g->cells[3], 100u);
    EXPECT_EQ(g->cells[4], overlume::ros::kUnknownCell)
        << "127 (illegal, out of 0..100) must ALSO become the sentinel, not survive as 127";
    EXPECT_EQ(a.stats().dropped_malformed, 1u)
        << "exactly one malformed cell (127) in this message -- one dropped_malformed, not five";
}

TEST(OgmAdapter, PartialUpdatePatchesInPlaceWithoutResizing) {
    auto grid = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    auto patch = overlume::ros::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    ASSERT_EQ(patch.x, 1);
    ASSERT_EQ(patch.y, 1);
    ASSERT_EQ(patch.width, 2u);
    ASSERT_EQ(patch.height, 1u);

    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);
    a.ingest(grid, 1.0);
    a.ingest_update(patch, 1.1);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    ASSERT_EQ(g->width_cells, 5u);
    ASSERT_EQ(g->height_cells, 2u);

    EXPECT_EQ(g->cells[0], overlume::ros::kUnknownCell);
    EXPECT_EQ(g->cells[1], 0u);
    EXPECT_EQ(g->cells[2], 50u);
    EXPECT_EQ(g->cells[3], 100u);
    EXPECT_EQ(g->cells[4], overlume::ros::kUnknownCell);

    EXPECT_EQ(g->cells[5], 0u);
    EXPECT_EQ(g->cells[6], 42u);
    EXPECT_EQ(g->cells[7], overlume::ros::kUnknownCell)
        << "-1 in the PATCH must also convert to the sentinel";
    EXPECT_EQ(g->cells[8], 0u);
    EXPECT_EQ(g->cells[9], 0u);
}

TEST(OgmAdapter, UpdateBeforeAnyFullGridIsDroppedAndCounted) {
    auto patch = overlume::ros::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);

    a.ingest_update(patch, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.grids.empty()) << "fill() must emit nothing -- there is still no base grid";
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
    EXPECT_EQ(a.stats().msgs, 1u);
}

TEST(OgmAdapter, OutOfBoundsUpdateRectIsRejected) {
    auto grid = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    auto patch = overlume::ros::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    patch.x = 4;
    patch.width = 2;
    patch.data = {7, 7};

    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);
    a.ingest(grid, 1.0);
    a.ingest_update(patch, 1.1);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_EQ(g->cells[0], overlume::ros::kUnknownCell);
    EXPECT_EQ(g->cells[1], 0u);
    EXPECT_EQ(g->cells[2], 50u);
    EXPECT_EQ(g->cells[3], 100u);
    EXPECT_EQ(g->cells[4], overlume::ros::kUnknownCell);
    for (uint32_t i = 5; i < 10; ++i) EXPECT_EQ(g->cells[i], 0u) << "cell " << i;
    EXPECT_EQ(a.stats().dropped_malformed, 2u);
}

TEST(OgmAdapter, RoleSelectsLayerKind) {
    auto grid = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    TfFixture kTf;

    overlume::ros::OgmAdapter dynamic(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                      kTf.tf);
    dynamic.ingest(grid, 1.0);
    SceneAssembly dynAsm;
    dynamic.fill(dynAsm);
    ASSERT_EQ(dynAsm.grids.size(), 1u);
    EXPECT_EQ(dynAsm.grids[0].kind, 0u) << "role dynamic_ogm must select kind 0";

    overlume::ros::OgmAdapter gradient(
        overlume::ros::testing::urban_row("/perception/gradient_ogm"), kTf.tf);
    gradient.ingest(grid, 1.0);
    SceneAssembly gradAsm;
    gradient.fill(gradAsm);
    ASSERT_EQ(gradAsm.grids.size(), 1u);
    EXPECT_EQ(gradAsm.grids[0].kind, 1u) << "role gradient_ogm must select kind 1";
}

TEST(OgmAdapter, OneRowYieldsTwoSubscriptionsAndOneAdapter) {
    const auto specs = overlume::ros::subscriptions_for(
        overlume::ros::testing::urban_row("/perception/dynamic_ogm"));
    ASSERT_EQ(specs.size(), 2u);
    EXPECT_EQ(specs[0].topic, "/perception/dynamic_ogm");
    EXPECT_EQ(specs[0].type, "nav_msgs/msg/OccupancyGrid");
    EXPECT_EQ(specs[1].topic, "/perception/dynamic_ogm_updates");
    EXPECT_EQ(specs[1].type, "map_msgs/msg/OccupancyGridUpdate");

    auto full_grid = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    auto patch = overlume::ros::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);
    a.ingest(full_grid, 1.0);
    a.ingest_update(patch, 1.1);

    SceneAssembly asm_;
    a.fill(asm_);
    ASSERT_EQ(asm_.grids.size(), 1u) << "one adapter, one GroundGridLayer, fed by both overloads";
    EXPECT_EQ(a.stats().msgs, 2u) << "stats().msgs counts BOTH overloads' accepted messages";
}

TEST(OgmAdapter, CostmapEncodingDecodesNav2CostsInsteadOfDroppingThem) {
    auto msg = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    msg.data = {0,
                1,
                static_cast<int8_t>(127),
                static_cast<int8_t>(-3),
                static_cast<int8_t>(-2),
                static_cast<int8_t>(-1)};
    msg.info.width = 6;
    msg.info.height = 1;
    TfFixture kTf;
    auto row = overlume::ros::testing::urban_row("/perception/dynamic_ogm");
    row.encoding = "costmap";
    overlume::ros::OgmAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_EQ(g->cells[0], 0u) << "cost 0 is free";
    EXPECT_EQ(g->cells[1], 0u) << "cost 1 rounds to ~0";
    EXPECT_EQ(g->cells[2], 50u) << "cost 127 is half-way";
    EXPECT_EQ(g->cells[3], 100u) << "cost 253 (inscribed, int8 -3) renders as occupied";
    EXPECT_EQ(g->cells[4], 100u) << "cost 254 (lethal, int8 -2) renders as occupied";
    EXPECT_EQ(g->cells[5], overlume::ros::kUnknownCell) << "cost 255 is no-information";
    EXPECT_EQ(a.stats().dropped_malformed, 0u) << "costmap values are never malformed";
}

TEST(OgmAdapter, DefaultOccupancyEncodingStillRejectsCostmapValues) {
    auto msg = overlume::ros::testing::load_occupancy_grid("ogm_synthetic.yaml");
    msg.data = {static_cast<int8_t>(-2)};
    msg.info.width = 1;
    msg.info.height = 1;
    TfFixture kTf;
    overlume::ros::OgmAdapter a(overlume::ros::testing::urban_row("/perception/dynamic_ogm"),
                                kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_EQ(g->cells[0], overlume::ros::kUnknownCell);
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
}
