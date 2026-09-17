/** @file test_ogm_adapter.cpp
 *  @brief OgmAdapter tests.
 *
 *  Zero OccupancyGrid topics exist in the recorded bag or stack -- every
 *  fixture here is SYNTHETIC (see fixtures/ogm_synthetic.yaml /
 *  ogm_update_synthetic.yaml's own header comments).
 */
#include "overlume_ros/adapters/ogm.hpp"

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using overlume::ros::FrameTransformer;
using overlume::ros::SceneAssembly;

namespace
{

// Hand-built tf2_ros::Buffer + FrameTransformer -- identical fixture style
// to test_path_adapter.cpp/test_hd_map_adapter.cpp. An empty buffer is
// enough: every fixture here is already in the "map" frame
// (FrameTransformer's identity shortcut never touches it).
struct TfFixture
{
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

const overlume::GroundGridLayer* OnlyGrid(const SceneAssembly& asm_)
{
    return asm_.grids.size() == 1 ? &asm_.grids[0] : nullptr;
}

}  // namespace

// ── Step 1: full grid populates geometry + converted cells ─────────────────

TEST(OgmAdapter, FullGridPopulatesLayerGeometryAndCells)
{
    auto msg = overlume_node::testing::load_occupancy_grid("ogm_synthetic.yaml");
    ASSERT_EQ(msg.info.width, 5u);
    ASSERT_EQ(msg.info.height, 2u);
    TfFixture kTf;
    overlume_node::OgmAdapter a(overlume_node::testing::urban_row("/perception/dynamic_ogm"), kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);

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
    // Row 1 (filler): every cell converts 0 -> 0.
    for (uint32_t i = 5; i < 10; ++i) EXPECT_EQ(g->cells[i], 0u) << "cell " << i;
    EXPECT_EQ(a.stats().msgs, 1u);
}

// ── flatten_z: origin z zeroed by default ───────────────────────────────────

TEST(OgmAdapter, OriginZFlattenedToZeroByDefault)
{
    auto msg = overlume_node::testing::load_occupancy_grid("ogm_synthetic.yaml");
    msg.info.origin.position.z = 2.5;  // real altitude a live TF/publisher could carry
    TfFixture kTf;  // FrameTransformer defaults flatten_z=true, same as the node's own default
    overlume_node::OgmAdapter a(overlume_node::testing::urban_row("/perception/dynamic_ogm"), kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.z, 0.0)
        << "flatten_z (default true) must zero the stored grid origin z";
}

// ── Step 1: -1/0/50/100/127 -> 255/0/50/100/255, one dropped_malformed ──────

TEST(OgmAdapter, UnknownCellsBecomeTheSentinelNotTwoFiftyFive)
{
    auto msg = overlume_node::testing::load_occupancy_grid("ogm_synthetic.yaml");
    TfFixture kTf;
    overlume_node::OgmAdapter a(overlume_node::testing::urban_row("/perception/dynamic_ogm"), kTf.tf);
    a.ingest(msg, /*sim_time_sec=*/1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    // Row 0: -1, 0, 50, 100, 127 (wire) -> 255, 0, 50, 100, 255 (converted).
    EXPECT_EQ(g->cells[0], overlume_node::kUnknownCell) << "-1 (unknown) must become the sentinel";
    EXPECT_EQ(g->cells[1], 0u);
    EXPECT_EQ(g->cells[2], 50u);
    EXPECT_EQ(g->cells[3], 100u);
    EXPECT_EQ(g->cells[4], overlume_node::kUnknownCell)
        << "127 (illegal, out of 0..100) must ALSO become the sentinel, not survive as 127";
    EXPECT_EQ(a.stats().dropped_malformed, 1u)
        << "exactly one malformed cell (127) in this message -- one dropped_malformed, not five";
}

// ── Step 2: a patch rewrites exactly its own sub-rectangle ─────────────────

TEST(OgmAdapter, PartialUpdatePatchesInPlaceWithoutResizing)
{
    auto grid = overlume_node::testing::load_occupancy_grid("ogm_synthetic.yaml");
    auto patch = overlume_node::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    ASSERT_EQ(patch.x, 1);
    ASSERT_EQ(patch.y, 1);
    ASSERT_EQ(patch.width, 2u);
    ASSERT_EQ(patch.height, 1u);

    TfFixture kTf;
    overlume_node::OgmAdapter a(overlume_node::testing::urban_row("/perception/dynamic_ogm"), kTf.tf);
    a.ingest(grid, 1.0);
    a.ingest_update(patch, 1.1);  // SAME object, not a second instance

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    ASSERT_EQ(g->width_cells, 5u);   // never resized by a patch
    ASSERT_EQ(g->height_cells, 2u);

    // Row 0 -- entirely outside the patch rect -- byte-identical to the
    // full-grid ingest.
    EXPECT_EQ(g->cells[0], overlume_node::kUnknownCell);
    EXPECT_EQ(g->cells[1], 0u);
    EXPECT_EQ(g->cells[2], 50u);
    EXPECT_EQ(g->cells[3], 100u);
    EXPECT_EQ(g->cells[4], overlume_node::kUnknownCell);

    // Row 1: index (1,1)=6 and (2,1)=7 patched to [42, -1] -> [42, 255];
    // everything else in row 1 stays the original filler 0.
    EXPECT_EQ(g->cells[5], 0u);
    EXPECT_EQ(g->cells[6], 42u);
    EXPECT_EQ(g->cells[7], overlume_node::kUnknownCell) << "-1 in the PATCH must also convert to the sentinel";
    EXPECT_EQ(g->cells[8], 0u);
    EXPECT_EQ(g->cells[9], 0u);
}

// ── Step 2: an _updates message with no base grid yet is a no-op ───────────

TEST(OgmAdapter, UpdateBeforeAnyFullGridIsDroppedAndCounted)
{
    auto patch = overlume_node::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    TfFixture kTf;
    overlume_node::OgmAdapter a(overlume_node::testing::urban_row("/perception/dynamic_ogm"), kTf.tf);

    a.ingest_update(patch, 1.0);  // no ingest() ever called -- must not allocate/crash

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.grids.empty()) << "fill() must emit nothing -- there is still no base grid";
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
    EXPECT_EQ(a.stats().msgs, 1u);
}

// ── Step 2: an out-of-bounds update rect is rejected, base untouched ───────

TEST(OgmAdapter, OutOfBoundsUpdateRectIsRejected)
{
    auto grid = overlume_node::testing::load_occupancy_grid("ogm_synthetic.yaml");
    auto patch = overlume_node::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    // 5-wide base grid; x=4 + width=2 = 6 > 5 -- overruns by one column.
    patch.x = 4;
    patch.width = 2;
    patch.data = {7, 7};

    TfFixture kTf;
    overlume_node::OgmAdapter a(overlume_node::testing::urban_row("/perception/dynamic_ogm"), kTf.tf);
    a.ingest(grid, 1.0);
    a.ingest_update(patch, 1.1);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::GroundGridLayer* g = OnlyGrid(asm_);
    ASSERT_NE(g, nullptr);
    // Base untouched -- row 0 (where the rejected rect would have landed,
    // y=1 actually, but the WHOLE patch is rejected, so check both rows)
    // stays exactly the full-grid ingest's converted values.
    EXPECT_EQ(g->cells[0], overlume_node::kUnknownCell);
    EXPECT_EQ(g->cells[1], 0u);
    EXPECT_EQ(g->cells[2], 50u);
    EXPECT_EQ(g->cells[3], 100u);
    EXPECT_EQ(g->cells[4], overlume_node::kUnknownCell);
    for (uint32_t i = 5; i < 10; ++i) EXPECT_EQ(g->cells[i], 0u) << "cell " << i;
    // 2 total: the base ingest()'s own 127-cell (ogm_synthetic.yaml's row 0)
    // PLUS this out-of-bounds ingest_update() -- the rejected rect adds
    // exactly one more, on top of whatever the base grid already carried.
    EXPECT_EQ(a.stats().dropped_malformed, 2u);
}

// ── Step 2: role -> kind ─────────────────────────────────────────────────

TEST(OgmAdapter, RoleSelectsLayerKind)
{
    auto grid = overlume_node::testing::load_occupancy_grid("ogm_synthetic.yaml");
    TfFixture kTf;

    overlume_node::OgmAdapter dynamic(overlume_node::testing::urban_row("/perception/dynamic_ogm"),
                                    kTf.tf);
    dynamic.ingest(grid, 1.0);
    SceneAssembly dynAsm;
    dynamic.fill(dynAsm);
    ASSERT_EQ(dynAsm.grids.size(), 1u);
    EXPECT_EQ(dynAsm.grids[0].kind, 0u) << "role dynamic_ogm must select kind 0";

    overlume_node::OgmAdapter gradient(overlume_node::testing::urban_row("/perception/gradient_ogm"),
                                     kTf.tf);
    gradient.ingest(grid, 1.0);
    SceneAssembly gradAsm;
    gradient.fill(gradAsm);
    ASSERT_EQ(gradAsm.grids.size(), 1u);
    EXPECT_EQ(gradAsm.grids[0].kind, 1u) << "role gradient_ogm must select kind 1";
}

// ── The wiring test -- subscriptions_for(), not the adapter ────────────────

TEST(OgmAdapter, OneRowYieldsTwoSubscriptionsAndOneAdapter)
{
    const auto specs =
        overlume_node::subscriptions_for(overlume_node::testing::urban_row("/perception/dynamic_ogm"));
    ASSERT_EQ(specs.size(), 2u);
    EXPECT_EQ(specs[0].topic, "/perception/dynamic_ogm");
    EXPECT_EQ(specs[0].type, "nav_msgs/msg/OccupancyGrid");
    EXPECT_EQ(specs[1].topic, "/perception/dynamic_ogm_updates");
    EXPECT_EQ(specs[1].type, "map_msgs/msg/OccupancyGridUpdate");

    // ...and ONE adapter consumes both, via two ingest overloads -- the same
    // object, not a second instance.
    auto full_grid = overlume_node::testing::load_occupancy_grid("ogm_synthetic.yaml");
    auto patch = overlume_node::testing::load_occupancy_grid_update("ogm_update_synthetic.yaml");
    TfFixture kTf;
    overlume_node::OgmAdapter a(overlume_node::testing::urban_row("/perception/dynamic_ogm"), kTf.tf);
    a.ingest(full_grid, 1.0);
    a.ingest_update(patch, 1.1);

    SceneAssembly asm_;
    a.fill(asm_);
    ASSERT_EQ(asm_.grids.size(), 1u) << "one adapter, one GroundGridLayer, fed by both overloads";
    EXPECT_EQ(a.stats().msgs, 2u) << "stats().msgs counts BOTH overloads' accepted messages";
}
