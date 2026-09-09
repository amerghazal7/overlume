/** @file test_scene_assembly.cpp
 *  @brief SceneAssembly tests.
 */
#include "micropilot_visualization_node/scene_assembly.hpp"

#include <gtest/gtest.h>

using micropilot::visualization_app::apply_layer_gates;
using micropilot::visualization_app::LayerFlags;
using micropilot::visualization_app::SceneAssembly;

namespace
{

mpviz::MapElement MakeElement()
{
    mpviz::MapElement e{};
    e.points = nullptr;
    e.point_count = 0;
    e.is_polygon = 0;
    return e;
}

}  // namespace

TEST(SceneAssembly, TwoAdaptersOnOneCategoryBothSurvive)
{
    // the finding that motivated SceneAssembly: fill() two hd_map adapters
    // (3 + 5 elements) -> map_element_count == 8, not 5.
    SceneAssembly asm_;
    for (int i = 0; i < 3; ++i) asm_.map_elements.push_back(MakeElement());  // adapter A's fill()
    for (int i = 0; i < 5; ++i) asm_.map_elements.push_back(MakeElement());  // adapter B's fill()

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.map_element_count, 8u);
    EXPECT_EQ(scene.map_elements, asm_.map_elements.data());
}

TEST(SceneAssembly, ClearBetweenTicksDoesNotAccumulate)
{
    SceneAssembly asm_;
    for (int i = 0; i < 3; ++i) asm_.map_elements.push_back(MakeElement());
    mpviz::SceneGraph scene1{};
    asm_.point_at(scene1);
    EXPECT_EQ(scene1.map_element_count, 3u);

    asm_.clear();
    for (int i = 0; i < 2; ++i) asm_.map_elements.push_back(MakeElement());
    mpviz::SceneGraph scene2{};
    asm_.point_at(scene2);
    EXPECT_EQ(scene2.map_element_count, 2u);  // not 3+2 == 5
}

TEST(SceneAssembly, ClearEmptiesEveryCategoryNotJustMapElements)
{
    SceneAssembly asm_;
    asm_.map_elements.push_back(MakeElement());
    asm_.objects.push_back(mpviz::TrackedObject{});
    asm_.paths.push_back(mpviz::PathRibbon{});
    asm_.grids.push_back(mpviz::GroundGridLayer{});
    asm_.alerts.push_back(mpviz::AlertPolygon{});
    asm_.markers.push_back(mpviz::GenericMarker{});
    asm_.point_clouds.push_back(mpviz::PointCloud{});

    asm_.clear();

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.map_element_count, 0u);
    EXPECT_EQ(scene.object_count, 0u);
    EXPECT_EQ(scene.path_count, 0u);
    EXPECT_EQ(scene.grid_count, 0u);
    EXPECT_EQ(scene.alert_count, 0u);
    EXPECT_EQ(scene.marker_count, 0u);
    EXPECT_EQ(scene.point_cloud_count, 0u);
}

TEST(SceneAssembly, PointCloudRowAppendsIntoScenePointClouds)
{
    // Same shape as TwoAdaptersOnOneCategoryBothSurvive above, for the new
    // category (Task 6 / VM-035).
    SceneAssembly asm_;
    asm_.point_clouds.push_back(mpviz::PointCloud{});
    asm_.point_clouds.push_back(mpviz::PointCloud{});

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 2u);
    EXPECT_EQ(scene.point_clouds, asm_.point_clouds.data());
}

TEST(SceneAssembly, ApplyLayerGatesPointCloudsOffZeroesOnlyPointClouds)
{
    // The finding this covers: visualization_node.cpp's
    // `if (!layer_point_clouds_) scene_asm_.point_clouds.clear();` gate had
    // no test exercising it directly. apply_layer_gates() is the exact
    // function that line now calls, with the same LayerFlags shape.
    SceneAssembly asm_;
    asm_.point_clouds.push_back(mpviz::PointCloud{});
    asm_.objects.push_back(mpviz::TrackedObject{});  // another category, must survive

    LayerFlags flags;  // all true by default
    flags.point_clouds = false;
    apply_layer_gates(asm_, flags);

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 0u);
    EXPECT_EQ(scene.object_count, 1u);
}

TEST(SceneAssembly, ApplyLayerGatesPointCloudsOnLeavesItIntact)
{
    SceneAssembly asm_;
    asm_.point_clouds.push_back(mpviz::PointCloud{});

    LayerFlags flags;  // all true, including point_clouds
    apply_layer_gates(asm_, flags);

    mpviz::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.point_cloud_count, 1u);
}
