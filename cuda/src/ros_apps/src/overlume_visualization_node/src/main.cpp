/** @file main.cpp @brief Entry point for the visualization_node executable. */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "overlume_visualization_node/visualization_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<overlume::visualization_app::VisualizationNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
