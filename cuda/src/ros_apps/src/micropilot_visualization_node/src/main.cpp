/** @file main.cpp @brief Entry point for the visualization_node executable. */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "micropilot_visualization_node/visualization_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<micropilot::visualization_app::VisualizationNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
