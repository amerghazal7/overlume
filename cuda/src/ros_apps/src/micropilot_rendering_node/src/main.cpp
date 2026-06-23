/** @file main.cpp @brief Entry point for the rendering_node executable. */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "micropilot_rendering_node/rendering_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<micropilot::rendering_app::RenderingNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
