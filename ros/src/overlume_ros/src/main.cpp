/** @file main.cpp @brief Entry point for the overlume_node executable. */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "overlume_ros/overlume_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<overlume::ros::OverlumeNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
