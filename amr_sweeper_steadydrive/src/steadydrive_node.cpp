#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("steadydrive_node");
  RCLCPP_INFO(node->get_logger(), "steadydrive_node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
