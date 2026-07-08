#include <chrono>
#include <fstream>
#include <rcutils/logging_macros.h>
#include <stdexcept>
#include <string>

#include "system_info_node.hpp"

namespace
{
std::string trim(const std::string & value)
{
  const size_t start = value.find_first_not_of(" \t\r\n");
  const size_t end = value.find_last_not_of(" \t\r\n");
  return (start == std::string::npos) ? "" : value.substr(start, end - start + 1);
}

bool string_to_bool(const std::string & value)
{
  return value == "1";
}
}  // namespace

SystemInfoPublisher::SystemInfoPublisher()
: Node("system_info_node"),
  use_simulation_(declare_parameter<bool>("use_simulation", false)),
  monitored_files_(declare_parameter<std::vector<std::string>>(
      "monitored_files",
      std::vector<std::string>{
        "/opt/robot_config/robot_config.global.env",
        "/opt/robot_config/monitoring/temperature.txt",
        "/opt/robot_config/monitoring/cpu.txt",
        "/opt/robot_config/monitoring/memory.txt",
        "/opt/robot_config/monitoring/disk.txt",
        "/opt/robot_config/monitoring/network.txt",
      }))
{
  const auto publish_period_sec = declare_parameter<double>("publish_period_sec", 15.0);
  publisher_ = create_publisher<amr_sweeper_system_info_msgs::msg::SystemState>(
    "system_info",
    rclcpp::QoS(1).reliable().transient_local());
  publish_data();
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(publish_period_sec)),
    std::bind(&SystemInfoPublisher::publish_data, this));
}

void SystemInfoPublisher::publish_data()
{
  auto message = amr_sweeper_system_info_msgs::msg::SystemState();

  if (use_simulation_) {
    message.device_type = "simulation";
    message.robot_number = 0;
    message.temperature = 25;
    message.cpu_load = 10;
    message.cpu_idle = 90;
    message.memory_usage = 20;
    message.disk_usage = 15;
    message.conn_type = "simulated";
    message.is_wifi = false;
    message.is_mobile = false;
    publisher_->publish(message);
    return;
  }

  for (const auto & filename : monitored_files_) {
    std::ifstream file(filename);

    if (!file.is_open()) {
      if (unreadable_files_.insert(filename).second) {
        RCLCPP_ERROR(get_logger(), "Could not open file '%s'", filename.c_str());
      }
      continue;
    }
    unreadable_files_.erase(filename);

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }

      const size_t delimiter_pos = line.find('=');
      if (delimiter_pos == std::string::npos) {
        RCLCPP_WARN(get_logger(), "Skipping invalid line: %s", line.c_str());
        continue;
      }

      const std::string key = trim(line.substr(0, delimiter_pos));
      const std::string value = trim(line.substr(delimiter_pos + 1));
      if (!key.empty()) {
        apply_key_value(message, key, value);
      }
    }
  }

  publisher_->publish(message);
}

void SystemInfoPublisher::apply_key_value(
  amr_sweeper_system_info_msgs::msg::SystemState & message,
  const std::string & key,
  const std::string & value) const
{
  if (key == "DEVICE_TYPE") {
    message.device_type = value;
    return;
  }
  if (key == "ROBOT_NUMBER") {
    message.robot_number = std::stoi(value);
    return;
  }
  if (key == "TEMPERATURE") {
    message.temperature = std::stoi(value);
    return;
  }
  if (key == "CPU_LOAD") {
    message.cpu_load = std::stoi(value);
    return;
  }
  if (key == "CPU_IDLE") {
    message.cpu_idle = std::stoi(value);
    return;
  }
  if (key == "MEMORY_USAGE") {
    message.memory_usage = std::stoi(value);
    return;
  }
  if (key == "DISK_USAGE") {
    message.disk_usage = std::stoi(value);
    return;
  }
  if (key == "CONN_TYPE") {
    message.conn_type = value;
    return;
  }
  if (key == "IS_WIFI") {
    message.is_wifi = string_to_bool(value);
    return;
  }
  if (key == "IS_MOBILE") {
    message.is_mobile = string_to_bool(value);
  }
}

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemInfoPublisher>();
    RCLCPP_INFO(node->get_logger(), "Loaded SystemInfoPublisher node.");
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception & exception) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("system_info_node"),
        "Unhandled exception: %s", exception.what());
      rclcpp::shutdown();
    } else {
      RCUTILS_LOG_FATAL_NAMED(
        "system_info_node", "Unhandled exception before ROS startup: %s",
        exception.what());
    }
    return 1;
  }
  return 0;
}
