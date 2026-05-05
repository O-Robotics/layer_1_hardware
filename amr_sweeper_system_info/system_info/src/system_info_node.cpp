#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "amr_sweeper_system_info/system_info_publisher.hpp"

using namespace std::chrono_literals;

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
: Node("amr_sweeper_system_info_node"),
  monitored_files_({
    "/opt/robot_config/robot_config.global.env",
    "/opt/robot_config/monitoring/temperature.txt",
    "/opt/robot_config/monitoring/cpu.txt",
    "/opt/robot_config/monitoring/memory.txt",
    "/opt/robot_config/monitoring/disk.txt",
    "/opt/robot_config/monitoring/network.txt",
  })
{
  publisher_ = create_publisher<amr_sweeper_system_info_msgs::msg::SystemState>("system_info", 10);
  timer_ = create_wall_timer(15s, std::bind(&SystemInfoPublisher::publish_data, this));
}

void SystemInfoPublisher::publish_data()
{
  auto message = amr_sweeper_system_info_msgs::msg::SystemState();

  for (const auto & filename : monitored_files_) {
    std::ifstream file(filename);

    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open file '%s'", filename.c_str());
      return;
    }

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
  std::cout.setf(std::ios::unitbuf);
  std::cout << "\033[2J\033[1;1H";
  std::cout << "Loaded SystemInfoPublisher node. Please wait.\n";

  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemInfoPublisher>());
    rclcpp::shutdown();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "Exception: %s\n", exception.what());
    return 1;
  }
  return 0;
}
