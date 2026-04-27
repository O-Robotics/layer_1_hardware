#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "system_info_msgs/msg/system_state.hpp"

using namespace std::chrono_literals;

std::string cfg_directory = "/opt/robot_config/";
std::string mon_directory = cfg_directory + "monitoring/";
std::string list_of_files[] = {
              cfg_directory + "robot_config.global.env",
              mon_directory + "temperature.txt",
              mon_directory + "cpu.txt",
              mon_directory + "memory.txt",
              mon_directory + "disk.txt",
              mon_directory + "network.txt"
            };

// Function to trim whitespace from both ends of a string
std::string trim(const std::string &s) {

    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);

}

// String to Boolean conversion
bool stringToBool(const std::string& str) {
    return (str == "1");
}

// Define a class that inherits from rclcpp::Node
class SystemInfoPublisher : public rclcpp::Node
{
public:
    SystemInfoPublisher()
    : Node("system_info_node"), count_(0)
    {
        // Create a publisher for system_info_msgs::msg::SystemState on topic "/system_info"
        publisher_ = this->create_publisher<system_info_msgs::msg::SystemState>("system_info", 10);

        // Create a timer to call publish_message() every 1 minute
        timer_ = this->create_wall_timer(
            15s, std::bind(&SystemInfoPublisher::publish_data, this));
    }

private:
    int publish_data()
    {
        auto message = system_info_msgs::msg::SystemState();

        for(const auto &filename : list_of_files) {
           std::ifstream file(filename);

           if (!file.is_open()) {
              std::cerr << "Error: Could not open file '" << filename << "'\n";
              return 1;
           }

           std::string line;

           while (std::getline(file, line)) {
              // Ignore empty lines and comments starting with '#'
              if (line.empty() || line[0] == '#') continue;

              size_t delimiterPos = line.find('=');
              if (delimiterPos == std::string::npos) {
                 std::cerr << "Warning: Skipping invalid line: " << line << "\n";
                 continue;
              }

              std::string key = trim(line.substr(0, delimiterPos));
              std::string value = trim(line.substr(delimiterPos + 1));

              if (!key.empty()) {
                 if (!key.compare("DEVICE_TYPE"))
                    { message.device_type = value; continue; };
                 if (!key.compare("ROBOT_NUMBER"))
                    { message.robot_number = std::stoi(value); continue; };
                 if (!key.compare("TEMPERATURE"))
                    { message.temperature = std::stoi(value); continue; };
                 if (!key.compare("CPU_LOAD"))
                    { message.cpu_load = std::stoi(value); continue; };
                 if (!key.compare("CPU_IDLE"))
                    { message.cpu_idle = std::stoi(value); continue; };
                 if (!key.compare("MEMORY_USAGE"))
                    { message.memory_usage = std::stoi(value); continue; };
                 if (!key.compare("DISK_USAGE"))
                    { message.disk_usage = std::stoi(value); continue; };
                 if (!key.compare("CONN_TYPE"))
                    { message.conn_type = value; continue; };
                 if (!key.compare("IS_WIFI"))
                    { message.is_wifi = stringToBool(value); continue; };
                 if (!key.compare("IS_MOBILE"))
                    { message.is_mobile = stringToBool(value); };
              }
           }
           file.close();
        }

        /* RCLCPP_INFO(this->get_logger(), "%s%d%s%s%s%d%s%d%s%d%s%d%s%d%s%s%s%d%s%d%s", \
                   "\nrobot_number: '", message.robot_number, \
                   "'\ndevice_type: '", message.device_type.c_str(), \
                   "'\ntemperature: '", message.temperature, \
                   "'\ncpu_load: '", message.cpu_load, \
                   "'\ncpu_idle: '", message.cpu_idle, \
                   "'\nmemory_usage: '", message.memory_usage, \
                   "'\ndisk_usage: '", message.disk_usage, \
                   "'\nconn_type: '", message.conn_type.c_str(), \
                   "'\nis_wifi: '", message.is_wifi, \
                   "'\nis_mobile: '", message.is_mobile, "'\n"); */
        publisher_->publish(message);
        return 0;
    }

    rclcpp::Publisher<system_info_msgs::msg::SystemState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;    
};

int main(int argc, char *argv[])
{
    std::cout.setf(std::ios::unitbuf);
    std::cout << "\033[2J\033[1;1H"; // clear terminal window
    std::cout << "Loaded SystemInfoPublisher node. Please wait.\n";

    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<SystemInfoPublisher>());
        rclcpp::shutdown();
    } catch (const std::exception &e) {
        fprintf(stderr, "Exception: %s\n", e.what());
        return 1;
    }
    return 0;
}
