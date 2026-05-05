#ifndef AMR_SWEEPER_SYSTEM_INFO__SYSTEM_INFO_PUBLISHER_HPP_
#define AMR_SWEEPER_SYSTEM_INFO__SYSTEM_INFO_PUBLISHER_HPP_

#include <string>
#include <vector>

#include "amr_sweeper_system_info_msgs/msg/system_state.hpp"
#include "rclcpp/rclcpp.hpp"

class SystemInfoPublisher : public rclcpp::Node
{
public:
  SystemInfoPublisher();

private:
  void publish_data();
  void apply_key_value(
    amr_sweeper_system_info_msgs::msg::SystemState & message,
    const std::string & key,
    const std::string & value) const;

  rclcpp::Publisher<amr_sweeper_system_info_msgs::msg::SystemState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> monitored_files_;
};

#endif  // AMR_SWEEPER_SYSTEM_INFO__SYSTEM_INFO_PUBLISHER_HPP_
