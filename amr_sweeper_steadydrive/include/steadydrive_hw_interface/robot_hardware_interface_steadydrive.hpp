#ifndef STEADYDRIVE_HW_INTERFACE__ROBOT_HARDWARE_INTERFACE_STEADYDRIVE_HPP_
#define STEADYDRIVE_HW_INTERFACE__ROBOT_HARDWARE_INTERFACE_STEADYDRIVE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace amr_sweeper_steadydrive
{

class SteadydriveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SteadydriveHardwareInterface)

  // Life-cycle interfaces
  virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Hardware interfaces
  virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Callbacks
  void callback_motor_state_left(const sensor_msgs::msg::JointState::SharedPtr msg);
  void callback_motor_state_right(const sensor_msgs::msg::JointState::SharedPtr msg);

  void writeCommandsToHardware();
  void updateJointsFromHardware();
  virtual hardware_interface::CallbackReturn validateJoints();

  // Store the command for the robot
  std::vector<double> velocity_commands_;
  std::vector<double> prev_velocity_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> position_states_;
  std::vector<double> positive_motor_direction_signs_;
  std::vector<double> gear_ratios_;

  // Config parameters 
  std::string hw_name_;
  uint8_t num_joints_;

  // ROS republisher
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_left_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_right_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_motor_state_left;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_motor_state_right;

  

};




} // amr_sweeper_steadydrive



#endif  // STEADYDRIVE_HW_INTERFACE__ROBOT_HARDWARE_INTERFACE_STEADYDRIVE_HPP_
