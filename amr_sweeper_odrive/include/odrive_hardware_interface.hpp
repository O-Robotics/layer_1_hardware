#ifndef AMR_SWEEPER_ODRIVE__ODRIVE_HARDWARE_INTERFACE_HPP_
#define AMR_SWEEPER_ODRIVE__ODRIVE_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

struct can_frame;

namespace amr_sweeper_odrive
{

class ODriveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterface)
  virtual ~ODriveHardwareInterface() override;

  virtual hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  virtual hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  virtual hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void writeCommandsToHardware();
  void updateJointsFromHardware();
  virtual hardware_interface::CallbackReturn validateJoints();
  bool initializeCanInterface();
  void closeCanInterface();
  void configureAxisForVelocity(size_t joint_index);
  void requestAxisIdle(size_t joint_index);
  void sendVelocityCommand(size_t joint_index, double joint_velocity_rad_s);
  void on_can_msg(const can_frame & frame);
  void processAxisFrame(size_t joint_index, const can_frame & frame);

  std::vector<double> velocity_commands_;
  std::vector<double> prev_velocity_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> position_states_;
  std::vector<double> positive_motor_direction_signs_;
  std::vector<double> gear_ratios_;
  std::vector<uint32_t> node_ids_;

  std::string hw_name_;
  std::string can_interface_;
  uint8_t num_joints_ = 0;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace amr_sweeper_odrive

#endif  // AMR_SWEEPER_ODRIVE__ODRIVE_HARDWARE_INTERFACE_HPP_
