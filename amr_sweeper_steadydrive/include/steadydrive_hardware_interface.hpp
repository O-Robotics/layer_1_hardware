#ifndef AMR_SWEEPER_STEADYDRIVE__STEADYDRIVE_HARDWARE_INTERFACE_HPP_
#define AMR_SWEEPER_STEADYDRIVE__STEADYDRIVE_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <linux/can.h>

#include "rclcpp/rclcpp.hpp"

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
  virtual hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Hardware interfaces
  virtual hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void writeCommandsToHardware();
  void updateJointsFromHardware();
  virtual hardware_interface::CallbackReturn validateJoints();
  bool initializeCanSockets();
  bool initializeMotorSocket(size_t motor_index);
  void closeCanSockets();
  bool sendMotorCommand(
    size_t motor_index, uint8_t command_byte,
    uint8_t byte1 = 0x00, uint8_t byte2 = 0x00, uint8_t byte3 = 0x00,
    uint8_t byte4 = 0x00, uint8_t byte5 = 0x00, uint8_t byte6 = 0x00,
    uint8_t byte7 = 0x00);
  void readAvailableMotorFrames(size_t motor_index);
  void processMotorFrame(size_t motor_index, const struct can_frame & frame);
  double unwrapEncoderPositionRad(size_t motor_index, uint16_t encoder_position_raw);

  // Store the command for the robot
  std::vector<double> velocity_commands_;
  std::vector<double> prev_velocity_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> position_states_;
  std::vector<double> positive_motor_direction_signs_;
  std::vector<double> gear_ratios_;
  std::vector<uint32_t> motor_can_ids_;
  std::vector<int> can_sockets_;
  std::vector<std::optional<uint16_t>> last_encoder_position_raw_;
  std::vector<double> accumulated_motor_position_rad_;

  // Config parameters 
  std::string hw_name_;
  std::string can_interface_;
  uint8_t num_joints_;
};




} // amr_sweeper_steadydrive



#endif  // AMR_SWEEPER_STEADYDRIVE__STEADYDRIVE_HARDWARE_INTERFACE_HPP_
