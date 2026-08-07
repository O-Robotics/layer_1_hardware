#ifndef AMR_SWEEPER_ODRIVE__AMR_SWEEPER_ODRIVE_HARDWARE_INTERFACE_HPP_
#define AMR_SWEEPER_ODRIVE__AMR_SWEEPER_ODRIVE_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <array>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "amr_sweeper_safety_msgs/msg/safety_stop.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

struct can_frame;

namespace amr_sweeper_odrive
{

enum class ProtectionType : std::size_t
{
  OverTorque = 0,
  OverSpeed = 1,
  OverTemperature = 2,
  OverCurrent = 3,
  OverVoltage = 4,
  Count = 5,
};

struct ProtectionLimit
{
  bool enabled{false};
  double threshold{0.0};
  std::string units;
  std::chrono::milliseconds trip_duration{0};
};

struct ProtectionFault
{
  ProtectionType type{ProtectionType::OverTorque};
  std::string joint_name;
  double measured_value{0.0};
  double threshold{0.0};
  std::string units;
};

struct JointTelemetry
{
  double torque_estimate{0.0};
  bool has_torque_estimate{false};
  double speed_rad_s{0.0};
  bool has_speed{false};
  double position_rad{0.0};
  bool has_position{false};
  double current_a{0.0};
  bool has_current{false};
  double motor_temperature_c{0.0};
  bool has_motor_temperature{false};
  double controller_temperature_c{0.0};
  bool has_controller_temperature{false};
  double voltage_v{0.0};
  bool has_voltage{false};
};

struct MotorProtectionState
{
  std::string joint_name;
  std::array<ProtectionLimit, static_cast<std::size_t>(ProtectionType::Count)> limits{};
  std::array<double, static_cast<std::size_t>(ProtectionType::Count)> over_threshold_seconds{};
  std::optional<ProtectionFault> fault;
  bool unsupported_warning_logged{false};
  bool safe_stop_sent{false};
  bool safety_stop_published{false};
};

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
  bool ensureCanInterface();
  void reportConnectionIssue(const std::string & message);
  void logEscalatingIssue(int count, const std::string & message);
  void resetIssueCounters();
  void configureAxisForVelocity(size_t joint_index);
  void requestAxisIdle(size_t joint_index);
  bool requestAxisTelemetry(size_t joint_index, uint8_t cmd_id);
  bool sendVelocityCommand(size_t joint_index, double joint_velocity_rad_s);
  void on_can_msg(const can_frame & frame);
  void processAxisFrame(size_t joint_index, const can_frame & frame);
  void loadProtectionParameters();
  void clearProtectionFaults();
  void requestTelemetryUpdates();
  void evaluateProtections(const rclcpp::Duration & period);
  void updateProtectionStatusState(size_t joint_index);
  void latchProtectionFault(
    size_t joint_index, ProtectionType type, double measured_value,
    const ProtectionLimit & limit);
  bool motorHasLatchedFault(size_t joint_index) const;
  void stopOrDisableMotor(size_t joint_index);
  void clearSafetyStopService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::vector<double> velocity_commands_;
  std::vector<double> prev_velocity_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> position_states_;
  std::vector<double> effort_states_;
  std::vector<double> torque_states_;
  std::vector<double> current_states_;
  std::vector<double> temperature_states_;
  std::vector<double> voltage_states_;
  std::vector<double> fault_latched_states_;
  std::vector<double> fault_type_states_;
  std::vector<double> fault_measured_states_;
  std::vector<double> fault_threshold_states_;
  std::vector<double> positive_motor_direction_signs_;
  std::vector<double> gear_ratios_;
  std::vector<uint32_t> node_ids_;
  std::vector<JointTelemetry> joint_telemetry_;
  std::vector<MotorProtectionState> protection_states_;

  std::string hw_name_;
  std::string can_interface_;
  std::string motor_designation_{"Z4BLD60-48"};
  uint8_t num_joints_ = 0;
  int reconnect_attempt_interval_ms_ {1000};
  int retry_attempts_before_error_ {3};
  int fatal_after_consecutive_errors_ {10};
  int max_reconnect_attempts_ {10};
  std::chrono::milliseconds motor_ready_timeout_{1500};
  int reconnect_attempt_count_ {0};
  int connection_issue_count_ {0};
  bool fatal_error_ {false};
  bool lifecycle_active_ {false};
  bool protection_enabled_ {false};
  bool clear_faults_on_activate_ {true};
  bool latch_faults_ {true};
  std::vector<uint32_t> axis_error_states_;
  std::vector<uint8_t> axis_lifecycle_states_;
  std::vector<bool> axis_heartbeat_received_;
  double command_deadband_for_checks_ {0.0};
  std::chrono::milliseconds startup_ignore_duration_{500};
  std::string safety_stop_topic_name_{"safety_msgs/stop"};
  std::string safety_stop_sender_name_{"odrive_hardware_interface"};
  std::string clear_safety_stop_service_name_{"/odrive_ros2_control/clear_safety_stop"};
  std::string last_connection_error_message_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_time_{};
  rclcpp::Time activation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr safety_stop_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_safety_stop_service_;

  struct Impl;
  std::unique_ptr<Impl> impl_;

  bool confirmMotorTelemetryReady(std::chrono::milliseconds timeout, std::string & failure_reason);
  bool confirmMotorsActive(std::chrono::milliseconds timeout, std::string & failure_reason);
  void resetReadinessTracking();
};

}  // namespace amr_sweeper_odrive

#endif  // AMR_SWEEPER_ODRIVE__AMR_SWEEPER_ODRIVE_HARDWARE_INTERFACE_HPP_
