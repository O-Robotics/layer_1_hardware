#ifndef AMR_SWEEPER_IMU__IMU_NODE_HPP_
#define AMR_SWEEPER_IMU__IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <termios.h>

namespace amr_sweeper_imu
{

class JY901ImuNode : public rclcpp::Node
{
public:
  JY901ImuNode();
  ~JY901ImuNode() override;

  JY901ImuNode(const JY901ImuNode &) = delete;
  JY901ImuNode & operator=(const JY901ImuNode &) = delete;

private:
  static constexpr uint8_t kFrameHeader = 0x55;
  static constexpr std::size_t kFrameLength = 11;
  static constexpr uint8_t kRegisterReadReply = 0x5F;

  struct DeviceConfigurationSnapshot
  {
    uint16_t return_content_mask{0};
    uint16_t rate_code{0};
    uint16_t baud_code{0};
    uint16_t led_off{0};
    uint16_t orient{0};
    uint16_t axis6{0};
  };

  bool open_serial();
  void close_serial();
  bool configure_device();
  bool configure_device_profile(int target_baud, double target_rate_hz);
  bool revert_to_fallback_profile();
  bool establish_initial_connection();
  bool wait_for_valid_frames(std::chrono::milliseconds timeout);
  bool confirm_stream_rate(double expected_hz, std::chrono::milliseconds timeout);
  bool send_unlock_command();
  bool send_command(uint8_t address, uint16_t value);
  bool send_read_command(uint8_t start_register);
  std::optional<std::array<uint16_t, 4>> read_register_block(
    uint8_t start_register,
    std::chrono::milliseconds timeout);
  std::optional<DeviceConfigurationSnapshot> read_device_configuration();
  std::optional<DeviceConfigurationSnapshot> build_desired_device_configuration(
    int target_baud,
    double target_rate_hz) const;
  std::vector<std::string> diff_device_configuration(
    const DeviceConfigurationSnapshot & current,
    const DeviceConfigurationSnapshot & desired) const;
  void log_device_configuration(
    const std::string & label,
    const DeviceConfigurationSnapshot & config) const;
  std::string describe_return_content_mask(uint16_t mask) const;
  std::string describe_rate_code(uint16_t code) const;
  std::string describe_baud_code(uint16_t code) const;
  bool reopen_serial_with_baud(int baud);
  std::optional<uint8_t> baud_to_device_code(int baud) const;
  std::optional<uint8_t> rate_to_device_code(double hz) const;
  uint16_t build_return_content_mask() const;
  void enter_fatal_state(const std::string & message);
  void report_connection_issue(const std::string & message);
  void report_configuration_issue(const std::string & message);
  void log_escalating_issue(int count, const std::string & message, const std::string & issue_type);
  void reset_issue_counters();
  void read_serial();
  void parse_byte(uint8_t byte);
  void maybe_publish();
  void ensure_publishers_created();
  sensor_msgs::msg::Imu build_raw_imu_message(const rclcpp::Time & stamp) const;
  sensor_msgs::msg::Imu build_accel_gyro_message(const sensor_msgs::msg::Imu & raw_msg) const;
  sensor_msgs::msg::Imu build_heading_message(const sensor_msgs::msg::Imu & raw_msg) const;

  std::string device_path_;
  int baud_{9600};
  std::string frame_id_{"imu_link"};
  double publish_hz_{10.0};
  int read_period_ms_{2};
  int reconnect_attempt_interval_ms_{1000};
  int retry_attempts_before_error_{3};
  int fatal_after_consecutive_errors_{10};
  int max_reconnect_attempts_{10};
  bool configure_device_on_startup_{true};
  bool save_configuration_{true};
  int fallback_baud_{9600};
  double fallback_rate_hz_{10.0};
  std::string installation_direction_{"horizontal"};
  std::string algorithm_mode_{"nine_axis"};
  bool gyroscope_auto_calibration_{true};
  bool led_enabled_{true};
  bool output_time_{false};
  bool output_acceleration_{true};
  bool output_angular_velocity_{true};
  bool output_angle_{true};
  bool output_magnetic_{false};
  bool output_port_status_{false};
  bool output_pressure_height_{false};
  bool output_gps_coordinates_{false};
  bool output_gps_velocity_{false};
  bool output_quaternion_{false};
  bool output_satellite_accuracy_{false};
  double yaw_offset_deg_{0.0};
  double yaw_offset_rad_{0.0};

  int serial_fd_{-1};
  int active_baud_{9600};
  bool termios_valid_{false};
  bool device_config_applied_{false};
  bool publishing_enabled_{false};
  bool publishing_started_logged_{false};
  termios original_tty_{};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> fatal_error_{false};
  int reconnect_attempt_count_{0};
  int connection_issue_count_{0};
  int configuration_issue_count_{0};
  std::string fatal_error_message_;
  std::string last_serial_error_message_;
  bool saw_valid_frame_since_open_{false};
  std::size_t validation_frame_count_{0};
  rclcpp::Time validation_first_frame_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time validation_last_frame_time_{0, 0, RCL_SYSTEM_TIME};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_acc_gyro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_heading_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};

  std::array<uint8_t, kFrameLength> frame_buf_{};
  std::size_t frame_size_{0};
  std::array<double, 3> accel_{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_{0.0, 0.0, 0.0};
  std::array<double, 3> euler_deg_{0.0, 0.0, 0.0};
  std::array<double, 4> quaternion_{1.0, 0.0, 0.0, 0.0};
  bool has_quaternion_{false};

  std::vector<double> orientation_covariance_;
  std::vector<double> angular_velocity_covariance_;
  std::vector<double> linear_acceleration_covariance_;
};

}  // namespace amr_sweeper_imu

#endif  // AMR_SWEEPER_IMU__IMU_NODE_HPP_
