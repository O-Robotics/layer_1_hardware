#ifndef AMR_SWEEPER_IMU__JY901_IMU_HPP_
#define AMR_SWEEPER_IMU__JY901_IMU_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
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

  bool open_serial();
  void close_serial();
  bool configure_device();
  bool send_command(uint8_t address, uint16_t value);
  bool reopen_serial_with_baud(int baud);
  std::optional<uint8_t> baud_to_device_code(int baud) const;
  std::optional<uint8_t> rate_to_device_code(double hz) const;
  uint16_t build_return_content_mask() const;
  void report_connection_issue(const std::string & message);
  void report_configuration_issue(const std::string & message);
  void log_escalating_issue(int count, const std::string & message, const std::string & issue_type);
  void reset_issue_counters();
  void read_serial();
  void parse_byte(uint8_t byte);
  void maybe_publish();

  std::string port_;
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
  int device_bootstrap_baud_{9600};
  double device_return_rate_hz_{10.0};
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

  int serial_fd_{-1};
  int active_baud_{9600};
  bool termios_valid_{false};
  bool device_config_applied_{false};
  termios original_tty_{};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> fatal_error_{false};
  int reconnect_attempt_count_{0};
  int connection_issue_count_{0};
  int configuration_issue_count_{0};
  std::string fatal_error_message_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};

  std::array<uint8_t, kFrameLength> frame_buf_{};
  std::size_t frame_size_{0};
  std::array<double, 3> accel_{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_{0.0, 0.0, 0.0};
  std::array<double, 3> euler_deg_{0.0, 0.0, 0.0};

  std::vector<double> orientation_covariance_;
  std::vector<double> angular_velocity_covariance_;
  std::vector<double> linear_acceleration_covariance_;
};

}  // namespace amr_sweeper_imu

#endif  // AMR_SWEEPER_IMU__JY901_IMU_HPP_
