#include "jy901_imu.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>

namespace
{

constexpr double kGravity = 9.8;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::size_t kFrameLength = 11;
constexpr auto kCommandDelay = std::chrono::milliseconds(50);
constexpr auto kBaudTransitionDelay = std::chrono::milliseconds(200);

speed_t baud_to_termios(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}

int16_t read_i16_le(const uint8_t * bytes)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(bytes[0]) |
    (static_cast<uint16_t>(bytes[1]) << 8));
}

bool valid_checksum(const std::array<uint8_t, kFrameLength> & frame)
{
  uint8_t sum = 0;
  for (std::size_t index = 0; index < kFrameLength - 1; ++index) {
    sum = static_cast<uint8_t>(sum + frame[index]);
  }
  return sum == frame[kFrameLength - 1];
}

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

}  // namespace

namespace amr_sweeper_imu
{

JY901ImuNode::JY901ImuNode()
: rclcpp::Node("imu_node")
{
  port_ = declare_parameter<std::string>("port", "/dev/imu_usb");
  baud_ = declare_parameter<int>("baud", 9600);
  frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
  publish_hz_ = declare_parameter<double>("publish_hz", 10.0);
  read_period_ms_ = declare_parameter<int>("read_period_ms", 2);
  reconnect_attempt_interval_ms_ = declare_parameter<int>("reconnect_attempt_interval_ms", 1000);
  retry_attempts_before_error_ = declare_parameter<int>("retry_attempts_before_error", 3);
  fatal_after_consecutive_errors_ = declare_parameter<int>("fatal_after_consecutive_errors", 10);
  max_reconnect_attempts_ = declare_parameter<int>("max_reconnect_attempts", 10);
  configure_device_on_startup_ = declare_parameter<bool>("configure_device_on_startup", true);
  save_configuration_ = declare_parameter<bool>("save_configuration", true);
  device_bootstrap_baud_ = declare_parameter<int>("device_bootstrap_baud", 9600);
  device_return_rate_hz_ = declare_parameter<double>("device_return_rate_hz", 10.0);
  installation_direction_ = declare_parameter<std::string>("installation_direction", "horizontal");
  algorithm_mode_ = declare_parameter<std::string>("algorithm_mode", "nine_axis");
  gyroscope_auto_calibration_ = declare_parameter<bool>("gyroscope_auto_calibration", true);
  led_enabled_ = declare_parameter<bool>("led_enabled", true);
  output_time_ = declare_parameter<bool>("output_time", false);
  output_acceleration_ = declare_parameter<bool>("output_acceleration", true);
  output_angular_velocity_ = declare_parameter<bool>("output_angular_velocity", true);
  output_angle_ = declare_parameter<bool>("output_angle", true);
  output_magnetic_ = declare_parameter<bool>("output_magnetic", false);
  output_port_status_ = declare_parameter<bool>("output_port_status", false);
  output_pressure_height_ = declare_parameter<bool>("output_pressure_height", false);
  output_gps_coordinates_ = declare_parameter<bool>("output_gps_coordinates", false);
  output_gps_velocity_ = declare_parameter<bool>("output_gps_velocity", false);
  output_quaternion_ = declare_parameter<bool>("output_quaternion", false);
  output_satellite_accuracy_ = declare_parameter<bool>("output_satellite_accuracy", false);
  orientation_covariance_ = declare_parameter<std::vector<double>>(
    "orientation_covariance",
    std::vector<double>{0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.05});
  angular_velocity_covariance_ = declare_parameter<std::vector<double>>(
    "angular_velocity_covariance",
    std::vector<double>{0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.01});
  linear_acceleration_covariance_ = declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance",
    std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5});

  if (publish_hz_ < 1.0) {
    publish_hz_ = 1.0;
  }
  if (read_period_ms_ < 1) {
    read_period_ms_ = 1;
  }
  if (reconnect_attempt_interval_ms_ < read_period_ms_) {
    reconnect_attempt_interval_ms_ = read_period_ms_;
  }
  if (retry_attempts_before_error_ < 1) {
    retry_attempts_before_error_ = 1;
  }
  if (fatal_after_consecutive_errors_ < 1) {
    fatal_after_consecutive_errors_ = 1;
  }
  if (max_reconnect_attempts_ < 0) {
    max_reconnect_attempts_ = 0;
  }
  active_baud_ = configure_device_on_startup_ ? device_bootstrap_baud_ : baud_;
  if (orientation_covariance_.size() != 9) {
    RCLCPP_WARN(get_logger(), "orientation_covariance must have 9 elements; using defaults");
    orientation_covariance_ = {0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.05};
  }
  if (angular_velocity_covariance_.size() != 9) {
    RCLCPP_WARN(get_logger(), "angular_velocity_covariance must have 9 elements; using defaults");
    angular_velocity_covariance_ = {0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.01};
  }
  if (linear_acceleration_covariance_.size() != 9) {
    RCLCPP_WARN(get_logger(), "linear_acceleration_covariance must have 9 elements; using defaults");
    linear_acceleration_covariance_ = {0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5};
  }

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("data_raw", 10);

  if (!open_serial()) {
    report_connection_issue(
      last_serial_error_message_.empty() ?
      "Failed to open IMU serial port '" + port_ + "'" :
      last_serial_error_message_);
  } else if (!configure_device()) {
    report_configuration_issue("IMU opened, but device programming did not fully succeed");
  } else {
    reset_issue_counters();
  }

  read_timer_ = create_wall_timer(
    std::chrono::milliseconds(read_period_ms_), std::bind(&JY901ImuNode::read_serial, this));
}

JY901ImuNode::~JY901ImuNode()
{
  stop_requested_.store(true);
  if (read_timer_) {
    read_timer_->cancel();
  }
  close_serial();
}

bool JY901ImuNode::open_serial()
{
  close_serial();
  last_serial_error_message_.clear();

  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    last_serial_error_message_ = errno_message("Failed to open IMU serial port '" + port_ + "'");
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    last_serial_error_message_ = errno_message(
      "Failed to read IMU serial attributes for '" + port_ + "'");
    close_serial();
    return false;
  }

  original_tty_ = tty;
  termios_valid_ = true;

  cfmakeraw(&tty);
  const speed_t speed = baud_to_termios(active_baud_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    last_serial_error_message_ = errno_message(
      "Failed to configure IMU serial attributes for '" + port_ + "'");
    close_serial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(get_logger(), "Opened IMU serial: %s @ %d", port_.c_str(), active_baud_);
  return true;
}

void JY901ImuNode::close_serial()
{
  if (serial_fd_ < 0) {
    return;
  }

  if (read_timer_) {
    read_timer_->cancel();
  }

  tcflush(serial_fd_, TCIOFLUSH);

  if (termios_valid_) {
    (void)tcsetattr(serial_fd_, TCSANOW, &original_tty_);
    termios_valid_ = false;
  }

  ::close(serial_fd_);
  serial_fd_ = -1;
  frame_size_ = 0U;
}

bool JY901ImuNode::send_command(uint8_t address, uint16_t value)
{
  if (serial_fd_ < 0) {
    return false;
  }

  const std::array<uint8_t, 5> command{
    0xFF,
    0xAA,
    address,
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
  };

  const ssize_t bytes_written = ::write(serial_fd_, command.data(), command.size());
  if (bytes_written != static_cast<ssize_t>(command.size())) {
    return false;
  }

  if (tcdrain(serial_fd_) != 0) {
    return false;
  }

  std::this_thread::sleep_for(kCommandDelay);
  return true;
}

std::optional<uint8_t> JY901ImuNode::baud_to_device_code(int baud) const
{
  switch (baud) {
    case 2400: return 0x00;
    case 4800: return 0x01;
    case 9600: return 0x02;
    case 19200: return 0x03;
    case 38400: return 0x04;
    case 57600: return 0x05;
    case 115200: return 0x06;
    case 230400: return 0x07;
    case 460800: return 0x08;
    case 921600: return 0x09;
    default: return std::nullopt;
  }
}

std::optional<uint8_t> JY901ImuNode::rate_to_device_code(double hz) const
{
  const auto nearly_equal = [](double left, double right) {
    return std::fabs(left - right) < 1e-6;
  };

  if (nearly_equal(hz, 0.1)) return 0x01;
  if (nearly_equal(hz, 0.5)) return 0x02;
  if (nearly_equal(hz, 1.0)) return 0x03;
  if (nearly_equal(hz, 2.0)) return 0x04;
  if (nearly_equal(hz, 5.0)) return 0x05;
  if (nearly_equal(hz, 10.0)) return 0x06;
  if (nearly_equal(hz, 20.0)) return 0x07;
  if (nearly_equal(hz, 50.0)) return 0x08;
  if (nearly_equal(hz, 100.0)) return 0x09;
  if (nearly_equal(hz, 125.0)) return 0x0A;
  if (nearly_equal(hz, 200.0)) return 0x0B;
  return std::nullopt;
}

uint16_t JY901ImuNode::build_return_content_mask() const
{
  uint16_t mask = 0;
  mask |= static_cast<uint16_t>(output_time_) << 0;
  mask |= static_cast<uint16_t>(output_acceleration_) << 1;
  mask |= static_cast<uint16_t>(output_angular_velocity_) << 2;
  mask |= static_cast<uint16_t>(output_angle_) << 3;
  mask |= static_cast<uint16_t>(output_magnetic_) << 4;
  mask |= static_cast<uint16_t>(output_port_status_) << 5;
  mask |= static_cast<uint16_t>(output_pressure_height_) << 6;
  mask |= static_cast<uint16_t>(output_gps_coordinates_) << 7;
  mask |= static_cast<uint16_t>(output_gps_velocity_) << 8;
  mask |= static_cast<uint16_t>(output_quaternion_) << 9;
  mask |= static_cast<uint16_t>(output_satellite_accuracy_) << 10;
  return mask;
}

bool JY901ImuNode::reopen_serial_with_baud(int baud)
{
  active_baud_ = baud;
  if (!open_serial()) {
    return false;
  }
  return true;
}

void JY901ImuNode::report_connection_issue(const std::string & message)
{
  ++connection_issue_count_;
  ++reconnect_attempt_count_;

  if (max_reconnect_attempts_ > 0 && reconnect_attempt_count_ >= max_reconnect_attempts_) {
    fatal_error_message_ =
      message + ". Reached reconnect limit after " + std::to_string(reconnect_attempt_count_) +
      " attempts";
    RCLCPP_FATAL(get_logger(), "%s", fatal_error_message_.c_str());
    fatal_error_.store(true);
    stop_requested_.store(true);
    if (read_timer_) {
      read_timer_->cancel();
    }
    return;
  }

  log_escalating_issue(connection_issue_count_, message, "connection");
}

void JY901ImuNode::report_configuration_issue(const std::string & message)
{
  ++configuration_issue_count_;
  log_escalating_issue(configuration_issue_count_, message, "configuration");
}

void JY901ImuNode::log_escalating_issue(
  int count,
  const std::string & message,
  const std::string & issue_type)
{
  if (count < retry_attempts_before_error_) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return;
  }

  if (count < fatal_after_consecutive_errors_) {
    if (count == retry_attempts_before_error_) {
      RCLCPP_ERROR(
        get_logger(),
        "%s. Escalating after %d consecutive failures",
        message.c_str(),
        count);
      return;
    }

    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return;
  }

  fatal_error_message_ =
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive " + issue_type + " failures";
  RCLCPP_FATAL(get_logger(), "%s", fatal_error_message_.c_str());
  fatal_error_.store(true);
  stop_requested_.store(true);
  if (read_timer_) {
    read_timer_->cancel();
  }
}

void JY901ImuNode::reset_issue_counters()
{
  reconnect_attempt_count_ = 0;
  connection_issue_count_ = 0;
  configuration_issue_count_ = 0;
  fatal_error_.store(false);
  fatal_error_message_.clear();
}

bool JY901ImuNode::configure_device()
{
  if (!configure_device_on_startup_ || device_config_applied_ || serial_fd_ < 0) {
    return true;
  }

  if (!output_acceleration_ || !output_angular_velocity_ || !output_angle_) {
    RCLCPP_WARN(
      get_logger(),
      "The driver expects acceleration, angular velocity, and angle packets to be enabled");
  }

  const auto rate_code = rate_to_device_code(device_return_rate_hz_);
  if (!rate_code.has_value()) {
    RCLCPP_ERROR(get_logger(), "Unsupported device_return_rate_hz: %.3f", device_return_rate_hz_);
    return false;
  }

  const auto baud_code = baud_to_device_code(baud_);
  if (!baud_code.has_value()) {
    RCLCPP_ERROR(get_logger(), "Unsupported baud: %d", baud_);
    return false;
  }

  const std::string normalized_direction =
    installation_direction_ == "vertical" ? "vertical" : "horizontal";
  const uint16_t direction_value = normalized_direction == "vertical" ? 1U : 0U;

  std::string normalized_algorithm = algorithm_mode_;
  std::transform(
    normalized_algorithm.begin(),
    normalized_algorithm.end(),
    normalized_algorithm.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const bool use_six_axis_algorithm =
    normalized_algorithm == "six_axis" || normalized_algorithm == "6_axis" ||
    normalized_algorithm == "6axis";

  bool okay = true;
  okay = send_command(0x02, build_return_content_mask()) && okay;
  okay = send_command(0x03, rate_code.value()) && okay;
  okay = send_command(0x23, direction_value) && okay;
  okay = send_command(0x24, use_six_axis_algorithm ? 1U : 0U) && okay;
  okay = send_command(0x63, gyroscope_auto_calibration_ ? 0U : 1U) && okay;
  okay = send_command(0x1B, led_enabled_ ? 0U : 1U) && okay;
  okay = send_command(0x04, baud_code.value()) && okay;
  if (!okay) {
    return false;
  }

  bool save_ok = true;
  if (save_configuration_) {
    save_ok = send_command(0x00, 0U);
  }

  if (baud_ != active_baud_) {
    std::this_thread::sleep_for(kBaudTransitionDelay);
    if (reopen_serial_with_baud(baud_)) {
      if (save_configuration_ && !save_ok) {
        save_ok = send_command(0x00, 0U);
      }
    } else {
      const int desired_baud = baud_;
      const int bootstrap_baud = device_bootstrap_baud_;
      active_baud_ = bootstrap_baud;
      if (open_serial()) {
        RCLCPP_WARN(
          get_logger(),
          "IMU accepted configuration writes, but baud %d was not active yet. A device restart may still be required.",
          desired_baud);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "IMU baud transition failed and reconnect at bootstrap baud %d also failed",
          bootstrap_baud);
        return false;
      }
    }
  }

  device_config_applied_ = save_ok;
  return save_ok;
}

void JY901ImuNode::read_serial()
{
  if (stop_requested_.load() || fatal_error_.load()) {
    return;
  }

  if (serial_fd_ < 0) {
    static int retry_elapsed_ms = 0;
    retry_elapsed_ms += read_period_ms_;
    if (retry_elapsed_ms >= reconnect_attempt_interval_ms_) {
      retry_elapsed_ms = 0;
      if (open_serial()) {
        if (!configure_device()) {
          report_configuration_issue("Reconnected to IMU, but device programming did not fully succeed");
        } else {
          reset_issue_counters();
        }
        if (read_timer_) {
          read_timer_->reset();
        }
      } else {
        report_connection_issue(
          last_serial_error_message_.empty() ?
          "Failed to reconnect IMU serial port '" + port_ + "'" :
          last_serial_error_message_);
      }
    }
    return;
  }

  std::array<uint8_t, 256> tmp{};
  const ssize_t bytes_read = ::read(serial_fd_, tmp.data(), tmp.size());
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return;
    }
    close_serial();
    if (read_timer_) {
      read_timer_->reset();
    }
    report_connection_issue(errno_message("Serial read failure on '" + port_ + "'"));
    return;
  }

  if (bytes_read == 0) {
    return;
  }

  for (ssize_t index = 0; index < bytes_read; ++index) {
    parse_byte(tmp[static_cast<std::size_t>(index)]);
  }
}

void JY901ImuNode::parse_byte(uint8_t byte)
{
  if (frame_size_ == 0U && byte != kFrameHeader) {
    return;
  }

  frame_buf_[frame_size_++] = byte;
  if (frame_size_ < kFrameLength) {
    return;
  }

  frame_size_ = 0U;
  if (!valid_checksum(frame_buf_)) {
    return;
  }

  const uint8_t type = frame_buf_[1];
  const int16_t d0 = read_i16_le(&frame_buf_[2]);
  const int16_t d1 = read_i16_le(&frame_buf_[4]);
  const int16_t d2 = read_i16_le(&frame_buf_[6]);

  switch (type) {
    case 0x51:
      accel_[0] = static_cast<double>(d0) / 32768.0 * 16.0 * kGravity;
      accel_[1] = static_cast<double>(d1) / 32768.0 * 16.0 * kGravity;
      accel_[2] = static_cast<double>(d2) / 32768.0 * 16.0 * kGravity;
      break;
    case 0x52:
      gyro_[0] = static_cast<double>(d0) / 32768.0 * 2000.0 * kDegToRad;
      gyro_[1] = static_cast<double>(d1) / 32768.0 * 2000.0 * kDegToRad;
      gyro_[2] = static_cast<double>(d2) / 32768.0 * 2000.0 * kDegToRad;
      break;
    case 0x53:
      euler_deg_[0] = static_cast<double>(d0) / 32768.0 * 180.0;
      euler_deg_[1] = static_cast<double>(d1) / 32768.0 * 180.0;
      euler_deg_[2] = static_cast<double>(d2) / 32768.0 * 180.0;
      maybe_publish();
      break;
    default:
      break;
  }
}

void JY901ImuNode::maybe_publish()
{
  const auto now = get_clock()->now();
  const double period_s = 1.0 / publish_hz_;
  if ((now - last_pub_time_).seconds() < period_s) {
    return;
  }
  last_pub_time_ = now;

  const double roll = euler_deg_[0] * kDegToRad;
  const double pitch = euler_deg_[1] * kDegToRad;
  const double yaw = euler_deg_[2] * kDegToRad;

  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now;
  msg.header.frame_id = frame_id_;

  msg.orientation.x = sr * cp * cy - cr * sp * sy;
  msg.orientation.y = cr * sp * cy + sr * cp * sy;
  msg.orientation.z = cr * cp * sy - sr * sp * cy;
  msg.orientation.w = cr * cp * cy + sr * sp * sy;
  std::copy(orientation_covariance_.begin(), orientation_covariance_.end(), msg.orientation_covariance.begin());

  msg.angular_velocity.x = gyro_[0];
  msg.angular_velocity.y = gyro_[1];
  msg.angular_velocity.z = gyro_[2];
  std::copy(
    angular_velocity_covariance_.begin(),
    angular_velocity_covariance_.end(),
    msg.angular_velocity_covariance.begin());

  msg.linear_acceleration.x = accel_[0];
  msg.linear_acceleration.y = accel_[1];
  msg.linear_acceleration.z = accel_[2];
  std::copy(
    linear_acceleration_covariance_.begin(),
    linear_acceleration_covariance_.end(),
    msg.linear_acceleration_covariance.begin());

  imu_pub_->publish(msg);
}

}  // namespace amr_sweeper_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_imu::JY901ImuNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
