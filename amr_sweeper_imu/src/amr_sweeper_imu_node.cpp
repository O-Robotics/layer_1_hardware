#include "amr_sweeper_imu_node.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <functional>
#include <iostream>
#include <sstream>
#include <cstring>
#include <thread>

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
constexpr auto kFrameValidationTimeout = std::chrono::milliseconds(750);
constexpr auto kRegisterReadTimeout = std::chrono::milliseconds(500);
constexpr double kRateConfirmationTolerance = 0.6;

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

void quaternion_to_rpy(
  double w,
  double x,
  double y,
  double z,
  double & roll,
  double & pitch,
  double & yaw)
{
  const double sinr_cosp = 2.0 * ((w * x) + (y * z));
  const double cosr_cosp = 1.0 - 2.0 * ((x * x) + (y * y));
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * ((w * y) - (z * x));
  if (std::fabs(sinp) >= 1.0) {
    pitch = std::copysign(kPi / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z));
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

void rpy_to_quaternion(
  double roll,
  double pitch,
  double yaw,
  double & w,
  double & x,
  double & y,
  double & z)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;
  w = cr * cp * cy + sr * sp * sy;
}

void rotate_xy(double yaw, double & x, double & y)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double rotated_x = (cos_yaw * x) - (sin_yaw * y);
  const double rotated_y = (sin_yaw * x) + (cos_yaw * y);
  x = rotated_x;
  y = rotated_y;
}

}  // namespace

namespace amr_sweeper_imu
{

JY901ImuNode::JY901ImuNode()
: rclcpp::Node("imu_node")
{
  constexpr char kDefaultDevicePath[] = "/dev/imu_usb";
  device_path_ = declare_parameter<std::string>("device_path", kDefaultDevicePath);
  const auto legacy_port = declare_parameter<std::string>("port", kDefaultDevicePath);
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
  fallback_baud_ = declare_parameter<int>("fallback_baud", 9600);
  fallback_rate_hz_ = declare_parameter<double>("fallback_rate_hz", 10.0);
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
  yaw_offset_deg_ = declare_parameter<double>("yaw_offset_deg", 0.0);
  orientation_covariance_ = declare_parameter<std::vector<double>>(
    "orientation_covariance",
    std::vector<double>{0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.05});
  angular_velocity_covariance_ = declare_parameter<std::vector<double>>(
    "angular_velocity_covariance",
    std::vector<double>{0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.01});
  linear_acceleration_covariance_ = declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance",
    std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5});

  if (device_path_ == kDefaultDevicePath && legacy_port != kDefaultDevicePath) {
    device_path_ = legacy_port;
    RCLCPP_WARN(
      get_logger(),
      "Parameter 'port' is deprecated; use 'device_path' instead. Using legacy value '%s'.",
      device_path_.c_str());
  } else if (legacy_port != kDefaultDevicePath && legacy_port != device_path_) {
    RCLCPP_WARN(
      get_logger(),
      "Both 'device_path' ('%s') and deprecated 'port' ('%s') were set; using 'device_path'.",
      device_path_.c_str(),
      legacy_port.c_str());
  }

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
  yaw_offset_rad_ = yaw_offset_deg_ * kDegToRad;
  active_baud_ = configure_device_on_startup_ ? fallback_baud_ : baud_;
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

  if (!establish_initial_connection()) {
    report_connection_issue(
      last_serial_error_message_.empty() ?
      "Failed to establish IMU serial device '" + device_path_ + "'" :
      last_serial_error_message_);
  } else {
    RCLCPP_INFO(get_logger(), "IMU configuration complete");
    publishing_enabled_ = true;
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
  saw_valid_frame_since_open_ = false;

  serial_fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    last_serial_error_message_ = errno_message(
      "Failed to open IMU serial device '" + device_path_ + "'");
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    last_serial_error_message_ = errno_message(
      "Failed to read IMU serial attributes for '" + device_path_ + "'");
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
      "Failed to configure IMU serial attributes for '" + device_path_ + "'");
    close_serial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(get_logger(), "Opened IMU serial device: %s @ %d", device_path_.c_str(), active_baud_);
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
  saw_valid_frame_since_open_ = false;
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

  std::this_thread::sleep_for(kCommandDelay);
  return true;
}

bool JY901ImuNode::send_read_command(uint8_t start_register)
{
  if (serial_fd_ < 0) {
    return false;
  }

  // The newer WIT standard protocol requires unlocking before read/write access.
  if (!send_unlock_command()) {
    return false;
  }

  const std::array<uint8_t, 5> command{
    0xFF,
    0xAA,
    0x27,
    start_register,
    0x00,
  };

  tcflush(serial_fd_, TCIFLUSH);
  const ssize_t bytes_written = ::write(serial_fd_, command.data(), command.size());
  if (bytes_written != static_cast<ssize_t>(command.size())) {
    return false;
  }

  std::this_thread::sleep_for(kCommandDelay);
  return true;
}

bool JY901ImuNode::send_unlock_command()
{
  return send_command(0x69, 0xB588);
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

std::optional<std::array<uint16_t, 4>> JY901ImuNode::read_register_block(
  uint8_t start_register,
  std::chrono::milliseconds timeout)
{
  if (!send_read_command(start_register)) {
    return std::nullopt;
  }

  std::array<uint8_t, kFrameLength> response{};
  std::size_t response_size = 0U;
  std::array<uint8_t, 256> tmp{};
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline && !stop_requested_.load() && !fatal_error_.load()) {
    const ssize_t bytes_read = ::read(serial_fd_, tmp.data(), tmp.size());
    if (bytes_read > 0) {
      for (ssize_t index = 0; index < bytes_read; ++index) {
        const uint8_t byte = tmp[static_cast<std::size_t>(index)];
        if (response_size == 0U && byte != kFrameHeader) {
          continue;
        }

        response[response_size++] = byte;
        if (response_size < kFrameLength) {
          continue;
        }

        response_size = 0U;
        if (!valid_checksum(response)) {
          continue;
        }

        if (response[1] != kRegisterReadReply) {
          continue;
        }

        return std::array<uint16_t, 4>{
          static_cast<uint16_t>(read_i16_le(&response[2])),
          static_cast<uint16_t>(read_i16_le(&response[4])),
          static_cast<uint16_t>(read_i16_le(&response[6])),
          static_cast<uint16_t>(read_i16_le(&response[8])),
        };
      }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      last_serial_error_message_ = errno_message(
        "Serial read failure while reading IMU registers on '" + device_path_ + "'");
      return std::nullopt;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  last_serial_error_message_ =
    "Timed out while reading IMU registers starting at 0x" +
    [&start_register]() {
      std::ostringstream stream;
      stream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
             << static_cast<int>(start_register);
      return stream.str();
    }();
  return std::nullopt;
}

std::optional<JY901ImuNode::DeviceConfigurationSnapshot> JY901ImuNode::read_device_configuration()
{
  const auto basic_block = read_register_block(0x02, kRegisterReadTimeout);
  if (!basic_block.has_value()) {
    return std::nullopt;
  }

  const auto led_block = read_register_block(0x1B, kRegisterReadTimeout);
  if (!led_block.has_value()) {
    return std::nullopt;
  }

  const auto orientation_block = read_register_block(0x23, kRegisterReadTimeout);
  if (!orientation_block.has_value()) {
    return std::nullopt;
  }

  const auto gyro_auto_calibration_block = read_register_block(0x63, kRegisterReadTimeout);
  if (!gyro_auto_calibration_block.has_value()) {
    return std::nullopt;
  }

  DeviceConfigurationSnapshot config;
  config.return_content_mask = basic_block.value()[0];
  config.rate_code = basic_block.value()[1];
  config.baud_code = basic_block.value()[2];
  config.led_off = led_block.value()[0];
  config.orient = orientation_block.value()[0];
  config.axis6 = orientation_block.value()[1];
  config.gyroscope_auto_calibration_mode = gyro_auto_calibration_block.value()[0];
  return config;
}

std::optional<JY901ImuNode::DeviceConfigurationSnapshot>
JY901ImuNode::build_desired_device_configuration(int target_baud, double target_rate_hz) const
{
  const auto rate_code = rate_to_device_code(target_rate_hz);
  if (!rate_code.has_value()) {
    return std::nullopt;
  }

  const auto baud_code = baud_to_device_code(target_baud);
  if (!baud_code.has_value()) {
    return std::nullopt;
  }

  std::string normalized_algorithm = algorithm_mode_;
  std::transform(
    normalized_algorithm.begin(),
    normalized_algorithm.end(),
    normalized_algorithm.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  DeviceConfigurationSnapshot config;
  config.return_content_mask = build_return_content_mask();
  config.rate_code = rate_code.value();
  config.baud_code = baud_code.value();
  config.led_off = led_enabled_ ? 0U : 1U;
  config.orient = installation_direction_ == "vertical" ? 1U : 0U;
  config.axis6 =
    normalized_algorithm == "six_axis" ||
    normalized_algorithm == "6_axis" ||
    normalized_algorithm == "6axis" ? 1U : 0U;
  config.gyroscope_auto_calibration_mode = gyroscope_auto_calibration_ ? 0U : 1U;
  return config;
}

std::string JY901ImuNode::describe_rate_code(uint16_t code) const
{
  switch (code) {
    case 0x01: return "0.1 Hz";
    case 0x02: return "0.5 Hz";
    case 0x03: return "1 Hz";
    case 0x04: return "2 Hz";
    case 0x05: return "5 Hz";
    case 0x06: return "10 Hz";
    case 0x07: return "20 Hz";
    case 0x08: return "50 Hz";
    case 0x09: return "100 Hz";
    case 0x0A: return "125 Hz";
    case 0x0B: return "200 Hz";
    default: {
      std::ostringstream stream;
      stream << "unknown(0x" << std::hex << std::uppercase << code << ")";
      return stream.str();
    }
  }
}

std::string JY901ImuNode::describe_return_content_mask(uint16_t mask) const
{
  struct MaskBitDescription
  {
    uint16_t bit;
    const char * label;
  };

  static constexpr std::array<MaskBitDescription, 11> descriptions{{
    {static_cast<uint16_t>(1U << 0), "time"},
    {static_cast<uint16_t>(1U << 1), "accel"},
    {static_cast<uint16_t>(1U << 2), "gyro"},
    {static_cast<uint16_t>(1U << 3), "angle"},
    {static_cast<uint16_t>(1U << 4), "magnetic"},
    {static_cast<uint16_t>(1U << 5), "port_status"},
    {static_cast<uint16_t>(1U << 6), "pressure_height"},
    {static_cast<uint16_t>(1U << 7), "gps_coordinates"},
    {static_cast<uint16_t>(1U << 8), "gps_velocity"},
    {static_cast<uint16_t>(1U << 9), "quaternion"},
    {static_cast<uint16_t>(1U << 10), "satellite_accuracy"},
  }};

  std::ostringstream stream;
  bool first = true;
  for (const auto & description : descriptions) {
    if ((mask & description.bit) == 0U) {
      continue;
    }

    if (!first) {
      stream << ", ";
    }
    stream << description.label;
    first = false;
  }

  if (first) {
    stream << "none";
  }

  return stream.str();
}

std::string JY901ImuNode::describe_baud_code(uint16_t code) const
{
  switch (code) {
    case 0x00: return "2400";
    case 0x01: return "4800";
    case 0x02: return "9600";
    case 0x03: return "19200";
    case 0x04: return "38400";
    case 0x05: return "57600";
    case 0x06: return "115200";
    case 0x07: return "230400";
    case 0x08: return "460800";
    case 0x09: return "921600";
    default: {
      std::ostringstream stream;
      stream << "unknown(0x" << std::hex << std::uppercase << code << ")";
      return stream.str();
    }
  }
}

void JY901ImuNode::log_device_configuration(
  const std::string & label,
  const DeviceConfigurationSnapshot & config) const
{
  RCLCPP_INFO(get_logger(), "%s", label.c_str());
  std::cout
    << "return_content_mask=0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
    << config.return_content_mask << std::dec << " [" << describe_return_content_mask(config.return_content_mask) << "]\n"
    << "rate=" << describe_rate_code(config.rate_code) << "\n"
    << "baud=" << describe_baud_code(config.baud_code) << "\n"
    << "led=" << (config.led_off == 0U ? "enabled" : "disabled") << "\n"
    << "orientation=" << (config.orient == 0U ? "horizontal" : "vertical") << "\n"
    << "algorithm=" << (config.axis6 == 0U ? "nine_axis" : "six_axis") << "\n"
    << "gyro_auto_calibration="
    << (config.gyroscope_auto_calibration_mode == 0U ? "enabled" : "disabled")
    << std::endl;
}

std::vector<std::string> JY901ImuNode::diff_device_configuration(
  const DeviceConfigurationSnapshot & current,
  const DeviceConfigurationSnapshot & desired) const
{
  std::vector<std::string> mismatches;

  if (current.return_content_mask != desired.return_content_mask) {
    std::ostringstream stream;
    stream << "return_content_mask current=0x" << std::hex << std::uppercase
           << current.return_content_mask << " desired=0x" << desired.return_content_mask;
    mismatches.push_back(stream.str());
  }
  if (current.rate_code != desired.rate_code) {
    mismatches.push_back(
      "output_rate current=" + describe_rate_code(current.rate_code) +
      " desired=" + describe_rate_code(desired.rate_code));
  }
  if (current.baud_code != desired.baud_code) {
    mismatches.push_back(
      "baud current=" + describe_baud_code(current.baud_code) +
      " desired=" + describe_baud_code(desired.baud_code));
  }
  if (current.led_off != desired.led_off) {
    mismatches.push_back(
      std::string("led current=") + (current.led_off == 0U ? "enabled" : "disabled") +
      " desired=" + (desired.led_off == 0U ? "enabled" : "disabled"));
  }
  if (current.orient != desired.orient) {
    mismatches.push_back(
      std::string("installation_direction current=") +
      (current.orient == 0U ? "horizontal" : "vertical") +
      " desired=" + (desired.orient == 0U ? "horizontal" : "vertical"));
  }
  if (current.axis6 != desired.axis6) {
    mismatches.push_back(
      std::string("algorithm_mode current=") +
      (current.axis6 == 0U ? "nine_axis" : "six_axis") +
      " desired=" + (desired.axis6 == 0U ? "nine_axis" : "six_axis"));
  }
  if (current.gyroscope_auto_calibration_mode != desired.gyroscope_auto_calibration_mode) {
    mismatches.push_back(
      std::string("gyroscope_auto_calibration current=") +
      (current.gyroscope_auto_calibration_mode == 0U ? "enabled" : "disabled") +
      " desired=" + (desired.gyroscope_auto_calibration_mode == 0U ? "enabled" : "disabled"));
  }

  return mismatches;
}

void JY901ImuNode::enter_fatal_state(const std::string & message)
{
  fatal_error_message_ = message;
  RCLCPP_FATAL(get_logger(), "%s", fatal_error_message_.c_str());
  fatal_error_.store(true);
  stop_requested_.store(true);
  if (read_timer_) {
    read_timer_->cancel();
  }
  rclcpp::shutdown();
}

void JY901ImuNode::report_connection_issue(const std::string & message)
{
  ++connection_issue_count_;
  ++reconnect_attempt_count_;

  if (max_reconnect_attempts_ > 0 && reconnect_attempt_count_ >= max_reconnect_attempts_) {
    enter_fatal_state(
      message + ". Reached reconnect limit after " + std::to_string(reconnect_attempt_count_) +
      " attempts");
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

  enter_fatal_state(
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive " + issue_type + " failures");
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
  return configure_device_profile(baud_, publish_hz_);
}

bool JY901ImuNode::configure_device_profile(int target_baud, double target_rate_hz)
{
  if (!configure_device_on_startup_ || device_config_applied_ || serial_fd_ < 0) {
    return true;
  }

  if (!output_acceleration_ || !output_angular_velocity_ ||
    (!output_angle_ && !output_quaternion_))
  {
    RCLCPP_WARN(
      get_logger(),
      "The driver expects acceleration, angular velocity, and either angle or quaternion packets to be enabled");
  }

  const auto desired_config = build_desired_device_configuration(target_baud, target_rate_hz);
  if (!desired_config.has_value()) {
    const auto rate_code = rate_to_device_code(target_rate_hz);
    if (!rate_code.has_value()) {
      RCLCPP_ERROR(get_logger(), "Unsupported target rate: %.3f", target_rate_hz);
    } else {
      RCLCPP_ERROR(get_logger(), "Unsupported baud: %d", target_baud);
    }
    return false;
  }

  const auto current_config = read_device_configuration();
  if (!current_config.has_value()) {
    RCLCPP_ERROR(
      get_logger(),
      "Unable to read IMU configuration from device after opening serial: %s",
      last_serial_error_message_.c_str());
    return false;
  }

  log_device_configuration("Current IMU configuration:", current_config.value());

  const auto mismatches = diff_device_configuration(current_config.value(), desired_config.value());
  if (mismatches.empty()) {
    device_config_applied_ = true;
    return true;
  }

  RCLCPP_WARN(get_logger(), "IMU configuration differs from requested parameters; updating device");
  for (const auto & mismatch : mismatches) {
    RCLCPP_WARN(get_logger(), "  %s", mismatch.c_str());
  }

  bool okay = send_unlock_command();
  okay = send_command(0x02, desired_config->return_content_mask) && okay;
  const bool rate_changed = current_config->rate_code != desired_config->rate_code;
  okay = send_command(0x03, desired_config->rate_code) && okay;
  okay = send_command(0x23, desired_config->orient) && okay;
  okay = send_command(0x24, desired_config->axis6) && okay;
  okay = send_command(0x63, desired_config->gyroscope_auto_calibration_mode) && okay;
  okay = send_command(0x1B, desired_config->led_off) && okay;
  const bool baud_changed = current_config->baud_code != desired_config->baud_code;
  if (baud_changed) {
    okay = send_command(0x04, desired_config->baud_code) && okay;
  }
  if (!okay) {
    return false;
  }

  bool save_ok = true;
  if (save_configuration_) {
    save_ok = send_command(0x00, 0U);
  }

  if (baud_changed && target_baud != active_baud_) {
    std::this_thread::sleep_for(kBaudTransitionDelay);
    if (reopen_serial_with_baud(target_baud)) {
      if (save_configuration_ && !save_ok) {
        save_ok = send_command(0x00, 0U);
      }
    } else {
      const int desired_baud = target_baud;
      const int bootstrap_baud = fallback_baud_;
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

  if (!save_ok) {
    RCLCPP_ERROR(get_logger(), "Failed to save IMU configuration to persistent storage");
    return false;
  }

  const auto verified_config = read_device_configuration();
  if (!verified_config.has_value()) {
    RCLCPP_ERROR(
      get_logger(),
      "Unable to re-read IMU configuration after update: %s",
      last_serial_error_message_.c_str());
    return false;
  }

  log_device_configuration("Verified IMU configuration:", verified_config.value());
  const auto remaining_mismatches =
    diff_device_configuration(verified_config.value(), desired_config.value());
  if (!remaining_mismatches.empty()) {
    for (const auto & mismatch : remaining_mismatches) {
      RCLCPP_ERROR(get_logger(), "IMU configuration verification failed: %s", mismatch.c_str());
    }
    return false;
  }

  if (baud_changed || rate_changed) {
    RCLCPP_WARN(
      get_logger(),
      "The JY901 datasheet notes that baud/rate changes may still require a module restart or re-power to fully take effect.");
  }

  device_config_applied_ = true;
  return true;
}

bool JY901ImuNode::wait_for_valid_frames(std::chrono::milliseconds timeout)
{
  if (serial_fd_ < 0) {
    return false;
  }

  saw_valid_frame_since_open_ = false;
  frame_size_ = 0U;
  validation_frame_count_ = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::array<uint8_t, 256> tmp{};

  while (std::chrono::steady_clock::now() < deadline && !stop_requested_.load() && !fatal_error_.load()) {
    const ssize_t bytes_read = ::read(serial_fd_, tmp.data(), tmp.size());
    if (bytes_read > 0) {
      for (ssize_t index = 0; index < bytes_read; ++index) {
        parse_byte(tmp[static_cast<std::size_t>(index)]);
      }
      if (saw_valid_frame_since_open_) {
        return true;
      }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      last_serial_error_message_ = errno_message(
        "Serial read failure while validating '" + device_path_ + "'");
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  return saw_valid_frame_since_open_;
}

bool JY901ImuNode::confirm_stream_rate(double expected_hz, std::chrono::milliseconds timeout)
{
  if (serial_fd_ < 0) {
    return false;
  }

  saw_valid_frame_since_open_ = false;
  frame_size_ = 0U;
  validation_frame_count_ = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::array<uint8_t, 256> tmp{};

  while (std::chrono::steady_clock::now() < deadline && !stop_requested_.load() && !fatal_error_.load()) {
    const ssize_t bytes_read = ::read(serial_fd_, tmp.data(), tmp.size());
    if (bytes_read > 0) {
      for (ssize_t index = 0; index < bytes_read; ++index) {
        parse_byte(tmp[static_cast<std::size_t>(index)]);
      }
      if (validation_frame_count_ >= 3) {
        const double duration_s = (validation_last_frame_time_ - validation_first_frame_time_).seconds();
        if (duration_s > 1e-6) {
          const double observed_hz =
            static_cast<double>(validation_frame_count_ - 1U) / duration_s;
          if (observed_hz >= expected_hz * kRateConfirmationTolerance) {
            return true;
          }
        }
      }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      last_serial_error_message_ = errno_message(
        "Serial read failure while confirming rate on '" + device_path_ + "'");
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (validation_frame_count_ >= 3) {
    const double duration_s = (validation_last_frame_time_ - validation_first_frame_time_).seconds();
    if (duration_s > 1e-6) {
      const double observed_hz =
        static_cast<double>(validation_frame_count_ - 1U) / duration_s;
      return observed_hz >= expected_hz * kRateConfirmationTolerance;
    }
  }

  return false;
}

bool JY901ImuNode::revert_to_fallback_profile()
{
  active_baud_ = fallback_baud_;
  device_config_applied_ = false;
  if (!open_serial()) {
    return false;
  }
  if (!wait_for_valid_frames(kFrameValidationTimeout)) {
    close_serial();
    return false;
  }
  if (!configure_device_profile(fallback_baud_, fallback_rate_hz_)) {
    close_serial();
    return false;
  }
  if (!confirm_stream_rate(fallback_rate_hz_, std::chrono::milliseconds(1500))) {
    RCLCPP_WARN(
      get_logger(),
      "Fallback IMU profile %.1f Hz could not be confirmed; continuing with the recovered serial link.",
      fallback_rate_hz_);
  }
  return true;
}

bool JY901ImuNode::establish_initial_connection()
{
  const auto try_baud = [this](int baud, const char * label) {
    active_baud_ = baud;
    if (!open_serial()) {
      return false;
    }
    if (!wait_for_valid_frames(kFrameValidationTimeout)) {
      last_serial_error_message_ =
        "No valid IMU frames received while probing " + std::string(label) +
        " baud " + std::to_string(baud);
      close_serial();
      return false;
    }
    return true;
  };

  if (try_baud(baud_, "preferred")) {
    if (!configure_device_profile(baud_, publish_hz_)) {
      last_serial_error_message_ =
        "IMU opened at preferred baud, but device programming did not fully succeed";
      close_serial();
      return false;
    }
    if (!confirm_stream_rate(publish_hz_, std::chrono::milliseconds(1500))) {
      RCLCPP_WARN(
        get_logger(),
        "Preferred IMU profile %d baud / %.1f Hz could not be confirmed. Reverting to fallback %.1f Hz.",
        baud_,
        publish_hz_,
        fallback_rate_hz_);
      if (!revert_to_fallback_profile()) {
        last_serial_error_message_ =
          "Preferred profile could not be confirmed and fallback profile recovery failed";
        return false;
      }
    }
    return true;
  }

  if (baud_ == fallback_baud_) {
    return false;
  }

  const std::string preferred_error = last_serial_error_message_;
  RCLCPP_WARN(
    get_logger(),
    "Preferred IMU baud %d did not yield valid frames: %s. Falling back to bootstrap baud %d.",
    baud_,
    preferred_error.c_str(),
    fallback_baud_);

  if (!try_baud(fallback_baud_, "bootstrap")) {
    last_serial_error_message_ =
      "Preferred baud failed (" + preferred_error + "); bootstrap baud also failed (" +
      last_serial_error_message_ + ")";
    return false;
  }

  if (!configure_device_profile(baud_, publish_hz_)) {
    last_serial_error_message_ =
      "IMU opened at bootstrap baud, but device programming did not fully succeed";
    close_serial();
    return false;
  }

  if (!confirm_stream_rate(publish_hz_, std::chrono::milliseconds(1500))) {
    RCLCPP_WARN(
      get_logger(),
      "Preferred IMU profile %d baud / %.1f Hz could not be confirmed after bootstrap recovery. "
      "Reverting to fallback %.1f Hz.",
      baud_,
      publish_hz_,
      fallback_rate_hz_);
    if (!revert_to_fallback_profile()) {
      last_serial_error_message_ =
        "Preferred profile could not be confirmed after bootstrap recovery and fallback profile recovery failed";
      return false;
    }
  }

  return true;
}

bool JY901ImuNode::reopen_serial_with_baud(int baud)
{
  active_baud_ = baud;
  if (!open_serial()) {
    return false;
  }
  return true;
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
          "Failed to reconnect IMU serial device '" + device_path_ + "'" :
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
    report_connection_issue(errno_message("Serial read failure on '" + device_path_ + "'"));
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

  saw_valid_frame_since_open_ = true;
  const auto now = get_clock()->now();
  if (validation_frame_count_ == 0U) {
    validation_first_frame_time_ = now;
  }
  validation_last_frame_time_ = now;
  ++validation_frame_count_;

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
      if (!output_quaternion_ || !has_quaternion_) {
        maybe_publish();
      }
      break;
    case 0x59:
      quaternion_[0] = static_cast<double>(d0) / 32768.0;
      quaternion_[1] = static_cast<double>(d1) / 32768.0;
      quaternion_[2] = static_cast<double>(d2) / 32768.0;
      quaternion_[3] = static_cast<double>(read_i16_le(&frame_buf_[8])) / 32768.0;
      has_quaternion_ = true;
      maybe_publish();
      break;
    default:
      break;
  }
}

void JY901ImuNode::maybe_publish()
{
  if (!publishing_enabled_) {
    return;
  }

  const auto now = get_clock()->now();
  ensure_publishers_created();
  if (!publishing_started_logged_) {
    RCLCPP_INFO(get_logger(), "IMU publishing topics");
    publishing_started_logged_ = true;
  }
  last_pub_time_ = now;

  const sensor_msgs::msg::Imu raw_msg = build_raw_imu_message(now);
  imu_pub_->publish(raw_msg);
  imu_acc_gyro_pub_->publish(build_accel_gyro_message(raw_msg));
  imu_heading_pub_->publish(build_heading_message(raw_msg));
}

void JY901ImuNode::ensure_publishers_created()
{
  if (!imu_pub_) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("data_raw", 10);
  }
  if (!imu_acc_gyro_pub_) {
    imu_acc_gyro_pub_ = create_publisher<sensor_msgs::msg::Imu>("data_acc_gyro", 10);
  }
  if (!imu_heading_pub_) {
    imu_heading_pub_ = create_publisher<sensor_msgs::msg::Imu>("data_heading", 10);
  }
}

sensor_msgs::msg::Imu JY901ImuNode::build_raw_imu_message(const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  bool orientation_valid = true;
  double roll = euler_deg_[0] * kDegToRad;
  double pitch = euler_deg_[1] * kDegToRad;
  double yaw = euler_deg_[2] * kDegToRad;

  if (output_quaternion_ && has_quaternion_) {
    const double norm = std::sqrt(
      (quaternion_[0] * quaternion_[0]) +
      (quaternion_[1] * quaternion_[1]) +
      (quaternion_[2] * quaternion_[2]) +
      (quaternion_[3] * quaternion_[3]));
    if (norm > 1e-9) {
      quaternion_to_rpy(
        quaternion_[0] / norm,
        quaternion_[1] / norm,
        quaternion_[2] / norm,
        quaternion_[3] / norm,
        roll,
        pitch,
        yaw);
    } else {
      orientation_valid = false;
    }
  }

  if (orientation_valid) {
    yaw += yaw_offset_rad_;

    rpy_to_quaternion(
      roll,
      pitch,
      yaw,
      msg.orientation.w,
      msg.orientation.x,
      msg.orientation.y,
      msg.orientation.z);
  }
  std::copy(orientation_covariance_.begin(), orientation_covariance_.end(), msg.orientation_covariance.begin());
  if (!orientation_valid) {
    msg.orientation_covariance[0] = -1.0;
  }

  msg.angular_velocity.x = gyro_[0];
  msg.angular_velocity.y = gyro_[1];
  msg.angular_velocity.z = gyro_[2];
  rotate_xy(yaw_offset_rad_, msg.angular_velocity.x, msg.angular_velocity.y);
  std::copy(
    angular_velocity_covariance_.begin(),
    angular_velocity_covariance_.end(),
    msg.angular_velocity_covariance.begin());

  msg.linear_acceleration.x = accel_[0];
  msg.linear_acceleration.y = accel_[1];
  msg.linear_acceleration.z = accel_[2];
  rotate_xy(yaw_offset_rad_, msg.linear_acceleration.x, msg.linear_acceleration.y);
  std::copy(
    linear_acceleration_covariance_.begin(),
    linear_acceleration_covariance_.end(),
    msg.linear_acceleration_covariance.begin());

  return msg;
}

sensor_msgs::msg::Imu JY901ImuNode::build_accel_gyro_message(
  const sensor_msgs::msg::Imu & raw_msg) const
{
  sensor_msgs::msg::Imu msg = raw_msg;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return msg;
}

sensor_msgs::msg::Imu JY901ImuNode::build_heading_message(
  const sensor_msgs::msg::Imu & raw_msg) const
{
  sensor_msgs::msg::Imu msg = raw_msg;
  if (raw_msg.orientation_covariance[0] < 0.0) {
    msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return msg;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  quaternion_to_rpy(
    raw_msg.orientation.w,
    raw_msg.orientation.x,
    raw_msg.orientation.y,
    raw_msg.orientation.z,
    roll,
    pitch,
    yaw);
  rpy_to_quaternion(
    0.0,
    0.0,
    yaw,
    msg.orientation.w,
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z);
  const double yaw_variance = raw_msg.orientation_covariance[8] > 0.0 ?
    raw_msg.orientation_covariance[8] : 0.01;
  msg.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw_variance};
  return msg;
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
