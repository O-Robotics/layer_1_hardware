#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr uint8_t kFrameHeader = 0x55;
constexpr size_t kFrameLength = 11;
constexpr double kGravity = 9.8;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

static speed_t baud_to_termios(int baud)
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

static int16_t read_i16_le(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

static bool valid_checksum(const std::array<uint8_t, kFrameLength> & frame)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < kFrameLength - 1; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return sum == frame[kFrameLength - 1];
}

}  // namespace

class JY901ImuNode : public rclcpp::Node
{
public:
  JY901ImuNode()
  : Node("imu_node"), serial_fd_(-1)
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/imu_usb");
    baud_ = this->declare_parameter<int>("baud", 9600);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 10.0);

    if (publish_hz_ < 1.0) {
      publish_hz_ = 1.0;
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    if (!open_serial()) {
      RCLCPP_ERROR(get_logger(), "Failed to open IMU serial port '%s'", port_.c_str());
    }

    read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&JY901ImuNode::read_serial, this));
  }

  ~JY901ImuNode() override
  {
    close_serial();
  }

private:
  bool open_serial()
  {
    close_serial();

    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      return false;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      close_serial();
      return false;
    }

    cfmakeraw(&tty);
    const speed_t speed = baud_to_termios(baud_);
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
      close_serial();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Opened IMU serial: %s @ %d", port_.c_str(), baud_);
    return true;
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void read_serial()
  {
    if (serial_fd_ < 0) {
      static int retry = 0;
      if (++retry >= 500) {
        retry = 0;
        (void)open_serial();
      }
      return;
    }

    std::array<uint8_t, 256> tmp{};
    const ssize_t n = ::read(serial_fd_, tmp.data(), tmp.size());
    if (n <= 0) {
      return;
    }

    for (ssize_t i = 0; i < n; ++i) {
      parse_byte(tmp[static_cast<size_t>(i)]);
    }
  }

  void parse_byte(uint8_t b)
  {
    if (frame_size_ == 0U) {
      if (b != kFrameHeader) {
        return;
      }
    }

    frame_buf_[frame_size_++] = b;
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
      case 0x51: {
          accel_[0] = static_cast<double>(d0) / 32768.0 * 16.0 * kGravity;
          accel_[1] = static_cast<double>(d1) / 32768.0 * 16.0 * kGravity;
          accel_[2] = static_cast<double>(d2) / 32768.0 * 16.0 * kGravity;
        }
        break;
      case 0x52: {
          gyro_[0] = static_cast<double>(d0) / 32768.0 * 2000.0 * kDegToRad;
          gyro_[1] = static_cast<double>(d1) / 32768.0 * 2000.0 * kDegToRad;
          gyro_[2] = static_cast<double>(d2) / 32768.0 * 2000.0 * kDegToRad;
        }
        break;
      case 0x53: {
          euler_deg_[0] = static_cast<double>(d0) / 32768.0 * 180.0;
          euler_deg_[1] = static_cast<double>(d1) / 32768.0 * 180.0;
          euler_deg_[2] = static_cast<double>(d2) / 32768.0 * 180.0;
          maybe_publish();
        }
        break;
      default:
        break;
    }
  }

  void maybe_publish()
  {
    const auto now = this->get_clock()->now();
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
    msg.orientation_covariance = {
      0.2, 0.0, 0.0,
      0.0, 0.2, 0.0,
      0.0, 0.0, 0.05
    };

    msg.angular_velocity.x = gyro_[0];
    msg.angular_velocity.y = gyro_[1];
    msg.angular_velocity.z = gyro_[2];
    msg.angular_velocity_covariance = {
      0.02, 0.0, 0.0,
      0.0, 0.02, 0.0,
      0.0, 0.0, 0.01
    };

    msg.linear_acceleration.x = accel_[0];
    msg.linear_acceleration.y = accel_[1];
    msg.linear_acceleration.z = accel_[2];
    msg.linear_acceleration_covariance = {
      0.5, 0.0, 0.0,
      0.0, 0.5, 0.0,
      0.0, 0.0, 0.5
    };

    imu_pub_->publish(msg);
  }

private:
  std::string port_;
  int baud_;
  std::string frame_id_;
  double publish_hz_;

  int serial_fd_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};

  std::array<uint8_t, kFrameLength> frame_buf_{};
  size_t frame_size_{0};
  std::array<double, 3> accel_{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_{0.0, 0.0, 0.0};
  std::array<double, 3> euler_deg_{0.0, 0.0, 0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JY901ImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
