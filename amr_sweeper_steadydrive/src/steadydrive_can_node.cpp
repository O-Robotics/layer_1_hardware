#include "steadydrive_can_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

SteadyDriveCanNode::SteadyDriveCanNode()
: Node("steadymotor_driver")
{
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<std::string>("motor_can_id", "0x141");

  can_interface_ = this->get_parameter("can_interface").as_string();
  const std::string motor_can_id_str = this->get_parameter("motor_can_id").as_string();
  motor_can_id_ = std::stoul(motor_can_id_str, nullptr, 0);

  RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface_.c_str());

  if (!initialize_can_socket()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket. Exiting.");
    rclcpp::shutdown();
  }

  struct can_frame startup_response {};
  if (send_can_command(0x88, startup_response)) {
    motor_enabled_ = true;
    RCLCPP_INFO(this->get_logger(), "Motor On command sent during node startup.");
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to send Motor On command during node startup.");
  }

  motor_off_service_ = this->create_service<std_srvs::srv::Trigger>(
    "motor_off",
    std::bind(&SteadyDriveCanNode::motor_off_callback, this, std::placeholders::_1, std::placeholders::_2));
  motor_on_service_ = this->create_service<std_srvs::srv::Trigger>(
    "motor_on",
    std::bind(&SteadyDriveCanNode::motor_on_callback, this, std::placeholders::_1, std::placeholders::_2));
  motor_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "motor_stop",
    std::bind(&SteadyDriveCanNode::motor_stop_callback, this, std::placeholders::_1, std::placeholders::_2));
  read_encoder_service_ = this->create_service<std_srvs::srv::Trigger>(
    "read_encoder",
    std::bind(&SteadyDriveCanNode::read_encoder_callback, this, std::placeholders::_1, std::placeholders::_2));
  read_motor_state1_service_ = this->create_service<std_srvs::srv::Trigger>(
    "read_motor_state_1",
    std::bind(&SteadyDriveCanNode::read_motor_state1_callback, this, std::placeholders::_1, std::placeholders::_2));
  clear_motor_error_service_ = this->create_service<std_srvs::srv::Trigger>(
    "clear_motor_error",
    std::bind(&SteadyDriveCanNode::clear_motor_error_callback, this, std::placeholders::_1, std::placeholders::_2));
  read_motor_state2_service_ = this->create_service<std_srvs::srv::Trigger>(
    "read_motor_state_2",
    std::bind(&SteadyDriveCanNode::read_motor_state2_callback, this, std::placeholders::_1, std::placeholders::_2));
  read_motor_state3_service_ = this->create_service<std_srvs::srv::Trigger>(
    "read_motor_state_3",
    std::bind(&SteadyDriveCanNode::read_motor_state3_callback, this, std::placeholders::_1, std::placeholders::_2));

  speed_control_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
    "speed_control", 10, std::bind(&SteadyDriveCanNode::speed_control_callback, this, std::placeholders::_1));
  torque_control_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
    "torque_control", 10, std::bind(&SteadyDriveCanNode::torque_control_callback, this, std::placeholders::_1));

  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
  joint_state_publisher_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SteadyDriveCanNode::joint_state_update_callback, this));
}

SteadyDriveCanNode::~SteadyDriveCanNode()
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
  }
}

bool SteadyDriveCanNode::initialize_can_socket()
{
  can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    perror("Socket");
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl");
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("Bind");
    return false;
  }
  return true;
}

bool SteadyDriveCanNode::send_can_command(
  uint8_t command_byte, struct can_frame & response,
  uint8_t byte1, uint8_t byte2, uint8_t byte3,
  uint8_t byte4, uint8_t byte5, uint8_t byte6,
  uint8_t byte7)
{
  struct can_frame frame {};
  frame.can_id = motor_can_id_;
  frame.can_dlc = 8;
  std::memset(frame.data, 0x00, sizeof(frame.data));

  frame.data[0] = command_byte;
  frame.data[1] = byte1;
  frame.data[2] = byte2;
  frame.data[3] = byte3;
  frame.data[4] = byte4;
  frame.data[5] = byte5;
  frame.data[6] = byte6;
  frame.data[7] = byte7;

  if (::write(can_socket_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    perror("Write");
    return false;
  }

  if (::read(can_socket_, &response, sizeof(response)) < 0) {
    perror("Read");
    return false;
  }

  return true;
}

void SteadyDriveCanNode::joint_state_update_callback()
{
  struct can_frame response_frame {};
  if (send_can_command(0x9C, response_frame)) {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.push_back("motor");
    joint_state_msg.position.push_back(response_frame.data[2] | (response_frame.data[3] << 8));
    joint_state_msg.velocity.push_back(response_frame.data[4] | (response_frame.data[5] << 8));
    joint_state_msg.effort.push_back((response_frame.data[6] | (response_frame.data[7] << 8)) & 0x3fff);
    joint_state_publisher_->publish(joint_state_msg);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to read joint state.");
  }
}

void SteadyDriveCanNode::motor_off_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x80, response_frame)) {
    motor_enabled_ = false;
    response->success = true;
    response->message = "Motor turned off successfully.";
  } else {
    response->success = false;
    response->message = "Failed to send Motor Off command.";
  }
}

void SteadyDriveCanNode::motor_on_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x88, response_frame)) {
    motor_enabled_ = true;
    response->success = true;
    response->message = "Motor turned on successfully.";
  } else {
    response->success = false;
    response->message = "Failed to send Motor On command.";
  }
}

void SteadyDriveCanNode::motor_stop_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x81, response_frame)) {
    response->success = true;
    response->message = "Motor stopped successfully.";
  } else {
    response->success = false;
    response->message = "Failed to send Motor Stop command.";
  }
}

void SteadyDriveCanNode::read_encoder_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x90, response_frame)) {
    const uint16_t encoder_position = (response_frame.data[2] | (response_frame.data[3] << 8)) & 0x3fff;
    const uint16_t encoder_raw = (response_frame.data[4] | (response_frame.data[5] << 8)) & 0x3fff;
    const uint16_t encoder_offset = (response_frame.data[6] | (response_frame.data[7] << 8)) & 0x3fff;
    response->success = true;
    response->message =
      "Encoder Position: " + std::to_string(encoder_position) +
      " Encoder Raw " + std::to_string(encoder_raw) +
      " Encoder Offset: " + std::to_string(encoder_offset);
  } else {
    response->success = false;
    response->message = "Failed to read encoder position.";
  }
}

void SteadyDriveCanNode::read_motor_state1_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x9A, response_frame)) {
    const int8_t motor_temperature = response_frame.data[1];
    const uint16_t motor_voltage = response_frame.data[3] | (response_frame.data[4] << 8);
    const uint8_t error_state = response_frame.data[7];
    response->success = true;
    response->message =
      "Motor State 1 - Temperature: " + std::to_string(motor_temperature) +
      "C, Voltage " + std::to_string(motor_voltage) +
      "V, Error State: " + std::to_string(error_state);
  } else {
    response->success = false;
    response->message = "Failed to read Motor State 1.";
  }
}

void SteadyDriveCanNode::clear_motor_error_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x9B, response_frame)) {
    const uint8_t error_state = response_frame.data[7];
    if (error_state == 0) {
      response->success = true;
      response->message = "Motor error cleared successfully.";
    } else {
      response->success = false;
      response->message =
        "Failed to clear motor errors. Current error state: " + std::to_string(error_state);
    }
  } else {
    response->success = false;
    response->message = "Failed to send Clear Motor Error State command.";
  }
}

void SteadyDriveCanNode::read_motor_state2_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x9C, response_frame)) {
    const int temperature = response_frame.data[1];
    const int torque_current = response_frame.data[2] | (response_frame.data[3] << 8);
    const int speed = response_frame.data[4] | (response_frame.data[5] << 8);
    const int encoder_position = (response_frame.data[6] | (response_frame.data[7] << 8)) & 0x3fff;
    response->success = true;
    response->message =
      "Motor State 2 - Temperature: " + std::to_string(temperature) +
      ", Torque Current: " + std::to_string(torque_current) +
      ", Speed: " + std::to_string(speed) +
      ", Encoder Position: " + std::to_string(encoder_position);
  } else {
    response->success = false;
    response->message = "Failed to read Motor State 2.";
  }
}

void SteadyDriveCanNode::read_motor_state3_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  struct can_frame response_frame {};
  if (send_can_command(0x9D, response_frame)) {
    const int temperature = response_frame.data[1];
    const int phase_a_current = response_frame.data[2] | (response_frame.data[3] << 8);
    const int phase_b_current = response_frame.data[4] | (response_frame.data[5] << 8);
    const int phase_c_current = response_frame.data[6] | (response_frame.data[7] << 8);
    response->success = true;
    response->message =
      "Motor State 3 - Temperature: " + std::to_string(temperature) +
      ", Phase A Current: " + std::to_string(phase_a_current) +
      ", Phase B Current: " + std::to_string(phase_b_current) +
      ", Phase C Current: " + std::to_string(phase_c_current);
  } else {
    response->success = false;
    response->message = "Failed to read Motor State 3.";
  }
}

void SteadyDriveCanNode::speed_control_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  int32_t speed_control_value = static_cast<int32_t>(msg->data * 100.0F);
  constexpr int32_t kMaxSpeed = 7200000;
  constexpr int32_t kMinSpeed = -7200000;

  if (speed_control_value > kMaxSpeed || speed_control_value < kMinSpeed) {
    RCLCPP_WARN(this->get_logger(), "Desired speed %.2f dps is out of range. Limiting to valid range.", msg->data);
    speed_control_value = std::clamp(speed_control_value, kMinSpeed, kMaxSpeed);
  }

  const uint8_t byte4 = static_cast<uint8_t>(speed_control_value & 0xFF);
  const uint8_t byte5 = static_cast<uint8_t>((speed_control_value >> 8) & 0xFF);
  const uint8_t byte6 = static_cast<uint8_t>((speed_control_value >> 16) & 0xFF);
  const uint8_t byte7 = static_cast<uint8_t>((speed_control_value >> 24) & 0xFF);

  struct can_frame response_frame {};
  if (send_can_command(0xA2, response_frame, 0x00, 0x00, 0x00, byte4, byte5, byte6, byte7)) {
    if (!last_logged_speed_dps_ || *last_logged_speed_dps_ != msg->data) {
      RCLCPP_INFO(
        this->get_logger(),
        "Speed Control command sent successfully: Desired Speed = %.2f dps",
        msg->data);
      last_logged_speed_dps_ = msg->data;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send Speed Control command.");
  }
}

void SteadyDriveCanNode::torque_control_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  int16_t torque_control_value = static_cast<int16_t>(msg->data);
  constexpr int16_t kMaxTorque = 2048;
  constexpr int16_t kMinTorque = -2048;

  if (torque_control_value > kMaxTorque || torque_control_value < kMinTorque) {
    RCLCPP_WARN(this->get_logger(), "Desired torque %.2f is out of range. Limiting to valid range.", msg->data);
    torque_control_value = std::clamp(torque_control_value, kMinTorque, kMaxTorque);
  }

  const uint8_t byte4 = static_cast<uint8_t>(torque_control_value & 0xff);
  const uint8_t byte5 = static_cast<uint8_t>((torque_control_value >> 8) & 0xFF);

  struct can_frame response_frame {};
  if (send_can_command(0xA1, response_frame, 0x00, 0x00, 0x00, byte4, byte5, 0x00, 0x00)) {
    RCLCPP_INFO(this->get_logger(), "Torque control command sent successfully: desired torque = %.2f dps", msg->data);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send torque control command");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SteadyDriveCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
