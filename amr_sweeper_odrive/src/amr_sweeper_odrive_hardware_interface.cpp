#include "amr_sweeper_odrive_hardware_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

constexpr size_t LEFT_MOTOR_INDEX = 0;
constexpr size_t RIGHT_MOTOR_INDEX = 1;
constexpr double TWO_PI = 2.0 * M_PI;

enum ODriveAxisState
{
  AXIS_STATE_IDLE = 1,
  AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
};

enum ODriveControlMode
{
  CONTROL_MODE_VELOCITY_CONTROL = 2,
};

enum ODriveInputMode
{
  INPUT_MODE_PASSTHROUGH = 1,
};

template<typename T>
T can_get_signal_raw(const uint8_t * buf, size_t start_bit, size_t length, bool is_intel)
{
  union
  {
    uint64_t temp_bits;
    uint8_t temp_buf[8];
    T value;
  } decoded {};

  std::memcpy(decoded.temp_buf, buf, sizeof(decoded.temp_buf));

  const uint64_t mask = length < 64 ? ((1ULL << length) - 1ULL) : ~0ULL;
  const uint8_t shift = is_intel ? static_cast<uint8_t>(start_bit) :
    static_cast<uint8_t>((64 - start_bit) - length);

  if (is_intel) {
    decoded.temp_bits = (decoded.temp_bits >> shift) & mask;
  } else {
    decoded.temp_bits = __builtin_bswap64(decoded.temp_bits);
    decoded.temp_bits = (decoded.temp_bits >> shift) & mask;
  }

  return decoded.value;
}

template<typename T>
void can_set_signal_raw(uint8_t * buf, T value, size_t start_bit, size_t length, bool is_intel)
{
  union
  {
    uint64_t value_bits;
    T typed_value;
  } encoded_value {};

  union
  {
    uint64_t data;
    uint8_t temp_buf[8];
  } payload {};

  encoded_value.typed_value = value;
  std::memcpy(payload.temp_buf, buf, sizeof(payload.temp_buf));

  const uint64_t mask = length < 64 ? ((1ULL << length) - 1ULL) : ~0ULL;
  const uint8_t shift = is_intel ? static_cast<uint8_t>(start_bit) :
    static_cast<uint8_t>((64 - start_bit) - length);

  if (is_intel) {
    payload.data &= ~(mask << shift);
    payload.data |= encoded_value.value_bits << shift;
  } else {
    payload.data = __builtin_bswap64(payload.data);
    payload.data &= ~(mask << shift);
    payload.data |= encoded_value.value_bits << shift;
    payload.data = __builtin_bswap64(payload.data);
  }

  std::memcpy(buf, payload.temp_buf, sizeof(payload.temp_buf));
}

struct SetAxisStateMessage
{
  static constexpr uint8_t cmd_id = 0x07;
  static constexpr uint8_t msg_length = 8;

  uint32_t axis_requested_state = 0;

  void encode_buf(uint8_t * buf) const
  {
    std::memset(buf, 0, msg_length);
    can_set_signal_raw<uint32_t>(buf, axis_requested_state, 0, 32, true);
  }
};

struct GetEncoderEstimatesMessage
{
  static constexpr uint8_t cmd_id = 0x09;
  static constexpr uint8_t msg_length = 8;

  float pos_estimate = 0.0f;
  float vel_estimate = 0.0f;

  void decode_buf(const uint8_t * buf)
  {
    pos_estimate = can_get_signal_raw<float>(buf, 0, 32, true);
    vel_estimate = can_get_signal_raw<float>(buf, 32, 32, true);
  }
};

struct GetIqMessage
{
  static constexpr uint8_t cmd_id = 0x14;
  static constexpr uint8_t msg_length = 8;

  float iq_setpoint = 0.0f;
  float iq_measured = 0.0f;

  void decode_buf(const uint8_t * buf)
  {
    iq_setpoint = can_get_signal_raw<float>(buf, 0, 32, true);
    iq_measured = can_get_signal_raw<float>(buf, 32, 32, true);
  }
};

struct GetTemperatureMessage
{
  static constexpr uint8_t cmd_id = 0x15;
  static constexpr uint8_t msg_length = 8;

  float fet_temperature = 0.0f;
  float motor_temperature = 0.0f;

  void decode_buf(const uint8_t * buf)
  {
    fet_temperature = can_get_signal_raw<float>(buf, 0, 32, true);
    motor_temperature = can_get_signal_raw<float>(buf, 32, 32, true);
  }
};

struct GetBusVoltageCurrentMessage
{
  static constexpr uint8_t cmd_id = 0x17;
  static constexpr uint8_t msg_length = 8;

  float bus_voltage = 0.0f;
  float bus_current = 0.0f;

  void decode_buf(const uint8_t * buf)
  {
    bus_voltage = can_get_signal_raw<float>(buf, 0, 32, true);
    bus_current = can_get_signal_raw<float>(buf, 32, 32, true);
  }
};

struct GetTorquesMessage
{
  static constexpr uint8_t cmd_id = 0x1C;
  static constexpr uint8_t msg_length = 8;

  float torque_target = 0.0f;
  float torque_estimate = 0.0f;

  void decode_buf(const uint8_t * buf)
  {
    torque_target = can_get_signal_raw<float>(buf, 0, 32, true);
    torque_estimate = can_get_signal_raw<float>(buf, 32, 32, true);
  }
};

struct SetControllerModeMessage
{
  static constexpr uint8_t cmd_id = 0x0B;
  static constexpr uint8_t msg_length = 8;

  uint32_t control_mode = 0;
  uint32_t input_mode = 0;

  void encode_buf(uint8_t * buf) const
  {
    std::memset(buf, 0, msg_length);
    can_set_signal_raw<uint32_t>(buf, control_mode, 0, 32, true);
    can_set_signal_raw<uint32_t>(buf, input_mode, 32, 32, true);
  }
};

struct SetInputVelMessage
{
  static constexpr uint8_t cmd_id = 0x0D;
  static constexpr uint8_t msg_length = 8;

  float input_vel = 0.0f;
  float input_torque_ff = 0.0f;

  void encode_buf(uint8_t * buf) const
  {
    std::memset(buf, 0, msg_length);
    can_set_signal_raw<float>(buf, input_vel, 0, 32, true);
    can_set_signal_raw<float>(buf, input_torque_ff, 32, 32, true);
  }
};

struct ClearErrorsMessage
{
  static constexpr uint8_t cmd_id = 0x18;
  static constexpr uint8_t msg_length = 1;

  uint8_t identify = 0;

  void encode_buf(uint8_t * buf) const
  {
    std::memset(buf, 0, 8);
    can_set_signal_raw<uint8_t>(buf, identify, 0, 8, true);
  }
};

double parse_positive_motor_direction_sign(const std::string & direction, const std::string & joint_name)
{
  std::string normalized = direction;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  if (normalized == "CCW") {
    return 1.0;
  }
  if (normalized == "CW") {
    return -1.0;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("ODriveHardwareInterface"),
    "Unknown positive_motor_direction '%s' for joint '%s'. Falling back to CCW.",
    direction.c_str(),
    joint_name.c_str());
  return 1.0;
}

YAML::Node load_hardware_config(const std::string & package_name, const std::string & config_file_name)
{
  const auto package_share = ament_index_cpp::get_package_share_directory(package_name);
  const auto config_path = package_share + "/config/" + config_file_name;

  try {
    std::ifstream config_stream(config_path);
    if (!config_stream.is_open()) {
      throw std::runtime_error("could not open file");
    }

    const YAML::Node root = YAML::Load(config_stream);
    if (!root || !root.IsMap()) {
      throw std::runtime_error("config root must be a YAML map");
    }

    if (root["/**"] && root["/**"].IsMap() &&
      root["/**"]["ros__parameters"] && root["/**"]["ros__parameters"].IsMap())
    {
      return root["/**"]["ros__parameters"];
    }

    if (root["ros__parameters"] && root["ros__parameters"].IsMap()) {
      return root["ros__parameters"];
    }

    if (root.size() == 1) {
      const YAML::Node nested_root = root.begin()->second;
      if (nested_root && nested_root.IsMap() &&
        nested_root["ros__parameters"] && nested_root["ros__parameters"].IsMap())
      {
        return nested_root["ros__parameters"];
      }
    }

    return root;
  } catch (const YAML::ParserException & error) {
    throw std::runtime_error("Failed to parse YAML in " + config_path + ": " + error.what());
  } catch (const YAML::BadConversion & error) {
    throw std::runtime_error("Invalid value type in " + config_path + ": " + error.what());
  } catch (const std::exception & error) {
    throw std::runtime_error("Failed to load hardware config from " + config_path + ": " + error.what());
  }
}

std::string load_required_string(
  const YAML::Node & root, const std::string & key, const std::string & config_label)
{
  if (!root[key]) {
    throw std::runtime_error(config_label + " is missing required key '" + key + "'");
  }
  return root[key].as<std::string>();
}

std::string load_required_string_with_alias(
  const YAML::Node & root, const std::string & key, const std::string & alias,
  const std::string & config_label)
{
  if (root[key]) {
    return root[key].as<std::string>();
  }
  if (root[alias]) {
    return root[alias].as<std::string>();
  }

  throw std::runtime_error(
          config_label + " is missing required key '" + key + "'"
          " (legacy alias '" + alias + "' is also absent)");
}

uint32_t load_required_uint32(
  const YAML::Node & root, const std::string & key, const std::string & config_label)
{
  if (!root[key]) {
    throw std::runtime_error(config_label + " is missing required key '" + key + "'");
  }
  return root[key].as<uint32_t>();
}

int load_optional_int(const YAML::Node & root, const std::string & key, int fallback)
{
  if (!root[key]) {
    return fallback;
  }
  return root[key].as<int>();
}

double load_required_positive_double(
  const YAML::Node & root, const std::string & key, const std::string & config_label)
{
  if (!root[key]) {
    throw std::runtime_error(config_label + " is missing required key '" + key + "'");
  }

  const double value = root[key].as<double>();
  if (value <= 0.0) {
    throw std::runtime_error(config_label + " key '" + key + "' must be positive");
  }
  return value;
}

class EpollEventLoop
{
public:
  using Callback = std::function<void(uint32_t)>;

  struct EventContext
  {
    int fd = -1;
    Callback callback;
  };

  using EventId = EventContext *;

  EpollEventLoop()
  : epoll_fd_(epoll_create1(0))
  {
  }

  ~EpollEventLoop()
  {
    if (epoll_fd_ >= 0) {
      close(epoll_fd_);
    }
  }

  bool register_event(EventId * evt_id, int fd, uint32_t events, const Callback & callback)
  {
    auto * ctx = new EventContext{fd, callback};
    struct epoll_event evt {};
    evt.events = events;
    evt.data.ptr = ctx;

    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &evt) == -1) {
      delete ctx;
      return false;
    }

    if (evt_id != nullptr) {
      *evt_id = ctx;
    }

    ++registered_events_;
    return true;
  }

  bool deregister_event(EventId evt_id)
  {
    if (evt_id == nullptr) {
      return false;
    }

    if (epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, evt_id->fd, nullptr) == -1) {
      return false;
    }

    for (int i = 0; i < triggered_events_count_; ++i) {
      if (static_cast<EventContext *>(triggered_events_[i].data.ptr) == evt_id) {
        triggered_events_[i].data.ptr = nullptr;
      }
    }

    delete evt_id;
    --registered_events_;
    return true;
  }

private:
  int epoll_fd_ = -1;
  size_t registered_events_ = 0;
  int triggered_events_count_ = 0;
  static constexpr size_t kMaxEventsPerIteration = 16;
  struct epoll_event triggered_events_[kMaxEventsPerIteration] {};
};

class SocketCanInterface
{
public:
  using FrameProcessor = std::function<void(const can_frame &)>;

  bool init(
    const std::string & interface_name,
    EpollEventLoop * event_loop,
    FrameProcessor frame_processor)
  {
    interface_name_ = interface_name;
    event_loop_ = event_loop;
    frame_processor_ = std::move(frame_processor);
    broken_ = false;
    last_error_message_.clear();

    socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd_ == -1) {
      last_error_message_ =
        "Failed to create SocketCAN socket on '" + interface_name_ + "': " + std::strerror(errno);
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Failed to create SocketCAN socket on '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
      return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) == -1) {
      last_error_message_ =
        "Failed to resolve SocketCAN interface '" + interface_name_ + "': " + std::strerror(errno);
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Failed to resolve SocketCAN interface '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == -1) {
      last_error_message_ =
        "Failed to bind SocketCAN interface '" + interface_name_ + "': " + std::strerror(errno);
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Failed to bind SocketCAN interface '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    if (!event_loop_->register_event(
        &socket_event_id_, socket_fd_, EPOLLIN,
        [this](uint32_t mask) { on_socket_event(mask); }))
    {
      last_error_message_ =
        "Failed to register SocketCAN fd for interface '" + interface_name_ + "' with event loop";
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Failed to register SocketCAN fd for interface '%s' with event loop",
        interface_name_.c_str());
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    return true;
  }

  bool is_ready() const
  {
    return socket_fd_ >= 0 && !broken_;
  }

  std::string last_error() const
  {
    return last_error_message_;
  }

  void deinit()
  {
    if (socket_fd_ < 0) {
      return;
    }

    if (!broken_ && event_loop_ != nullptr) {
      event_loop_->deregister_event(socket_event_id_);
    }

    close(socket_fd_);
    socket_fd_ = -1;
    broken_ = true;
  }

  bool send_can_frame(const can_frame & frame)
  {
    if (socket_fd_ < 0) {
      last_error_message_ = "SocketCAN interface '" + interface_name_ + "' is not connected";
      return false;
    }

    if (write(socket_fd_, &frame, sizeof(frame)) == -1) {
      last_error_message_ =
        "Failed to send CAN frame on '" + interface_name_ + "': " + std::strerror(errno);
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Failed to send CAN frame on '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
      deinit();
      return false;
    }

    return true;
  }

  bool read_nonblocking()
  {
    if (socket_fd_ < 0) {
      return false;
    }

    struct can_frame frame {};
    struct cmsghdr control_message {};
    struct iovec payload {.iov_base = &frame, .iov_len = sizeof(frame)};
    struct msghdr message {
      .msg_name = nullptr,
      .msg_namelen = 0,
      .msg_iov = &payload,
      .msg_iovlen = 1,
      .msg_control = &control_message,
      .msg_controllen = sizeof(control_message),
      .msg_flags = 0,
    };

    const ssize_t bytes_received = recvmsg(socket_fd_, &message, MSG_DONTWAIT);
    if (bytes_received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }

      last_error_message_ =
        "SocketCAN read failed on '" + interface_name_ + "': " + std::strerror(errno);
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "SocketCAN read failed on '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
      deinit();
      return false;
    }

    if (bytes_received < static_cast<ssize_t>(sizeof(struct can_frame))) {
      last_error_message_ =
        "Invalid CAN frame length on '" + interface_name_ + "': " + std::to_string(bytes_received);
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Invalid CAN frame length on '%s': %zd",
        interface_name_.c_str(), bytes_received);
      return true;
    }

    frame_processor_(frame);
    return true;
  }

private:
  void on_socket_event(uint32_t mask)
  {
    if (mask & EPOLLIN) {
      while (read_nonblocking() && !broken_) {
      }
    }

    if (mask & EPOLLERR) {
      last_error_message_ = "SocketCAN interface '" + interface_name_ + "' disappeared";
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "SocketCAN interface '%s' disappeared",
        interface_name_.c_str());
      deinit();
      return;
    }
  }

  std::string interface_name_;
  int socket_fd_ = -1;
  EpollEventLoop * event_loop_ = nullptr;
  EpollEventLoop::EventId socket_event_id_ = nullptr;
  FrameProcessor frame_processor_;
  bool broken_ = false;
  std::string last_error_message_;
};

template<typename TMessage>
bool send_axis_message(SocketCanInterface & can_intf, uint32_t node_id, const TMessage & msg)
{
  struct can_frame frame {};
  frame.can_id = (node_id << 5) | TMessage::cmd_id;
  frame.can_dlc = TMessage::msg_length;
  msg.encode_buf(frame.data);
  return can_intf.send_can_frame(frame);
}

bool send_axis_rtr(SocketCanInterface & can_intf, uint32_t node_id, uint8_t cmd_id)
{
  struct can_frame frame {};
  frame.can_id = ((node_id << 5) | cmd_id) | CAN_RTR_FLAG;
  frame.can_dlc = 0;
  return can_intf.send_can_frame(frame);
}

constexpr std::array<amr_sweeper_odrive::ProtectionType, 5> kProtectionTypes = {
  amr_sweeper_odrive::ProtectionType::OverTorque,
  amr_sweeper_odrive::ProtectionType::OverSpeed,
  amr_sweeper_odrive::ProtectionType::OverTemperature,
  amr_sweeper_odrive::ProtectionType::OverCurrent,
  amr_sweeper_odrive::ProtectionType::OverVoltage,
};

std::size_t protectionIndex(amr_sweeper_odrive::ProtectionType type)
{
  return static_cast<std::size_t>(type);
}

const char * protectionKey(amr_sweeper_odrive::ProtectionType type)
{
  switch (type) {
    case amr_sweeper_odrive::ProtectionType::OverTorque:
      return "over_torque";
    case amr_sweeper_odrive::ProtectionType::OverSpeed:
      return "over_speed";
    case amr_sweeper_odrive::ProtectionType::OverTemperature:
      return "over_temperature";
    case amr_sweeper_odrive::ProtectionType::OverCurrent:
      return "over_current";
    case amr_sweeper_odrive::ProtectionType::OverVoltage:
      return "over_voltage";
    case amr_sweeper_odrive::ProtectionType::Count:
      break;
  }
  return "unknown";
}

double faultTypeStateValue(const std::optional<amr_sweeper_odrive::ProtectionFault> & fault)
{
  if (!fault) {
    return -1.0;
  }
  return static_cast<double>(static_cast<int>(fault->type));
}

std::optional<double> selectMeasuredValue(
  amr_sweeper_odrive::ProtectionType type,
  const amr_sweeper_odrive::JointTelemetry & telemetry)
{
  switch (type) {
    case amr_sweeper_odrive::ProtectionType::OverTorque:
      if (!telemetry.has_torque_estimate) {
        return std::nullopt;
      }
      return std::fabs(telemetry.torque_estimate);
    case amr_sweeper_odrive::ProtectionType::OverSpeed:
      if (!telemetry.has_speed) {
        return std::nullopt;
      }
      return std::fabs(telemetry.speed_rad_s);
    case amr_sweeper_odrive::ProtectionType::OverTemperature:
      if (!telemetry.has_motor_temperature && !telemetry.has_controller_temperature) {
        return std::nullopt;
      }
      return telemetry.has_motor_temperature ?
        telemetry.motor_temperature_c : telemetry.controller_temperature_c;
    case amr_sweeper_odrive::ProtectionType::OverCurrent:
      if (!telemetry.has_current) {
        return std::nullopt;
      }
      return std::fabs(telemetry.current_a);
    case amr_sweeper_odrive::ProtectionType::OverVoltage:
      if (!telemetry.has_voltage) {
        return std::nullopt;
      }
      return telemetry.voltage_v;
    case amr_sweeper_odrive::ProtectionType::Count:
      break;
  }
  return std::nullopt;
}

amr_sweeper_odrive::ProtectionLimit load_protection_limit(
  const YAML::Node & defaults_node,
  const YAML::Node & override_node,
  amr_sweeper_odrive::ProtectionType type)
{
  amr_sweeper_odrive::ProtectionLimit limit;
  const std::string key = protectionKey(type);

  const YAML::Node default_limit = defaults_node && defaults_node[key] ? defaults_node[key] : YAML::Node();
  const YAML::Node joint_limit = override_node && override_node[key] ? override_node[key] : YAML::Node();

  auto read_bool = [](const YAML::Node & preferred, const YAML::Node & fallback, const char * field, bool default_value) {
      if (preferred && preferred[field]) {
        return preferred[field].as<bool>();
      }
      if (fallback && fallback[field]) {
        return fallback[field].as<bool>();
      }
      return default_value;
    };
  auto read_double = [](const YAML::Node & preferred, const YAML::Node & fallback, const char * field, double default_value) {
      if (preferred && preferred[field]) {
        return preferred[field].as<double>();
      }
      if (fallback && fallback[field]) {
        return fallback[field].as<double>();
      }
      return default_value;
    };
  auto read_string = [](const YAML::Node & preferred, const YAML::Node & fallback, const char * field, const std::string & default_value) {
      if (preferred && preferred[field]) {
        return preferred[field].as<std::string>();
      }
      if (fallback && fallback[field]) {
        return fallback[field].as<std::string>();
      }
      return default_value;
    };

  limit.enabled = read_bool(joint_limit, default_limit, "enabled", false);
  limit.threshold = read_double(joint_limit, default_limit, "threshold", 0.0);
  limit.units = read_string(joint_limit, default_limit, "units", std::string{});
  limit.trip_duration = std::chrono::milliseconds(
    static_cast<int>(read_double(joint_limit, default_limit, "trip_duration_ms", 0.0)));
  return limit;
}

}  // namespace

namespace amr_sweeper_odrive
{

struct ODriveHardwareInterface::Impl
{
  EpollEventLoop event_loop;
  SocketCanInterface can_intf;
  rclcpp::Time timestamp;
};

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

ODriveHardwareInterface::~ODriveHardwareInterface() = default;

hardware_interface::CallbackReturn ODriveHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  impl_ = std::make_unique<Impl>();

  hw_name_ = info_.name;
  num_joints_ = static_cast<uint8_t>(info_.joints.size());

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Name: %s", hw_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Number of Joints %u", num_joints_);

  velocity_commands_.resize(num_joints_);
  prev_velocity_commands_.resize(num_joints_);
  velocity_states_.resize(num_joints_);
  position_states_.resize(num_joints_);
  effort_states_.assign(num_joints_, 0.0);
  torque_states_.assign(num_joints_, 0.0);
  current_states_.assign(num_joints_, 0.0);
  temperature_states_.assign(num_joints_, 0.0);
  voltage_states_.assign(num_joints_, 0.0);
  fault_latched_states_.assign(num_joints_, 0.0);
  fault_type_states_.assign(num_joints_, -1.0);
  fault_measured_states_.assign(num_joints_, 0.0);
  fault_threshold_states_.assign(num_joints_, 0.0);
  positive_motor_direction_signs_.resize(num_joints_, 1.0);
  gear_ratios_.resize(num_joints_, 1.0);
  node_ids_.resize(num_joints_);
  joint_telemetry_.resize(num_joints_);
  protection_states_.resize(num_joints_);

  if (num_joints_ != 2) {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Incorrect number of joints");
    return CallbackReturn::ERROR;
  }

  try {
    if (validateJoints() != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    const YAML::Node hardware_config =
      load_hardware_config("amr_sweeper_odrive", "amr_sweeper_odrive.yaml");
    const double shared_gear_ratio =
      load_required_positive_double(hardware_config, "gear_ratio", "amr_sweeper_odrive.yaml");
    can_interface_ =
      load_required_string(hardware_config, "can_interface", "amr_sweeper_odrive.yaml");
    const std::array<uint32_t, 2> config_node_ids = {
      load_required_uint32(hardware_config, "left_motor_id", "amr_sweeper_odrive.yaml"),
      load_required_uint32(hardware_config, "right_motor_id", "amr_sweeper_odrive.yaml"),
    };
    const std::array<std::string, 2> config_directions = {
      load_required_string_with_alias(
        hardware_config, "left_motor_positive_direction", "left_positive_motor_direction",
        "amr_sweeper_odrive.yaml"),
      load_required_string_with_alias(
        hardware_config, "right_motor_positive_direction", "right_positive_motor_direction",
        "amr_sweeper_odrive.yaml"),
    };

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & joint = info_.joints[i];
      positive_motor_direction_signs_[i] =
        parse_positive_motor_direction_sign(config_directions[i], joint.name);
      gear_ratios_[i] = shared_gear_ratio;
      node_ids_[i] = config_node_ids[i];
    }
    reconnect_attempt_interval_ms_ =
      load_optional_int(hardware_config, "reconnect_attempt_interval_ms", reconnect_attempt_interval_ms_);
    retry_attempts_before_error_ =
      load_optional_int(hardware_config, "retry_attempts_before_error", retry_attempts_before_error_);
    fatal_after_consecutive_errors_ = load_optional_int(
      hardware_config, "fatal_after_consecutive_errors", fatal_after_consecutive_errors_);
    max_reconnect_attempts_ =
      load_optional_int(hardware_config, "max_reconnect_attempts", max_reconnect_attempts_);
    loadProtectionParameters();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Error parsing parameter: %s", error.what());
    return CallbackReturn::ERROR;
  }

  if (reconnect_attempt_interval_ms_ < 1) {
    reconnect_attempt_interval_ms_ = 1;
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

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!safety_stop_publisher_) {
    if (auto node = get_node()) {
      safety_stop_publisher_ = node->create_publisher<amr_sweeper_safety_msgs::msg::SafetyStop>(
        safety_stop_topic_name_, rclcpp::SystemDefaultsQoS());
    }
  }
  if (!clear_safety_stop_service_) {
    if (auto node = get_node()) {
      clear_safety_stop_service_ = node->create_service<std_srvs::srv::Trigger>(
        clear_safety_stop_service_name_,
        std::bind(
          &ODriveHardwareInterface::clearSafetyStopService, this,
          std::placeholders::_1, std::placeholders::_2));
    }
  }

  if (!initializeCanInterface()) {
    reportConnectionIssue(
      last_connection_error_message_.empty() ?
      "Failed to initialize SocketCAN on " + can_interface_ :
      last_connection_error_message_);
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  lifecycle_active_ = false;
  closeCanInterface();
  return CallbackReturn::SUCCESS;
}

bool ODriveHardwareInterface::initializeCanInterface()
{
  closeCanInterface();

  if (!impl_->can_intf.init(
      can_interface_, &impl_->event_loop,
      [this](const can_frame & frame) { on_can_msg(frame); }))
  {
    last_connection_error_message_ = impl_->can_intf.last_error().empty() ?
      "Failed to initialize SocketCAN on " + can_interface_ :
      impl_->can_intf.last_error();
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "Failed to initialize SocketCAN on %s",
      can_interface_.c_str());
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(hw_name_), "Initialized SocketCAN on %s", can_interface_.c_str());
  last_connection_error_message_.clear();
  last_reconnect_attempt_time_ = std::chrono::steady_clock::now();
  resetIssueCounters();
  return true;
}

void ODriveHardwareInterface::closeCanInterface()
{
  if (impl_) {
    impl_->can_intf.deinit();
  }
}

bool ODriveHardwareInterface::ensureCanInterface()
{
  if (fatal_error_) {
    return false;
  }

  if (impl_->can_intf.is_ready()) {
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  if (last_reconnect_attempt_time_.time_since_epoch().count() != 0) {
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reconnect_attempt_time_).count();
    if (elapsed_ms < reconnect_attempt_interval_ms_) {
      return false;
    }
  }
  last_reconnect_attempt_time_ = now;

  if (!initializeCanInterface()) {
    reportConnectionIssue(
      last_connection_error_message_.empty() ?
      "Failed to initialize SocketCAN on " + can_interface_ :
      last_connection_error_message_);
    return false;
  }

  if (lifecycle_active_) {
    for (size_t i = 0; i < num_joints_; ++i) {
      configureAxisForVelocity(i);
      (void)sendVelocityCommand(i, 0.0);
    }
  }
  return true;
}

void ODriveHardwareInterface::reportConnectionIssue(const std::string & message)
{
  ++connection_issue_count_;
  ++reconnect_attempt_count_;

  if (max_reconnect_attempts_ > 0 && reconnect_attempt_count_ >= max_reconnect_attempts_) {
    fatal_error_ = true;
    RCLCPP_FATAL(
      rclcpp::get_logger(hw_name_),
      "%s. Reached reconnect limit after %d attempts",
      message.c_str(),
      reconnect_attempt_count_);
    return;
  }

  logEscalatingIssue(connection_issue_count_, message);
}

void ODriveHardwareInterface::logEscalatingIssue(int count, const std::string & message)
{
  if (count < retry_attempts_before_error_) {
    RCLCPP_WARN(rclcpp::get_logger(hw_name_), "%s", message.c_str());
    return;
  }

  if (count < fatal_after_consecutive_errors_) {
    if (count == retry_attempts_before_error_) {
      RCLCPP_ERROR(
        rclcpp::get_logger(hw_name_),
        "%s. Escalating after %d consecutive failures",
        message.c_str(),
        count);
      return;
    }
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "%s", message.c_str());
    return;
  }

  fatal_error_ = true;
  RCLCPP_FATAL(
    rclcpp::get_logger(hw_name_),
    "%s. Reached fatal threshold after %d consecutive connection failures",
    message.c_str(),
    count);
}

void ODriveHardwareInterface::resetIssueCounters()
{
  reconnect_attempt_count_ = 0;
  connection_issue_count_ = 0;
  fatal_error_ = false;
}

void ODriveHardwareInterface::loadProtectionParameters()
{
  const YAML::Node hardware_config =
    load_hardware_config("amr_sweeper_odrive", "amr_sweeper_odrive.yaml");
  const YAML::Node protection_node = hardware_config["protection"];

  if (!protection_node || !protection_node.IsMap()) {
    return;
  }

  protection_enabled_ = protection_node["enabled"] ? protection_node["enabled"].as<bool>() : false;
  clear_faults_on_activate_ =
    protection_node["clear_faults_on_activate"] ?
    protection_node["clear_faults_on_activate"].as<bool>() : true;
  latch_faults_ = protection_node["latch_faults"] ? protection_node["latch_faults"].as<bool>() : true;
  command_deadband_for_checks_ =
    protection_node["command_deadband_for_checks"] ?
    protection_node["command_deadband_for_checks"].as<double>() : 0.0;
  startup_ignore_duration_ = std::chrono::milliseconds(
    protection_node["startup_ignore_ms"] ? protection_node["startup_ignore_ms"].as<int>() : 500);
  safety_stop_topic_name_ =
    protection_node["safety_stop_topic_name"] ?
    protection_node["safety_stop_topic_name"].as<std::string>() : "safety_msgs/stop";
  safety_stop_sender_name_ =
    protection_node["safety_stop_sender_name"] ?
    protection_node["safety_stop_sender_name"].as<std::string>() : "odrive_hardware_interface";
  clear_safety_stop_service_name_ =
    protection_node["clear_safety_stop_service_name"] ?
    protection_node["clear_safety_stop_service_name"].as<std::string>() :
    "/odrive_ros2_control/clear_safety_stop";

  const YAML::Node defaults_node = protection_node["defaults"];
  const YAML::Node joint_overrides_node =
    protection_node["joint_overrides"] ? protection_node["joint_overrides"] : protection_node["joints"];

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    protection_states_[i].joint_name = info_.joints[i].name;
    const YAML::Node joint_override =
      joint_overrides_node && joint_overrides_node[info_.joints[i].name] ?
      joint_overrides_node[info_.joints[i].name] : YAML::Node();

    for (const auto type : kProtectionTypes) {
      protection_states_[i].limits[protectionIndex(type)] =
        load_protection_limit(defaults_node, joint_override, type);
    }
  }
}

void ODriveHardwareInterface::clearProtectionFaults()
{
  for (std::size_t i = 0; i < protection_states_.size(); ++i) {
    protection_states_[i].fault.reset();
    protection_states_[i].safe_stop_sent = false;
    protection_states_[i].safety_stop_published = false;
    protection_states_[i].over_threshold_seconds.fill(0.0);
    updateProtectionStatusState(i);
  }
}

void ODriveHardwareInterface::clearSafetyStopService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  if (!ensureCanInterface()) {
    response->success = false;
    response->message = last_connection_error_message_.empty() ?
      "ODrive CAN interface is not ready" : last_connection_error_message_;
    return;
  }

  clearProtectionFaults();

  for (std::size_t i = 0; i < num_joints_; ++i) {
    velocity_commands_[i] = 0.0;
    prev_velocity_commands_[i] = 0.0;
    configureAxisForVelocity(i);
    if (!sendVelocityCommand(i, 0.0)) {
      response->success = false;
      response->message = impl_->can_intf.last_error().empty() ?
        "Failed to send zero ODrive velocity command" : impl_->can_intf.last_error();
      return;
    }
  }

  response->success = true;
  response->message = "ODrive safety stop cleared and axes re-enabled";
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Starting ...please wait...");
  lifecycle_active_ = true;
  activation_time_ = impl_->timestamp.nanoseconds() == 0 ? rclcpp::Clock(RCL_ROS_TIME).now() : impl_->timestamp;
  if (clear_faults_on_activate_) {
    clearProtectionFaults();
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    velocity_commands_[i] = 0.0;
    prev_velocity_commands_[i] = 0.0;
    position_states_[i] = 0.0;
    velocity_states_[i] = 0.0;

    if (ensureCanInterface()) {
      configureAxisForVelocity(i);
      (void)sendVelocityCommand(i, 0.0);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");
  lifecycle_active_ = false;

  if (impl_->can_intf.is_ready()) {
    for (size_t i = 0; i < num_joints_; ++i) {
      (void)sendVelocityCommand(i, 0.0);
      requestAxisIdle(i);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System successfully stopped!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::validateJoints()
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu state interface. At least 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    const auto has_position = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface_info) {
        return interface_info.name == hardware_interface::HW_IF_POSITION;
      });
    const auto has_velocity = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface_info) {
        return interface_info.name == hardware_interface::HW_IF_VELOCITY;
      });
    if (!has_position || !has_velocity) {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' must expose at least '%s' and '%s' state interfaces.",
        joint.name.c_str(),
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < num_joints_; ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "torque", &torque_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "current", &current_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "temperature", &temperature_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "voltage", &voltage_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "protection_fault_latched", &fault_latched_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "protection_fault_type", &fault_type_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "protection_fault_measured", &fault_measured_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "protection_fault_threshold", &fault_threshold_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type ODriveHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    const std::string velocity_key = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;

    if (std::find(start_interfaces.begin(), start_interfaces.end(), velocity_key) != start_interfaces.end()) {
      configureAxisForVelocity(i);
      (void)sendVelocityCommand(i, velocity_commands_[i]);
    }

    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), velocity_key) != stop_interfaces.end()) {
      (void)sendVelocityCommand(i, 0.0);
      requestAxisIdle(i);
    }
  }

  return return_type::OK;
}

void ODriveHardwareInterface::configureAxisForVelocity(size_t joint_index)
{
  SetControllerModeMessage control_mode_msg;
  control_mode_msg.control_mode = CONTROL_MODE_VELOCITY_CONTROL;
  control_mode_msg.input_mode = INPUT_MODE_PASSTHROUGH;
  (void)send_axis_message(impl_->can_intf, node_ids_[joint_index], control_mode_msg);

  ClearErrorsMessage clear_errors_msg;
  clear_errors_msg.identify = 0;
  (void)send_axis_message(impl_->can_intf, node_ids_[joint_index], clear_errors_msg);

  SetAxisStateMessage axis_state_msg;
  axis_state_msg.axis_requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  (void)send_axis_message(impl_->can_intf, node_ids_[joint_index], axis_state_msg);
}

void ODriveHardwareInterface::requestAxisIdle(size_t joint_index)
{
  SetAxisStateMessage axis_state_msg;
  axis_state_msg.axis_requested_state = AXIS_STATE_IDLE;
  (void)send_axis_message(impl_->can_intf, node_ids_[joint_index], axis_state_msg);
}

bool ODriveHardwareInterface::requestAxisTelemetry(size_t joint_index, uint8_t cmd_id)
{
  return send_axis_rtr(impl_->can_intf, node_ids_[joint_index], cmd_id);
}

bool ODriveHardwareInterface::sendVelocityCommand(size_t joint_index, double joint_velocity_rad_s)
{
  const double motor_velocity_rad_s =
    joint_velocity_rad_s * gear_ratios_[joint_index] * positive_motor_direction_signs_[joint_index];

  SetInputVelMessage velocity_msg;
  velocity_msg.input_vel = static_cast<float>(motor_velocity_rad_s / TWO_PI);
  velocity_msg.input_torque_ff = 0.0f;
  return send_axis_message(impl_->can_intf, node_ids_[joint_index], velocity_msg);
}

void ODriveHardwareInterface::writeCommandsToHardware()
{
  for (size_t i = 0; i < num_joints_; ++i) {
    if (motorHasLatchedFault(i)) {
      stopOrDisableMotor(i);
      prev_velocity_commands_[i] = 0.0;
      continue;
    }

    if (!sendVelocityCommand(i, velocity_commands_[i])) {
      last_connection_error_message_ = impl_->can_intf.last_error().empty() ?
        "Failed to send ODrive velocity command on " + can_interface_ :
        impl_->can_intf.last_error();
      reportConnectionIssue(last_connection_error_message_);
      return;
    }
    prev_velocity_commands_[i] = velocity_commands_[i];
  }
  resetIssueCounters();
}

void ODriveHardwareInterface::requestTelemetryUpdates()
{
  for (std::size_t i = 0; i < num_joints_; ++i) {
    bool needs_iq = false;
    bool needs_temperature = false;
    bool needs_voltage = false;
    bool needs_torque = false;

    for (const auto type : kProtectionTypes) {
      const auto & limit = protection_states_[i].limits[protectionIndex(type)];
      if (!limit.enabled) {
        continue;
      }

      switch (type) {
        case ProtectionType::OverTorque:
          needs_torque = true;
          break;
        case ProtectionType::OverSpeed:
          break;
        case ProtectionType::OverTemperature:
          needs_temperature = true;
          break;
        case ProtectionType::OverCurrent:
          needs_iq = true;
          break;
        case ProtectionType::OverVoltage:
          needs_voltage = true;
          break;
        case ProtectionType::Count:
          break;
      }
    }

    if (needs_iq) {
      (void)requestAxisTelemetry(i, GetIqMessage::cmd_id);
    }
    if (needs_temperature) {
      (void)requestAxisTelemetry(i, GetTemperatureMessage::cmd_id);
    }
    if (needs_voltage) {
      (void)requestAxisTelemetry(i, GetBusVoltageCurrentMessage::cmd_id);
    }
    if (needs_torque) {
      (void)requestAxisTelemetry(i, GetTorquesMessage::cmd_id);
    }
  }
}

void ODriveHardwareInterface::updateJointsFromHardware()
{
  if (!impl_->can_intf.is_ready()) {
    return;
  }
  while (impl_->can_intf.read_nonblocking()) {
  }
  if (!impl_->can_intf.is_ready()) {
    last_connection_error_message_ = impl_->can_intf.last_error().empty() ?
      "SocketCAN interface '" + can_interface_ + "' disconnected" :
      impl_->can_intf.last_error();
    reportConnectionIssue(last_connection_error_message_);
  }

  requestTelemetryUpdates();
}

void ODriveHardwareInterface::updateProtectionStatusState(size_t joint_index)
{
  const auto & telemetry = joint_telemetry_[joint_index];
  const auto & state = protection_states_[joint_index];

  torque_states_[joint_index] = telemetry.has_torque_estimate ? telemetry.torque_estimate : 0.0;
  current_states_[joint_index] = telemetry.has_current ? telemetry.current_a : 0.0;
  temperature_states_[joint_index] =
    telemetry.has_motor_temperature ? telemetry.motor_temperature_c :
    (telemetry.has_controller_temperature ? telemetry.controller_temperature_c : 0.0);
  voltage_states_[joint_index] = telemetry.has_voltage ? telemetry.voltage_v : 0.0;
  effort_states_[joint_index] = telemetry.has_torque_estimate ? telemetry.torque_estimate :
    (telemetry.has_current ? telemetry.current_a : 0.0);
  fault_latched_states_[joint_index] = state.fault ? 1.0 : 0.0;
  fault_type_states_[joint_index] = faultTypeStateValue(state.fault);
  fault_measured_states_[joint_index] = state.fault ? state.fault->measured_value : 0.0;
  fault_threshold_states_[joint_index] = state.fault ? state.fault->threshold : 0.0;
}

void ODriveHardwareInterface::latchProtectionFault(
  size_t joint_index,
  ProtectionType type,
  double measured_value,
  const ProtectionLimit & limit)
{
  auto & state = protection_states_[joint_index];
  if (state.fault) {
    return;
  }

  state.fault = ProtectionFault{type, state.joint_name, measured_value, limit.threshold, limit.units};
  state.safe_stop_sent = false;

  RCLCPP_ERROR(
    rclcpp::get_logger(hw_name_),
    "Protection tripped: interface=odrive joint=%s protection=%s measured=%.3f %s threshold=%.3f %s action=\"motor stopped and fault latched\"",
    state.joint_name.c_str(),
    protectionKey(type),
    measured_value,
    limit.units.c_str(),
    limit.threshold,
    limit.units.c_str());

  if (safety_stop_publisher_ && !state.safety_stop_published) {
    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = impl_ && impl_->timestamp.nanoseconds() != 0 ?
      impl_->timestamp : get_clock()->now();
    stop_msg.sender = safety_stop_sender_name_;
    std::ostringstream reason;
    reason << "motor protection fault on " << state.joint_name
           << ": " << protectionKey(type)
           << " measured=" << measured_value << " " << limit.units
           << " threshold=" << limit.threshold << " " << limit.units;
    stop_msg.reason = reason.str();
    safety_stop_publisher_->publish(stop_msg);
    state.safety_stop_published = true;
  }

  stopOrDisableMotor(joint_index);
  updateProtectionStatusState(joint_index);
}

bool ODriveHardwareInterface::motorHasLatchedFault(size_t joint_index) const
{
  return protection_enabled_ && joint_index < protection_states_.size() && protection_states_[joint_index].fault.has_value();
}

void ODriveHardwareInterface::stopOrDisableMotor(size_t joint_index)
{
  if (joint_index >= protection_states_.size()) {
    return;
  }

  auto & state = protection_states_[joint_index];
  velocity_commands_[joint_index] = 0.0;

  if (state.safe_stop_sent) {
    return;
  }

  (void)sendVelocityCommand(joint_index, 0.0);
  requestAxisIdle(joint_index);
  state.safe_stop_sent = true;
}

void ODriveHardwareInterface::evaluateProtections(const rclcpp::Duration & period)
{
  if (!protection_enabled_) {
    for (std::size_t i = 0; i < num_joints_; ++i) {
      updateProtectionStatusState(i);
    }
    return;
  }

  const double dt = std::max(0.0, period.seconds());
  const bool in_startup_ignore =
    activation_time_.nanoseconds() != 0 &&
    (impl_->timestamp - activation_time_) < rclcpp::Duration(startup_ignore_duration_);

  for (std::size_t i = 0; i < num_joints_; ++i) {
    auto & state = protection_states_[i];
    auto & telemetry = joint_telemetry_[i];

    telemetry.speed_rad_s = velocity_states_[i];
    telemetry.has_speed = true;

    if (state.fault) {
      stopOrDisableMotor(i);
      updateProtectionStatusState(i);
      continue;
    }

    for (const auto type : kProtectionTypes) {
      const auto index = protectionIndex(type);
      const auto & limit = state.limits[index];
      if (!limit.enabled) {
        state.over_threshold_seconds[index] = 0.0;
        continue;
      }

      if (std::fabs(velocity_commands_[i]) <= command_deadband_for_checks_ &&
        (type == ProtectionType::OverTorque || type == ProtectionType::OverCurrent))
      {
        state.over_threshold_seconds[index] = 0.0;
        continue;
      }

      if (in_startup_ignore) {
        state.over_threshold_seconds[index] = 0.0;
        continue;
      }

      const auto measured = selectMeasuredValue(type, telemetry);
      if (!measured.has_value()) {
        state.over_threshold_seconds[index] = 0.0;
        continue;
      }

      if (*measured > limit.threshold) {
        state.over_threshold_seconds[index] += dt;
      } else {
        state.over_threshold_seconds[index] = 0.0;
      }

      if (state.over_threshold_seconds[index] >= (static_cast<double>(limit.trip_duration.count()) / 1000.0)) {
        latchProtectionFault(i, type, *measured, limit);
        if (!latch_faults_) {
          state.over_threshold_seconds[index] = 0.0;
        }
        break;
      }
    }

    updateProtectionStatusState(i);
  }
}

hardware_interface::return_type ODriveHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!ensureCanInterface()) {
    return fatal_error_ ? return_type::ERROR : return_type::OK;
  }
  impl_->timestamp = time;
  updateJointsFromHardware();
  evaluateProtections(period);
  return fatal_error_ ? return_type::ERROR : return_type::OK;
}

hardware_interface::return_type ODriveHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!ensureCanInterface()) {
    return fatal_error_ ? return_type::ERROR : return_type::OK;
  }
  writeCommandsToHardware();
  return fatal_error_ ? return_type::ERROR : return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame & frame)
{
  const uint32_t node_id = static_cast<uint32_t>(frame.can_id >> 5);

  for (size_t i = 0; i < node_ids_.size(); ++i) {
    if (node_ids_[i] == node_id) {
      processAxisFrame(i, frame);
      break;
    }
  }
}

void ODriveHardwareInterface::processAxisFrame(size_t joint_index, const can_frame & frame)
{
  const uint8_t cmd = static_cast<uint8_t>(frame.can_id & 0x1f);

  switch (cmd) {
    case GetEncoderEstimatesMessage::cmd_id: {
        if (frame.can_dlc < GetEncoderEstimatesMessage::msg_length) {
          RCLCPP_WARN(rclcpp::get_logger(hw_name_), "message %u too short", static_cast<unsigned>(cmd));
          return;
        }

        GetEncoderEstimatesMessage msg;
        msg.decode_buf(frame.data);

        const double motor_position_rad =
          positive_motor_direction_signs_[joint_index] * msg.pos_estimate * TWO_PI;
        const double motor_velocity_rad_s =
          positive_motor_direction_signs_[joint_index] * msg.vel_estimate * TWO_PI;

        position_states_[joint_index] = motor_position_rad / gear_ratios_[joint_index];
        velocity_states_[joint_index] = motor_velocity_rad_s / gear_ratios_[joint_index];
        joint_telemetry_[joint_index].speed_rad_s = velocity_states_[joint_index];
        joint_telemetry_[joint_index].has_speed = true;
        return;
      }
    case GetIqMessage::cmd_id: {
        if (frame.can_dlc < GetIqMessage::msg_length) {
          return;
        }
        GetIqMessage msg;
        msg.decode_buf(frame.data);
        joint_telemetry_[joint_index].current_a = msg.iq_measured;
        joint_telemetry_[joint_index].has_current = true;
        return;
      }
    case GetTemperatureMessage::cmd_id: {
        if (frame.can_dlc < GetTemperatureMessage::msg_length) {
          return;
        }
        GetTemperatureMessage msg;
        msg.decode_buf(frame.data);
        joint_telemetry_[joint_index].controller_temperature_c = msg.fet_temperature;
        joint_telemetry_[joint_index].motor_temperature_c = msg.motor_temperature;
        joint_telemetry_[joint_index].has_controller_temperature = true;
        joint_telemetry_[joint_index].has_motor_temperature = true;
        return;
      }
    case GetBusVoltageCurrentMessage::cmd_id: {
        if (frame.can_dlc < GetBusVoltageCurrentMessage::msg_length) {
          return;
        }
        GetBusVoltageCurrentMessage msg;
        msg.decode_buf(frame.data);
        joint_telemetry_[joint_index].voltage_v = msg.bus_voltage;
        joint_telemetry_[joint_index].has_voltage = true;
        return;
      }
    case GetTorquesMessage::cmd_id: {
        if (frame.can_dlc < GetTorquesMessage::msg_length) {
          return;
        }
        GetTorquesMessage msg;
        msg.decode_buf(frame.data);
        joint_telemetry_[joint_index].torque_estimate =
          static_cast<double>(positive_motor_direction_signs_[joint_index]) * msg.torque_estimate;
        joint_telemetry_[joint_index].has_torque_estimate = true;
        return;
      }
    default:
      return;
  }
}

}  // namespace amr_sweeper_odrive

PLUGINLIB_EXPORT_CLASS(
  amr_sweeper_odrive::ODriveHardwareInterface,
  hardware_interface::SystemInterface)
