#include "odrive_hardware_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
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

double load_shared_gear_ratio(const std::string & package_name, const std::string & config_file_name)
{
  try {
    const auto package_share = ament_index_cpp::get_package_share_directory(package_name);
    const auto config_path = package_share + "/config/" + config_file_name;
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node parameters = root["/**"] ? root["/**"]["ros__parameters"] : YAML::Node();

    if (!parameters || !parameters["gear_ratio"]) {
      throw std::runtime_error("missing required key 'gear_ratio'");
    }

    const double ratio = parameters["gear_ratio"].as<double>();
    if (ratio <= 0.0) {
      throw std::out_of_range("gear_ratio must be positive");
    }
    return ratio;
  } catch (const std::exception & error) {
    throw std::runtime_error(
      "Failed to load gear_ratio from " + package_name + "/config/" + config_file_name + ": " +
      error.what());
  }
}

YAML::Node load_hardware_config(const std::string & package_name, const std::string & config_file_name)
{
  try {
    const auto package_share = ament_index_cpp::get_package_share_directory(package_name);
    const auto config_path = package_share + "/config/" + config_file_name;
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node parameters = root["/**"] ? root["/**"]["ros__parameters"] : YAML::Node();
    if (!parameters) {
      throw std::runtime_error("missing required '/**/ros__parameters' block");
    }
    return parameters;
  } catch (const std::exception & error) {
    throw std::runtime_error(
      "Failed to load hardware config from " + package_name + "/config/" + config_file_name +
      ": " + error.what());
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

uint32_t load_required_uint32(
  const YAML::Node & root, const std::string & key, const std::string & config_label)
{
  if (!root[key]) {
    throw std::runtime_error(config_label + " is missing required key '" + key + "'");
  }
  return root[key].as<uint32_t>();
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

    socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd_ == -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Failed to create SocketCAN socket on '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
      return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) == -1) {
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
      return false;
    }

    if (write(socket_fd_, &frame, sizeof(frame)) == -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Failed to send CAN frame on '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
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

      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "SocketCAN read failed on '%s': %s",
        interface_name_.c_str(), std::strerror(errno));
      return false;
    }

    if (bytes_received < static_cast<ssize_t>(sizeof(struct can_frame))) {
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
};

template<typename TMessage>
void send_axis_message(SocketCanInterface & can_intf, uint32_t node_id, const TMessage & msg)
{
  struct can_frame frame {};
  frame.can_id = (node_id << 5) | TMessage::cmd_id;
  frame.can_dlc = TMessage::msg_length;
  msg.encode_buf(frame.data);
  can_intf.send_can_frame(frame);
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
  positive_motor_direction_signs_.resize(num_joints_, 1.0);
  gear_ratios_.resize(num_joints_, 1.0);
  node_ids_.resize(num_joints_);

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
      load_shared_gear_ratio("amr_sweeper_odrive", "amr_sweeper_odrive.yaml");
    can_interface_ =
      load_required_string(hardware_config, "can_interface", "amr_sweeper_odrive.yaml");
    const std::array<uint32_t, 2> config_node_ids = {
      load_required_uint32(hardware_config, "left_motor_id", "amr_sweeper_odrive.yaml"),
      load_required_uint32(hardware_config, "right_motor_id", "amr_sweeper_odrive.yaml"),
    };
    const std::array<std::string, 2> config_directions = {
      load_required_string(
        hardware_config, "left_positive_motor_direction", "amr_sweeper_odrive.yaml"),
      load_required_string(
        hardware_config, "right_positive_motor_direction", "amr_sweeper_odrive.yaml"),
    };

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & joint = info_.joints[i];
      positive_motor_direction_signs_[i] =
        parse_positive_motor_direction_sign(config_directions[i], joint.name);
      gear_ratios_[i] = shared_gear_ratio;
      node_ids_[i] = config_node_ids[i];
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Error parsing parameter: %s", error.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return initializeCanInterface() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "Failed to initialize SocketCAN on %s",
      can_interface_.c_str());
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(hw_name_), "Initialized SocketCAN on %s", can_interface_.c_str());
  return true;
}

void ODriveHardwareInterface::closeCanInterface()
{
  if (impl_) {
    impl_->can_intf.deinit();
  }
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Starting ...please wait...");

  for (size_t i = 0; i < num_joints_; ++i) {
    velocity_commands_[i] = 0.0;
    prev_velocity_commands_[i] = 0.0;
    position_states_[i] = 0.0;
    velocity_states_[i] = 0.0;

    configureAxisForVelocity(i);
    sendVelocityCommand(i, 0.0);
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");

  for (size_t i = 0; i < num_joints_; ++i) {
    sendVelocityCommand(i, 0.0);
    requestAxisIdle(i);
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

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
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
      sendVelocityCommand(i, velocity_commands_[i]);
    }

    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), velocity_key) != stop_interfaces.end()) {
      sendVelocityCommand(i, 0.0);
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
  send_axis_message(impl_->can_intf, node_ids_[joint_index], control_mode_msg);

  ClearErrorsMessage clear_errors_msg;
  clear_errors_msg.identify = 0;
  send_axis_message(impl_->can_intf, node_ids_[joint_index], clear_errors_msg);

  SetAxisStateMessage axis_state_msg;
  axis_state_msg.axis_requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  send_axis_message(impl_->can_intf, node_ids_[joint_index], axis_state_msg);
}

void ODriveHardwareInterface::requestAxisIdle(size_t joint_index)
{
  SetAxisStateMessage axis_state_msg;
  axis_state_msg.axis_requested_state = AXIS_STATE_IDLE;
  send_axis_message(impl_->can_intf, node_ids_[joint_index], axis_state_msg);
}

void ODriveHardwareInterface::sendVelocityCommand(size_t joint_index, double joint_velocity_rad_s)
{
  const double motor_velocity_rad_s =
    joint_velocity_rad_s * gear_ratios_[joint_index] * positive_motor_direction_signs_[joint_index];

  SetInputVelMessage velocity_msg;
  velocity_msg.input_vel = static_cast<float>(motor_velocity_rad_s / TWO_PI);
  velocity_msg.input_torque_ff = 0.0f;
  send_axis_message(impl_->can_intf, node_ids_[joint_index], velocity_msg);
}

void ODriveHardwareInterface::writeCommandsToHardware()
{
  for (size_t i = 0; i < num_joints_; ++i) {
    sendVelocityCommand(i, velocity_commands_[i]);
    prev_velocity_commands_[i] = velocity_commands_[i];
  }
}

void ODriveHardwareInterface::updateJointsFromHardware()
{
  while (impl_->can_intf.read_nonblocking()) {
  }
}

hardware_interface::return_type ODriveHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  impl_->timestamp = time;
  updateJointsFromHardware();
  return return_type::OK;
}

hardware_interface::return_type ODriveHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  writeCommandsToHardware();
  return return_type::OK;
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

  if (cmd != GetEncoderEstimatesMessage::cmd_id) {
    return;
  }

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
}

}  // namespace amr_sweeper_odrive

PLUGINLIB_EXPORT_CLASS(
  amr_sweeper_odrive::ODriveHardwareInterface,
  hardware_interface::SystemInterface)
