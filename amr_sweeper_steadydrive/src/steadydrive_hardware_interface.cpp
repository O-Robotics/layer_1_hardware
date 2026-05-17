#include "steadydrive_hardware_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace {
constexpr int LEFT_MOTOR_INDEX = 0;
constexpr int RIGHT_MOTOR_INDEX = 1;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double TWO_PI = 2.0 * M_PI;
constexpr double ENCODER_COUNTS_PER_REV = 16384.0;
constexpr double RAD_PER_COUNT = TWO_PI / ENCODER_COUNTS_PER_REV;
constexpr int32_t MAX_SPEED_COMMAND = 7200000;
constexpr int32_t MIN_SPEED_COMMAND = -7200000;

uint32_t parse_can_id(const std::string & value, const std::string & parameter_name)
{
  try {
    return static_cast<uint32_t>(std::stoul(value, nullptr, 0));
  } catch (const std::exception & error) {
    throw std::runtime_error("Invalid " + parameter_name + " '" + value + "': " + error.what());
  }
}

int16_t decode_signed_16bit(uint8_t low_byte, uint8_t high_byte)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(low_byte) | (static_cast<uint16_t>(high_byte) << 8));
}

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
    rclcpp::get_logger("SteadydriveHardwareInterface"),
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
}  // namespace

using amr_sweeper_steadydrive::SteadydriveHardwareInterface;
hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_name_ = info_.name;
  num_joints_ = info_.joints.size();

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Name: %s", hw_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Number of Joints %u", num_joints_);


  velocity_commands_.resize(num_joints_);
  prev_velocity_commands_.resize(num_joints_);
  velocity_states_.resize(num_joints_);
  position_states_.resize(num_joints_);
  positive_motor_direction_signs_.resize(num_joints_, 1.0);
  gear_ratios_.resize(num_joints_, 1.0);
  motor_can_ids_.resize(num_joints_);
  can_sockets_.assign(num_joints_, -1);
  last_encoder_position_raw_.resize(num_joints_);
  accumulated_motor_position_rad_.resize(num_joints_, 0.0);

  if (num_joints_ != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Incorrect number of joints");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    if (validateJoints() != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const YAML::Node hardware_config =
      load_hardware_config("amr_sweeper_steadydrive", "amr_sweeper_steadydrive.yaml");
    const double shared_gear_ratio =
      load_required_positive_double(
        hardware_config, "gear_ratio", "amr_sweeper_steadydrive.yaml");
    can_interface_ = load_required_string(
      hardware_config, "can_interface", "amr_sweeper_steadydrive.yaml");
    const std::array<std::string, 2> config_directions = {
      load_required_string(
        hardware_config, "left_positive_motor_direction", "amr_sweeper_steadydrive.yaml"),
      load_required_string(
        hardware_config, "right_positive_motor_direction", "amr_sweeper_steadydrive.yaml"),
    };
    const std::array<std::string, 2> config_can_ids = {
      load_required_string(
        hardware_config, "left_motor_id", "amr_sweeper_steadydrive.yaml"),
      load_required_string(
        hardware_config, "right_motor_id", "amr_sweeper_steadydrive.yaml"),
    };

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & joint = info_.joints[i];
      positive_motor_direction_signs_[i] =
        parse_positive_motor_direction_sign(config_directions[i], joint.name);
      gear_ratios_[i] = shared_gear_ratio;
    }
    motor_can_ids_[LEFT_MOTOR_INDEX] = parse_can_id(config_can_ids[LEFT_MOTOR_INDEX], "left_motor_id");
    motor_can_ids_[RIGHT_MOTOR_INDEX] = parse_can_id(config_can_ids[RIGHT_MOTOR_INDEX], "right_motor_id");

    RCLCPP_INFO(
      rclcpp::get_logger(hw_name_),
      "Loaded CAN parameters: can_interface=%s, left_motor_can_id=0x%03X, right_motor_can_id=0x%03X",
      can_interface_.c_str(),
      motor_can_ids_[LEFT_MOTOR_INDEX],
      motor_can_ids_[RIGHT_MOTOR_INDEX]);
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Parameter missing: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Error parsing parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!initializeCanSockets()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  closeCanSockets();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool SteadydriveHardwareInterface::initializeCanSockets()
{
  closeCanSockets();

  for (size_t motor_index = 0; motor_index < can_sockets_.size(); ++motor_index) {
    if (!initializeMotorSocket(motor_index)) {
      closeCanSockets();
      return false;
    }
  }

  return true;
}

bool SteadydriveHardwareInterface::initializeMotorSocket(size_t motor_index)
{
  const int socket_fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN socket creation failed on '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN ioctl(SIOCGIFINDEX) failed on '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  can_filter filter {};
  filter.can_id = motor_can_ids_[motor_index];
  filter.can_mask = CAN_SFF_MASK;
  if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN filter setup failed for 0x%03X on '%s': %s",
      motor_can_ids_[motor_index], can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  const int recv_own_msgs = 0;
  if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN own-message suppression failed on '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN bind failed on '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  can_sockets_[motor_index] = socket_fd;
  return true;
}

void SteadydriveHardwareInterface::closeCanSockets()
{
  for (auto & socket_fd : can_sockets_) {
    if (socket_fd >= 0) {
      ::close(socket_fd);
      socket_fd = -1;
    }
  }
}

bool SteadydriveHardwareInterface::sendMotorCommand(
  size_t motor_index, uint8_t command_byte,
  uint8_t byte1, uint8_t byte2, uint8_t byte3,
  uint8_t byte4, uint8_t byte5, uint8_t byte6,
  uint8_t byte7)
{
  if (motor_index >= can_sockets_.size() || can_sockets_[motor_index] < 0) {
    return false;
  }

  struct can_frame frame {};
  frame.can_id = motor_can_ids_[motor_index];
  frame.can_dlc = 8;
  frame.data[0] = command_byte;
  frame.data[1] = byte1;
  frame.data[2] = byte2;
  frame.data[3] = byte3;
  frame.data[4] = byte4;
  frame.data[5] = byte5;
  frame.data[6] = byte6;
  frame.data[7] = byte7;

  return ::write(can_sockets_[motor_index], &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}


/**
 * @brief Write commanded velocities to the steadydrive
 */
void SteadydriveHardwareInterface::writeCommandsToHardware()
{
  for (size_t motor_index = 0; motor_index < velocity_commands_.size(); ++motor_index) {
    const double motor_velocity_deg_s =
      velocity_commands_[motor_index] * gear_ratios_[motor_index] * RAD_TO_DEG *
      positive_motor_direction_signs_[motor_index];
    int32_t speed_control_value =
      static_cast<int32_t>(std::lround(motor_velocity_deg_s * 100.0));
    speed_control_value = std::clamp(speed_control_value, MIN_SPEED_COMMAND, MAX_SPEED_COMMAND);

    const uint8_t byte4 = static_cast<uint8_t>(speed_control_value & 0xFF);
    const uint8_t byte5 = static_cast<uint8_t>((speed_control_value >> 8) & 0xFF);
    const uint8_t byte6 = static_cast<uint8_t>((speed_control_value >> 16) & 0xFF);
    const uint8_t byte7 = static_cast<uint8_t>((speed_control_value >> 24) & 0xFF);

    (void)sendMotorCommand(motor_index, 0xA2, 0x00, 0x00, 0x00, byte4, byte5, byte6, byte7);
  }
}

void SteadydriveHardwareInterface::readAvailableMotorFrames(size_t motor_index)
{
  if (motor_index >= can_sockets_.size() || can_sockets_[motor_index] < 0) {
    return;
  }

  while (true) {
    struct can_frame response {};
    const ssize_t bytes_read = ::read(can_sockets_[motor_index], &response, sizeof(response));
    if (bytes_read < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(
          rclcpp::get_logger(hw_name_), "SocketCAN read failed on '%s' for motor 0x%03X: %s",
          can_interface_.c_str(), motor_can_ids_[motor_index], std::strerror(errno));
      }
      return;
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(response))) {
      continue;
    }

    processMotorFrame(motor_index, response);
  }
}

void SteadydriveHardwareInterface::processMotorFrame(size_t motor_index, const struct can_frame & frame)
{
  if (frame.can_id != motor_can_ids_[motor_index] || frame.can_dlc < 8 || frame.data[0] != 0x9C) {
    return;
  }

  const int16_t speed_deg_per_sec = decode_signed_16bit(frame.data[4], frame.data[5]);
  const uint16_t encoder_position_raw =
    (static_cast<uint16_t>(frame.data[6]) |
    (static_cast<uint16_t>(frame.data[7]) << 8)) & 0x3fff;

  velocity_states_[motor_index] =
    static_cast<double>(speed_deg_per_sec) * DEG_TO_RAD *
    positive_motor_direction_signs_[motor_index] / gear_ratios_[motor_index];
  position_states_[motor_index] =
    unwrapEncoderPositionRad(motor_index, encoder_position_raw) *
    positive_motor_direction_signs_[motor_index] / gear_ratios_[motor_index];
}

double SteadydriveHardwareInterface::unwrapEncoderPositionRad(
  size_t motor_index, uint16_t encoder_position_raw)
{
  if (!last_encoder_position_raw_[motor_index]) {
    last_encoder_position_raw_[motor_index] = encoder_position_raw;
    accumulated_motor_position_rad_[motor_index] =
      static_cast<double>(encoder_position_raw) * RAD_PER_COUNT;
    return accumulated_motor_position_rad_[motor_index];
  }

  int delta_counts =
    static_cast<int>(encoder_position_raw) - static_cast<int>(*last_encoder_position_raw_[motor_index]);
  const int half_turn_counts = static_cast<int>(ENCODER_COUNTS_PER_REV / 2.0);

  if (delta_counts > half_turn_counts) {
    delta_counts -= static_cast<int>(ENCODER_COUNTS_PER_REV);
  } else if (delta_counts < -half_turn_counts) {
    delta_counts += static_cast<int>(ENCODER_COUNTS_PER_REV);
  }

  accumulated_motor_position_rad_[motor_index] += static_cast<double>(delta_counts) * RAD_PER_COUNT;
  last_encoder_position_raw_[motor_index] = encoder_position_raw;
  return accumulated_motor_position_rad_[motor_index];
}

/**
 * @brief Pull latest speed and travel measurements from MCU, 
 * and store in joint structure for ros_control
 * 
 */
void SteadydriveHardwareInterface::updateJointsFromHardware()
{
  for (size_t motor_index = 0; motor_index < can_sockets_.size(); ++motor_index) {
    readAvailableMotorFrames(motor_index);
    (void)sendMotorCommand(motor_index, 0x9C);
  }
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), 
    "Reading joint states (L: %f, R: %f)",
    position_states_[LEFT_MOTOR_INDEX], position_states_[RIGHT_MOTOR_INDEX]);
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_),
    "Reading joint velocities (L: %f, R: %f)",
    velocity_states_[LEFT_MOTOR_INDEX], velocity_states_[RIGHT_MOTOR_INDEX]);     
}


hardware_interface::CallbackReturn SteadydriveHardwareInterface::validateJoints()
{ 
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffDriveHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> SteadydriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < num_joints_; i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SteadydriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < num_joints_; i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Starting ...please wait...");
  // set some default values
  for (auto i = 0u; i < num_joints_; i++) {
    velocity_commands_[i] = 0.0;
    prev_velocity_commands_[i] = 0.0;
    position_states_[i] = 0.0;
    velocity_states_[i] = 0.0;
    last_encoder_position_raw_[i].reset();
    accumulated_motor_position_rad_[i] = 0.0;
    (void)sendMotorCommand(i, 0x88);
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");
  for (auto i = 0u; i < num_joints_; i++) {
    (void)sendMotorCommand(i, 0xA2);
    (void)sendMotorCommand(i, 0x81);
  }
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System successfully stopped!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SteadydriveHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Reading from hardware");
  updateJointsFromHardware();
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully read!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SteadydriveHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Writing to hardware");
  writeCommandsToHardware();
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully written!");
  return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(amr_sweeper_steadydrive::SteadydriveHardwareInterface, hardware_interface::SystemInterface)
