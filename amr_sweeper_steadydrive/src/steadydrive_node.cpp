#include "steadydrive_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

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
constexpr double CURRENT_RAW_TO_AMPERE = 0.01;
constexpr auto kReadinessPollPeriod = std::chrono::milliseconds(20);

bool all_zero_payload_except_command(const struct can_frame & frame)
{
  for (std::size_t index = 1; index < 8; ++index) {
    if (frame.data[index] != 0x00) {
      return false;
    }
  }
  return true;
}

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

int load_optional_int(const YAML::Node & root, const std::string & key, int fallback)
{
  if (!root[key]) {
    return fallback;
  }
  return root[key].as<int>();
}

constexpr std::array<amr_sweeper_steadydrive::ProtectionType, 5> kProtectionTypes = {
  amr_sweeper_steadydrive::ProtectionType::OverTorque,
  amr_sweeper_steadydrive::ProtectionType::OverSpeed,
  amr_sweeper_steadydrive::ProtectionType::OverTemperature,
  amr_sweeper_steadydrive::ProtectionType::OverCurrent,
  amr_sweeper_steadydrive::ProtectionType::OverVoltage,
};

std::size_t protectionIndex(amr_sweeper_steadydrive::ProtectionType type)
{
  return static_cast<std::size_t>(type);
}

const char * protectionKey(amr_sweeper_steadydrive::ProtectionType type)
{
  switch (type) {
    case amr_sweeper_steadydrive::ProtectionType::OverTorque:
      return "over_torque";
    case amr_sweeper_steadydrive::ProtectionType::OverSpeed:
      return "over_speed";
    case amr_sweeper_steadydrive::ProtectionType::OverTemperature:
      return "over_temperature";
    case amr_sweeper_steadydrive::ProtectionType::OverCurrent:
      return "over_current";
    case amr_sweeper_steadydrive::ProtectionType::OverVoltage:
      return "over_voltage";
    case amr_sweeper_steadydrive::ProtectionType::Count:
      break;
  }
  return "unknown";
}

double faultTypeStateValue(const std::optional<amr_sweeper_steadydrive::ProtectionFault> & fault)
{
  if (!fault) {
    return -1.0;
  }
  return static_cast<double>(static_cast<int>(fault->type));
}

std::optional<double> selectMeasuredValue(
  amr_sweeper_steadydrive::ProtectionType type,
  const amr_sweeper_steadydrive::JointTelemetry & telemetry)
{
  switch (type) {
    case amr_sweeper_steadydrive::ProtectionType::OverTorque:
      if (!telemetry.has_torque_proxy) {
        return std::nullopt;
      }
      return std::fabs(telemetry.torque_proxy);
    case amr_sweeper_steadydrive::ProtectionType::OverSpeed:
      if (!telemetry.has_speed) {
        return std::nullopt;
      }
      return std::fabs(telemetry.speed_rad_s);
    case amr_sweeper_steadydrive::ProtectionType::OverTemperature:
      if (!telemetry.has_temperature) {
        return std::nullopt;
      }
      return telemetry.temperature_c;
    case amr_sweeper_steadydrive::ProtectionType::OverCurrent:
      if (!telemetry.has_current) {
        return std::nullopt;
      }
      return std::fabs(telemetry.current_a);
    case amr_sweeper_steadydrive::ProtectionType::OverVoltage:
      if (!telemetry.has_voltage) {
        return std::nullopt;
      }
      return telemetry.voltage_v;
    case amr_sweeper_steadydrive::ProtectionType::Count:
      break;
  }
  return std::nullopt;
}

amr_sweeper_steadydrive::ProtectionLimit load_protection_limit(
  const YAML::Node & defaults_node,
  const YAML::Node & override_node,
  amr_sweeper_steadydrive::ProtectionType type)
{
  amr_sweeper_steadydrive::ProtectionLimit limit;
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
  motor_can_ids_.resize(num_joints_);
  can_sockets_.assign(num_joints_, -1);
  last_encoder_position_raw_.resize(num_joints_);
  accumulated_motor_position_rad_.resize(num_joints_, 0.0);
  joint_telemetry_.resize(num_joints_);
  protection_states_.resize(num_joints_);
  motor_state_1_received_.assign(num_joints_, false);
  motor_state_2_received_.assign(num_joints_, false);

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
      load_required_string_with_alias(
        hardware_config, "left_motor_positive_direction", "left_positive_motor_direction",
        "amr_sweeper_steadydrive.yaml"),
      load_required_string_with_alias(
        hardware_config, "right_motor_positive_direction", "right_positive_motor_direction",
        "amr_sweeper_steadydrive.yaml"),
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
    reconnect_attempt_interval_ms_ =
      load_optional_int(hardware_config, "reconnect_attempt_interval_ms", reconnect_attempt_interval_ms_);
    retry_attempts_before_error_ =
      load_optional_int(hardware_config, "retry_attempts_before_error", retry_attempts_before_error_);
    fatal_after_consecutive_errors_ = load_optional_int(
      hardware_config, "fatal_after_consecutive_errors", fatal_after_consecutive_errors_);
    max_reconnect_attempts_ =
      load_optional_int(hardware_config, "max_reconnect_attempts", max_reconnect_attempts_);
    motor_ready_timeout_ = std::chrono::milliseconds(
      load_optional_int(
        hardware_config, "motor_ready_timeout_ms",
        static_cast<int>(motor_ready_timeout_.count())));
    loadProtectionParameters();

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
  if (motor_ready_timeout_ < std::chrono::milliseconds(1)) {
    motor_ready_timeout_ = std::chrono::milliseconds(1);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!safety_stop_publisher_) {
    if (auto node = get_node()) {
      safety_stop_publisher_ = node->create_publisher<amr_sweeper_safety_msgs::msg::SafetyStop>(
        safety_stop_topic_name_, rclcpp::QoS(10).reliable().transient_local());
    }
  }
  if (!clear_safety_stop_service_) {
    if (auto node = get_node()) {
      clear_safety_stop_service_ = node->create_service<std_srvs::srv::Trigger>(
        clear_safety_stop_service_name_,
        std::bind(
          &SteadydriveHardwareInterface::clearSafetyStopService, this,
          std::placeholders::_1, std::placeholders::_2));
    }
  }

  if (!initializeCanSockets()) {
    reportConnectionIssue(
      last_connection_error_message_.empty() ?
      "Failed to initialize SocketCAN on " + can_interface_ :
      last_connection_error_message_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string failure_reason;
  if (!confirmMotorTelemetryReady(motor_ready_timeout_, failure_reason)) {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "%s", failure_reason.c_str());
    closeCanSockets();
    last_connection_error_message_ = failure_reason;
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  lifecycle_active_ = false;
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

  last_connection_error_message_.clear();
  last_reconnect_attempt_time_ = std::chrono::steady_clock::now();
  resetIssueCounters();
  return true;
}

bool SteadydriveHardwareInterface::initializeMotorSocket(size_t motor_index)
{
  const int socket_fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd < 0) {
    last_connection_error_message_ =
      "SocketCAN socket creation failed on '" + can_interface_ + "': " + std::strerror(errno);
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN socket creation failed on '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    last_connection_error_message_ =
      "SocketCAN ioctl(SIOCGIFINDEX) failed on '" + can_interface_ + "': " + std::strerror(errno);
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
    last_connection_error_message_ =
      "SocketCAN filter setup failed for motor 0x" + [&]() {
        std::ostringstream stream;
        stream << std::hex << std::uppercase << motor_can_ids_[motor_index];
        return stream.str();
      }() + " on '" + can_interface_ + "': " + std::strerror(errno);
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN filter setup failed for 0x%03X on '%s': %s",
      motor_can_ids_[motor_index], can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  const int can_loopback = 0;
  if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &can_loopback, sizeof(can_loopback)) < 0) {
    last_connection_error_message_ =
      "SocketCAN loopback disable failed on '" + can_interface_ + "': " + std::strerror(errno);
    RCLCPP_ERROR(
      rclcpp::get_logger(hw_name_), "SocketCAN loopback disable failed on '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  const int recv_own_msgs = 0;
  if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
    last_connection_error_message_ =
      "SocketCAN own-message suppression failed on '" + can_interface_ + "': " + std::strerror(errno);
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
    last_connection_error_message_ =
      "SocketCAN bind failed on '" + can_interface_ + "': " + std::strerror(errno);
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

bool SteadydriveHardwareInterface::ensureCanSockets()
{
  if (fatal_error_) {
    return false;
  }

  const bool all_connected = std::all_of(
    can_sockets_.begin(), can_sockets_.end(), [](int socket_fd) { return socket_fd >= 0; });
  if (all_connected) {
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

  if (!initializeCanSockets()) {
    reportConnectionIssue(
      last_connection_error_message_.empty() ?
      "Failed to initialize SocketCAN on " + can_interface_ :
      last_connection_error_message_);
    return false;
  }

  if (lifecycle_active_) {
    for (size_t i = 0; i < num_joints_; ++i) {
      (void)sendMotorCommand(i, 0x88);
    }
    std::string failure_reason;
    if (!confirmMotorsActive(motor_ready_timeout_, failure_reason)) {
      last_connection_error_message_ = failure_reason;
      reportConnectionIssue(last_connection_error_message_);
      closeCanSockets();
      return false;
    }
  }
  return true;
}

void SteadydriveHardwareInterface::reportConnectionIssue(const std::string & message)
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

void SteadydriveHardwareInterface::logEscalatingIssue(int count, const std::string & message)
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

void SteadydriveHardwareInterface::resetIssueCounters()
{
  reconnect_attempt_count_ = 0;
  connection_issue_count_ = 0;
  fatal_error_ = false;
}

void SteadydriveHardwareInterface::loadProtectionParameters()
{
  const YAML::Node hardware_config =
    load_hardware_config("amr_sweeper_steadydrive", "amr_sweeper_steadydrive.yaml");
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
    protection_node["safety_stop_sender_name"].as<std::string>() : "steadydrive_hardware_interface";
  clear_safety_stop_service_name_ =
    protection_node["clear_safety_stop_service_name"] ?
    protection_node["clear_safety_stop_service_name"].as<std::string>() :
    "/steadydrive_ros2_control/clear_safety_stop";

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

void SteadydriveHardwareInterface::clearProtectionFaults()
{
  for (std::size_t i = 0; i < protection_states_.size(); ++i) {
    protection_states_[i].fault.reset();
    protection_states_[i].safe_stop_sent = false;
    protection_states_[i].safety_stop_published = false;
    protection_states_[i].over_threshold_seconds.fill(0.0);
    updateProtectionStatusState(i);
  }
}

void SteadydriveHardwareInterface::clearSafetyStopService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  if (!ensureCanSockets()) {
    response->success = false;
    response->message = last_connection_error_message_.empty() ?
      "Steadydrive CAN interface is not ready" : last_connection_error_message_;
    return;
  }

  clearProtectionFaults();

  for (std::size_t i = 0; i < num_joints_; ++i) {
    velocity_commands_[i] = 0.0;
    prev_velocity_commands_[i] = 0.0;
    if (!sendMotorCommand(i, 0x88)) {
      response->success = false;
      response->message = last_connection_error_message_.empty() ?
        "Failed to re-enable Steadydrive motor" : last_connection_error_message_;
      return;
    }
    if (!sendMotorCommand(i, 0xA2)) {
      response->success = false;
      response->message = last_connection_error_message_.empty() ?
        "Failed to send zero Steadydrive command" : last_connection_error_message_;
      return;
    }
  }

  response->success = true;
  response->message = "Steadydrive safety stop cleared and motors re-enabled";
}

void SteadydriveHardwareInterface::resetReadinessTracking()
{
  std::fill(motor_state_1_received_.begin(), motor_state_1_received_.end(), false);
  std::fill(motor_state_2_received_.begin(), motor_state_2_received_.end(), false);
  for (auto & telemetry : joint_telemetry_) {
    telemetry.has_error_state = false;
    telemetry.has_voltage = false;
    telemetry.has_temperature = false;
    telemetry.has_current = false;
    telemetry.has_torque_proxy = false;
    telemetry.has_speed = false;
  }
}

bool SteadydriveHardwareInterface::confirmMotorTelemetryReady(
  std::chrono::milliseconds timeout,
  std::string & failure_reason)
{
  resetReadinessTracking();
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    for (std::size_t i = 0; i < num_joints_; ++i) {
      if (!motor_state_1_received_[i]) {
        (void)sendMotorCommand(i, 0x9A);
      }
      if (!motor_state_2_received_[i]) {
        (void)sendMotorCommand(i, 0x9C);
      }
    }

    for (std::size_t i = 0; i < num_joints_; ++i) {
      readAvailableMotorFrames(i);
    }

    const bool all_ready = std::all_of(
      motor_state_1_received_.begin(), motor_state_1_received_.end(),
      [](bool received) { return received; }) &&
      std::all_of(
      motor_state_2_received_.begin(), motor_state_2_received_.end(),
      [](bool received) { return received; });
    if (all_ready) {
      failure_reason.clear();
      return true;
    }

    std::this_thread::sleep_for(kReadinessPollPeriod);
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    if (!motor_state_1_received_[i]) {
      failure_reason =
        "Steadydrive joint '" + info_.joints[i].name + "' did not return state-1 telemetry during configure";
      return false;
    }
    if (!motor_state_2_received_[i]) {
      failure_reason =
        "Steadydrive joint '" + info_.joints[i].name + "' did not return state-2 telemetry during configure";
      return false;
    }
  }

  failure_reason = "Timed out while confirming Steadydrive telemetry readiness";
  return false;
}

bool SteadydriveHardwareInterface::confirmMotorsActive(
  std::chrono::milliseconds timeout,
  std::string & failure_reason)
{
  resetReadinessTracking();
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_progress_log = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < deadline) {
    for (std::size_t i = 0; i < num_joints_; ++i) {
      if (!motor_state_1_received_[i]) {
        (void)sendMotorCommand(i, 0x9A);
      }
      if (!motor_state_2_received_[i]) {
        (void)sendMotorCommand(i, 0x9C);
      }
    }

    for (std::size_t i = 0; i < num_joints_; ++i) {
      readAvailableMotorFrames(i);
    }

    bool all_ready = true;
    for (std::size_t i = 0; i < num_joints_; ++i) {
      if (!motor_state_1_received_[i] || !motor_state_2_received_[i]) {
        all_ready = false;
        continue;
      }
      if (!joint_telemetry_[i].has_error_state) {
        all_ready = false;
        continue;
      }
      if (joint_telemetry_[i].error_state != 0) {
        failure_reason =
          "Steadydrive joint '" + info_.joints[i].name + "' reported error_state=0x" +
          [&]() {
            std::ostringstream stream;
            stream << std::hex << std::uppercase << static_cast<int>(joint_telemetry_[i].error_state);
            return stream.str();
          }();
        return false;
      }
    }

    if (all_ready) {
      failure_reason.clear();
      return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_progress_log) {
      std::ostringstream status;
      for (std::size_t i = 0; i < num_joints_; ++i) {
        if (i > 0) {
          status << " | ";
        }
        status
          << info_.joints[i].name
          << "(can_id=0x" << std::hex << std::uppercase << motor_can_ids_[i] << std::dec
          << ", state1=" << (motor_state_1_received_[i] ? "yes" : "no")
          << ", state2=" << (motor_state_2_received_[i] ? "yes" : "no")
          << ", err=";
        if (joint_telemetry_[i].has_error_state) {
          status << "0x" << std::hex << std::uppercase
                 << static_cast<int>(joint_telemetry_[i].error_state) << std::dec;
        } else {
          status << "unknown";
        }
        status << ")";
      }
      const auto remaining_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
      RCLCPP_INFO(
        rclcpp::get_logger(hw_name_),
        "Waiting for Steadydrive activation readiness; remaining=%lld ms; %s",
        static_cast<long long>(remaining_ms), status.str().c_str());
      next_progress_log = now + std::chrono::seconds(1);
    }

    std::this_thread::sleep_for(kReadinessPollPeriod);
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    if (!motor_state_1_received_[i]) {
      failure_reason =
        "Steadydrive joint '" + info_.joints[i].name + "' did not return state-1 telemetry during activation";
      return false;
    }
    if (!motor_state_2_received_[i]) {
      failure_reason =
        "Steadydrive joint '" + info_.joints[i].name + "' did not return state-2 telemetry during activation";
      return false;
    }
  }

  failure_reason = "Timed out while confirming Steadydrive activation readiness";
  return false;
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

  if (::write(can_sockets_[motor_index], &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    last_connection_error_message_ =
      "SocketCAN write failed on '" + can_interface_ + "' for motor 0x" + [&]() {
        std::ostringstream stream;
        stream << std::hex << std::uppercase << motor_can_ids_[motor_index];
        return stream.str();
      }() + ": " + std::strerror(errno);
    if (errno == ENETDOWN || errno == ENODEV || errno == EBADF) {
      closeCanSockets();
    }
    return false;
  }
  return true;
}


/**
 * @brief Write commanded velocities to the steadydrive
 */
void SteadydriveHardwareInterface::writeCommandsToHardware()
{
  for (size_t motor_index = 0; motor_index < velocity_commands_.size(); ++motor_index) {
    if (motorHasLatchedFault(motor_index)) {
      stopOrDisableMotor(motor_index);
      prev_velocity_commands_[motor_index] = 0.0;
      continue;
    }

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

    if (!sendMotorCommand(motor_index, 0xA2, 0x00, 0x00, 0x00, byte4, byte5, byte6, byte7)) {
      reportConnectionIssue(
        last_connection_error_message_.empty() ?
        "Failed to send Steadydrive command on " + can_interface_ :
        last_connection_error_message_);
      return;
    }
  }
  resetIssueCounters();
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
        last_connection_error_message_ =
          "SocketCAN read failed on '" + can_interface_ + "' for motor 0x" + [&]() {
            std::ostringstream stream;
            stream << std::hex << std::uppercase << motor_can_ids_[motor_index];
            return stream.str();
          }() + ": " + std::strerror(errno);
        RCLCPP_WARN(
          rclcpp::get_logger(hw_name_), "SocketCAN read failed on '%s' for motor 0x%03X: %s",
          can_interface_.c_str(), motor_can_ids_[motor_index], std::strerror(errno));
        closeCanSockets();
        reportConnectionIssue(last_connection_error_message_);
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
  if (frame.can_id != motor_can_ids_[motor_index] || frame.can_dlc < 8) {
    return;
  }

  if (frame.data[0] == 0x9A) {
    // The protocol defines 0x9A as a read command whose request payload is all zeros and whose
    // reply payload carries temperature, voltage, and error-state values. Ignore echoed requests.
    const uint16_t voltage_raw =
      static_cast<uint16_t>(frame.data[3]) | (static_cast<uint16_t>(frame.data[4]) << 8);
    if (voltage_raw == 0U && all_zero_payload_except_command(frame)) {
      return;
    }
    joint_telemetry_[motor_index].temperature_c = static_cast<double>(frame.data[1]);
    joint_telemetry_[motor_index].has_temperature = true;
    joint_telemetry_[motor_index].voltage_v = static_cast<double>(voltage_raw) * 0.1;
    joint_telemetry_[motor_index].has_voltage = true;
    joint_telemetry_[motor_index].error_state = frame.data[7];
    joint_telemetry_[motor_index].has_error_state = true;
    motor_state_1_received_[motor_index] = true;
    return;
  }

  if (frame.data[0] != 0x9C) {
    return;
  }

  // The protocol defines 0x9C as a read command whose request payload is all zeros and whose
  // reply carries temperature, iq current, speed, and encoder position. Ignore echoed requests.
  if (all_zero_payload_except_command(frame)) {
    return;
  }

  const double current_a =
    static_cast<double>(decode_signed_16bit(frame.data[2], frame.data[3])) * CURRENT_RAW_TO_AMPERE;
  const int16_t speed_deg_per_sec = decode_signed_16bit(frame.data[4], frame.data[5]);
  const uint16_t encoder_position_raw =
    (static_cast<uint16_t>(frame.data[6]) |
    (static_cast<uint16_t>(frame.data[7]) << 8)) & 0x3fff;

  joint_telemetry_[motor_index].temperature_c = static_cast<double>(frame.data[1]);
  joint_telemetry_[motor_index].has_temperature = true;
  joint_telemetry_[motor_index].current_a = current_a;
  joint_telemetry_[motor_index].has_current = true;
  joint_telemetry_[motor_index].torque_proxy = current_a;
  joint_telemetry_[motor_index].has_torque_proxy = true;
  velocity_states_[motor_index] =
    static_cast<double>(speed_deg_per_sec) * DEG_TO_RAD *
    positive_motor_direction_signs_[motor_index] / gear_ratios_[motor_index];
  joint_telemetry_[motor_index].speed_rad_s = velocity_states_[motor_index];
  joint_telemetry_[motor_index].has_speed = true;
  position_states_[motor_index] =
    unwrapEncoderPositionRad(motor_index, encoder_position_raw) *
    positive_motor_direction_signs_[motor_index] / gear_ratios_[motor_index];
  motor_state_2_received_[motor_index] = true;
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
    if (!sendMotorCommand(motor_index, 0x9A) || !sendMotorCommand(motor_index, 0x9C)) {
      reportConnectionIssue(
        last_connection_error_message_.empty() ?
        "Failed to query Steadydrive state on " + can_interface_ :
        last_connection_error_message_);
      return;
    }
  }
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), 
    "Reading joint states (L: %f, R: %f)",
    position_states_[LEFT_MOTOR_INDEX], position_states_[RIGHT_MOTOR_INDEX]);
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_),
    "Reading joint velocities (L: %f, R: %f)",
    velocity_states_[LEFT_MOTOR_INDEX], velocity_states_[RIGHT_MOTOR_INDEX]);     
}

void SteadydriveHardwareInterface::updateProtectionStatusState(size_t joint_index)
{
  const auto & telemetry = joint_telemetry_[joint_index];
  const auto & state = protection_states_[joint_index];

  torque_states_[joint_index] = telemetry.has_torque_proxy ? telemetry.torque_proxy : 0.0;
  current_states_[joint_index] = telemetry.has_current ? telemetry.current_a : 0.0;
  temperature_states_[joint_index] = telemetry.has_temperature ? telemetry.temperature_c : 0.0;
  voltage_states_[joint_index] = telemetry.has_voltage ? telemetry.voltage_v : 0.0;
  effort_states_[joint_index] = telemetry.has_torque_proxy ? telemetry.torque_proxy :
    (telemetry.has_current ? telemetry.current_a : 0.0);
  fault_latched_states_[joint_index] = state.fault ? 1.0 : 0.0;
  fault_type_states_[joint_index] = faultTypeStateValue(state.fault);
  fault_measured_states_[joint_index] = state.fault ? state.fault->measured_value : 0.0;
  fault_threshold_states_[joint_index] = state.fault ? state.fault->threshold : 0.0;
}

void SteadydriveHardwareInterface::latchProtectionFault(
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
    "Protection tripped: interface=steadydrive joint=%s protection=%s measured=%.3f %s threshold=%.3f %s action=\"motor stopped and fault latched\"",
    state.joint_name.c_str(),
    protectionKey(type),
    measured_value,
    limit.units.c_str(),
    limit.threshold,
    limit.units.c_str());

  if (safety_stop_publisher_ && !state.safety_stop_published) {
    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = get_clock()->now();
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

bool SteadydriveHardwareInterface::motorHasLatchedFault(size_t joint_index) const
{
  return protection_enabled_ && joint_index < protection_states_.size() && protection_states_[joint_index].fault.has_value();
}

void SteadydriveHardwareInterface::stopOrDisableMotor(size_t joint_index)
{
  if (joint_index >= protection_states_.size()) {
    return;
  }

  auto & state = protection_states_[joint_index];
  velocity_commands_[joint_index] = 0.0;
  if (state.safe_stop_sent) {
    return;
  }

  (void)sendMotorCommand(joint_index, 0xA2);
  (void)sendMotorCommand(joint_index, 0x81);
  state.safe_stop_sent = true;
}

void SteadydriveHardwareInterface::evaluateProtections(const rclcpp::Duration & period)
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
    (rclcpp::Clock(RCL_ROS_TIME).now() - activation_time_) < rclcpp::Duration(startup_ignore_duration_);

  for (std::size_t i = 0; i < num_joints_; ++i) {
    auto & state = protection_states_[i];

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

      const auto measured = selectMeasuredValue(type, joint_telemetry_[i]);
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

    if (joint.state_interfaces.size() < 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu state interface. At least 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
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
  RCLCPP_INFO(
    rclcpp::get_logger(hw_name_),
    "Steadydrive activation started on %s with timeout=%lld ms",
    can_interface_.c_str(), static_cast<long long>(motor_ready_timeout_.count()));
  lifecycle_active_ = true;
  activation_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
  if (clear_faults_on_activate_) {
    clearProtectionFaults();
  }
  // set some default values
  for (auto i = 0u; i < num_joints_; i++) {
    velocity_commands_[i] = 0.0;
    prev_velocity_commands_[i] = 0.0;
    position_states_[i] = 0.0;
    velocity_states_[i] = 0.0;
    last_encoder_position_raw_[i].reset();
    accumulated_motor_position_rad_[i] = 0.0;
    if (!ensureCanSockets()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(hw_name_),
      "Preparing Steadydrive joint '%s' on CAN ID 0x%03X for activation",
      info_.joints[i].name.c_str(), motor_can_ids_[i]);
    (void)sendMotorCommand(i, 0x88);
  }

  std::string failure_reason;
  if (!confirmMotorsActive(motor_ready_timeout_, failure_reason)) {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "%s", failure_reason.c_str());
    last_connection_error_message_ = failure_reason;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");
  lifecycle_active_ = false;
  if (std::all_of(can_sockets_.begin(), can_sockets_.end(), [](int socket_fd) { return socket_fd >= 0; })) {
    for (auto i = 0u; i < num_joints_; i++) {
      (void)sendMotorCommand(i, 0xA2);
      (void)sendMotorCommand(i, 0x81);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System successfully stopped!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SteadydriveHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!ensureCanSockets()) {
    return fatal_error_ ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
  }
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Reading from hardware");
  updateJointsFromHardware();
  evaluateProtections(period);
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully read!");
  return fatal_error_ ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

hardware_interface::return_type SteadydriveHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!ensureCanSockets()) {
    return fatal_error_ ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
  }
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Writing to hardware");
  writeCommandsToHardware();
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully written!");
  return fatal_error_ ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(amr_sweeper_steadydrive::SteadydriveHardwareInterface, hardware_interface::SystemInterface)
