#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "battery_node.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace
{

std::size_t protection_index(BatteryNode::ProtectionType type)
{
  return static_cast<std::size_t>(type);
}

builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time stamp;
  const auto nanoseconds = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

const char * protection_key(BatteryNode::ProtectionType type)
{
  switch (type) {
    case BatteryNode::ProtectionType::OverVoltage:
      return "over_voltage";
    case BatteryNode::ProtectionType::UnderVoltage:
      return "under_voltage";
    case BatteryNode::ProtectionType::ChargingOverCurrent:
      return "charging.over_current";
    case BatteryNode::ProtectionType::ChargingUnderCurrent:
      return "charging.under_current";
    case BatteryNode::ProtectionType::ChargingOverTemperature:
      return "charging.over_temperature";
    case BatteryNode::ProtectionType::ChargingUnderTemperature:
      return "charging.under_temperature";
    case BatteryNode::ProtectionType::DischargingOverCurrent:
      return "discharging.over_current";
    case BatteryNode::ProtectionType::DischargingUnderCurrent:
      return "discharging.under_current";
    case BatteryNode::ProtectionType::DischargingOverTemperature:
      return "discharging.over_temperature";
    case BatteryNode::ProtectionType::DischargingUnderTemperature:
      return "discharging.under_temperature";
    case BatteryNode::ProtectionType::Count:
      break;
  }
  return "unknown";
}

BatteryNode::ProtectionLimit load_protection_limit(
  const BatteryNode & node, BatteryNode::ProtectionType type)
{
  BatteryNode::ProtectionLimit limit;
  const std::string key = std::string("protection.") + protection_key(type);
  limit.enabled = node.get_parameter(key + ".enabled").as_bool();
  limit.threshold = node.get_parameter(key + ".threshold").as_double();
  limit.units = node.get_parameter(key + ".units").as_string();
  return limit;
}

}  // namespace

BatteryNode::BatteryNode()
: Node("amr_sweeper_battery_node")
{
  declare_parameter<std::string>("can_interface", "can0");
  declare_parameter<double>("timer_period", 15.0);
  declare_parameter<int64_t>("priority", 0x18);
  declare_parameter<int64_t>("bms_address", 0x01);
  declare_parameter<int64_t>("pc_address", 0x40);
  declare_parameter<int>("retry_attempts_before_error", 3);
  declare_parameter<int>("fatal_after_consecutive_errors", 10);
  declare_parameter<int>("max_reconnect_attempts", 10);
  declare_parameter<std::string>("protection.safety_stop_topic_name", "safety_msgs/stop");
  declare_parameter<std::string>("protection.safety_stop_sender_name", "battery_node");
  declare_parameter<bool>("protection.over_voltage.enabled", false);
  declare_parameter<double>("protection.over_voltage.threshold", 60.0);
  declare_parameter<std::string>("protection.over_voltage.units", "V");
  declare_parameter<bool>("protection.under_voltage.enabled", false);
  declare_parameter<double>("protection.under_voltage.threshold", 42.0);
  declare_parameter<std::string>("protection.under_voltage.units", "V");
  declare_parameter<bool>("protection.charging.over_current.enabled", false);
  declare_parameter<double>("protection.charging.over_current.threshold", 80.0);
  declare_parameter<std::string>("protection.charging.over_current.units", "A");
  declare_parameter<bool>("protection.charging.under_current.enabled", false);
  declare_parameter<double>("protection.charging.under_current.threshold", 5.0);
  declare_parameter<std::string>("protection.charging.under_current.units", "A");
  declare_parameter<bool>("protection.charging.over_temperature.enabled", false);
  declare_parameter<double>("protection.charging.over_temperature.threshold", 45.0);
  declare_parameter<std::string>("protection.charging.over_temperature.units", "degC");
  declare_parameter<bool>("protection.charging.under_temperature.enabled", false);
  declare_parameter<double>("protection.charging.under_temperature.threshold", 0.0);
  declare_parameter<std::string>("protection.charging.under_temperature.units", "degC");
  declare_parameter<bool>("protection.discharging.over_current.enabled", false);
  declare_parameter<double>("protection.discharging.over_current.threshold", -80.0);
  declare_parameter<std::string>("protection.discharging.over_current.units", "A");
  declare_parameter<bool>("protection.discharging.under_current.enabled", false);
  declare_parameter<double>("protection.discharging.under_current.threshold", -5.0);
  declare_parameter<std::string>("protection.discharging.under_current.units", "A");
  declare_parameter<bool>("protection.discharging.over_temperature.enabled", false);
  declare_parameter<double>("protection.discharging.over_temperature.threshold", 60.0);
  declare_parameter<std::string>("protection.discharging.over_temperature.units", "degC");
  declare_parameter<bool>("protection.discharging.under_temperature.enabled", false);
  declare_parameter<double>("protection.discharging.under_temperature.threshold", 0.0);
  declare_parameter<std::string>("protection.discharging.under_temperature.units", "degC");

  can_interface_ = get_parameter("can_interface").as_string();
  const auto timer_period = get_parameter("timer_period").as_double();
  priority_ = static_cast<uint8_t>(get_parameter("priority").as_int());
  bms_addr_ = static_cast<uint8_t>(get_parameter("bms_address").as_int());
  pc_addr_ = static_cast<uint8_t>(get_parameter("pc_address").as_int());
  retry_attempts_before_error_ = get_parameter("retry_attempts_before_error").as_int();
  fatal_after_consecutive_errors_ = get_parameter("fatal_after_consecutive_errors").as_int();
  max_reconnect_attempts_ = get_parameter("max_reconnect_attempts").as_int();
  safety_stop_topic_name_ = get_parameter("protection.safety_stop_topic_name").as_string();
  safety_stop_sender_name_ = get_parameter("protection.safety_stop_sender_name").as_string();

  if (retry_attempts_before_error_ < 1) {
    retry_attempts_before_error_ = 1;
  }
  if (fatal_after_consecutive_errors_ < 1) {
    fatal_after_consecutive_errors_ = 1;
  }
  if (max_reconnect_attempts_ < 0) {
    max_reconnect_attempts_ = 0;
  }

  RCLCPP_INFO(
    get_logger(),
    "Using interface %s, 29-bit IDs, prio=0x%02X, BMS=0x%02X, PC=0x%02X",
    can_interface_.c_str(), priority_, bms_addr_, pc_addr_);

  batt_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
  health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("battery_health", 10);
  safety_stop_pub_ = create_publisher<amr_sweeper_safety_msgs::msg::SafetyStop>(
    safety_stop_topic_name_, 10);

  for (auto type : {
      ProtectionType::OverVoltage,
      ProtectionType::UnderVoltage,
      ProtectionType::ChargingOverCurrent,
      ProtectionType::ChargingUnderCurrent,
      ProtectionType::ChargingOverTemperature,
      ProtectionType::ChargingUnderTemperature,
      ProtectionType::DischargingOverCurrent,
      ProtectionType::DischargingUnderCurrent,
      ProtectionType::DischargingOverTemperature,
      ProtectionType::DischargingUnderTemperature})
  {
    protection_states_[protection_index(type)].limit = load_protection_limit(*this, type);
  }

  if (!setup_can_socket(true)) {
    report_connection_issue(last_connection_error_message_);
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timer_period));
  timer_ = create_wall_timer(period, std::bind(&BatteryNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "amr_sweeper_battery_node started.");
}

BatteryNode::~BatteryNode()
{
  close_can_socket();
}

bool BatteryNode::setup_can_socket(bool log_failure)
{
  std::lock_guard<std::mutex> socket_lock(socket_mutex_);
  if (can_socket_ >= 0) {
    return true;
  }

  const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    (void)log_failure;
    last_connection_error_message_ =
      "Failed to create CAN socket for '" + can_interface_ + "': " + std::strerror(errno);
    return false;
  }

  ifreq ifr {};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    last_connection_error_message_ =
      "Failed to resolve CAN interface '" + can_interface_ + "': " + std::strerror(errno);
    ::close(fd);
    return false;
  }

  sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    last_connection_error_message_ =
      "Failed to bind CAN interface '" + can_interface_ + "': " + std::strerror(errno);
    ::close(fd);
    return false;
  }

  can_socket_ = fd;
  rx_running_.store(true);
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  rx_thread_ = std::thread(&BatteryNode::rx_loop, this);
  missing_can_warned_ = false;
  last_connection_error_message_.clear();
  reset_issue_counters();

  RCLCPP_INFO(get_logger(), "Connected to CAN interface '%s'.", can_interface_.c_str());
  return true;
}

void BatteryNode::close_can_socket()
{
  int socket_to_close = -1;
  {
    std::lock_guard<std::mutex> socket_lock(socket_mutex_);
    rx_running_.store(false);
    socket_to_close = can_socket_;
    can_socket_ = -1;
  }

  if (socket_to_close >= 0) {
    ::close(socket_to_close);
  }

  if (rx_thread_.joinable() && rx_thread_.get_id() != std::this_thread::get_id()) {
    rx_thread_.join();
  }
}

int BatteryNode::current_socket() const
{
  std::lock_guard<std::mutex> socket_lock(socket_mutex_);
  return can_socket_;
}

void BatteryNode::enter_fatal_state(const std::string & message)
{
  fatal_error_.store(true);
  RCLCPP_FATAL(get_logger(), "%s", message.c_str());
  if (timer_) {
    timer_->cancel();
  }
  rclcpp::shutdown();
}

void BatteryNode::report_connection_issue(const std::string & message)
{
  std::lock_guard<std::mutex> issue_lock(issue_mutex_);
  ++connection_issue_count_;
  ++reconnect_attempt_count_;

  if (max_reconnect_attempts_ > 0 && reconnect_attempt_count_ >= max_reconnect_attempts_) {
    enter_fatal_state(
      message + ". Reached reconnect limit after " + std::to_string(reconnect_attempt_count_) +
      " attempts");
    return;
  }

  log_escalating_issue(connection_issue_count_, message);
}

void BatteryNode::log_escalating_issue(int count, const std::string & message)
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
    " consecutive connection failures");
}

void BatteryNode::reset_issue_counters()
{
  std::lock_guard<std::mutex> issue_lock(issue_mutex_);
  reconnect_attempt_count_ = 0;
  connection_issue_count_ = 0;
  fatal_error_.store(false);
}

uint32_t BatteryNode::make_pc_to_bms_id(uint8_t data_id) const
{
  return
    (static_cast<uint32_t>(priority_) << 24) |
    (static_cast<uint32_t>(data_id) << 16) |
    (static_cast<uint32_t>(bms_addr_) << 8) |
    static_cast<uint32_t>(pc_addr_);
}

BatteryNode::ParsedId BatteryNode::parse_bms_to_pc_id(uint32_t arb_id)
{
  ParsedId parsed {};
  parsed.priority = static_cast<uint8_t>((arb_id >> 24) & 0xFF);
  parsed.data_id = static_cast<uint8_t>((arb_id >> 16) & 0xFF);
  parsed.dst_addr = static_cast<uint8_t>((arb_id >> 8) & 0xFF);
  parsed.src_addr = static_cast<uint8_t>(arb_id & 0xFF);
  return parsed;
}

void BatteryNode::on_timer()
{
  if (fatal_error_.load()) {
    return;
  }

  if (current_socket() < 0) {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "daly_bms_health";
    status.hardware_id = "daly_bms_can";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
      "No CAN interface '" + can_interface_ + "' detected; battery data is unavailable.";

    diag_array.status.push_back(status);
    health_pub_->publish(diag_array);

    if (!setup_can_socket(false)) {
      report_connection_issue(
        last_connection_error_message_.empty() ?
        "No CAN interface '" + can_interface_ + "' detected; battery data is unavailable." :
        last_connection_error_message_);
      missing_can_warned_ = true;
    }
    return;
  }

  bool had_tx_failure = false;
  for (const auto data_id : kDataIds) {
    if (!send_request(data_id)) {
      had_tx_failure = true;
      break;
    }
  }

  if (had_tx_failure) {
    report_connection_issue(
      last_connection_error_message_.empty() ?
      "CAN TX failed on interface '" + can_interface_ + "'" :
      last_connection_error_message_);
    return;
  }

  reset_issue_counters();

  publish_battery_state();
  publish_battery_health();
  evaluate_protections();
}

bool BatteryNode::send_request(uint8_t data_id)
{
  const auto fd = current_socket();
  if (fd < 0) {
    return false;
  }

  can_frame frame {};
  frame.can_id = make_pc_to_bms_id(data_id) | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::fill(std::begin(frame.data), std::end(frame.data), 0U);

  const auto written = write(fd, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    last_connection_error_message_ =
      "CAN TX failed on interface '" + can_interface_ + "' for request 0x" +
      [&data_id]() {
        std::ostringstream stream;
        stream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
               << static_cast<int>(data_id);
        return stream.str();
      }() + ": " + std::strerror(errno);
    if (errno == ENETDOWN || errno == ENODEV || errno == EBADF) {
      close_can_socket();
    }
    return false;
  }
  return true;
}

void BatteryNode::rx_loop()
{
  while (rx_running_.load()) {
    const auto fd = current_socket();
    if (fd < 0) {
      break;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    timeval timeout {};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    const auto ready = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
    if (!rx_running_.load()) {
      break;
    }
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      last_connection_error_message_ =
        "SocketCAN select failed on '" + can_interface_ + "': " + std::strerror(errno);
      close_can_socket();
      report_connection_issue(last_connection_error_message_);
      break;
    }
    if (ready == 0) {
      continue;
    }

    can_frame frame {};
    const auto bytes = read(fd, &frame, sizeof(frame));
    if (bytes != static_cast<ssize_t>(sizeof(frame))) {
      if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        last_connection_error_message_ =
          "SocketCAN read failed on '" + can_interface_ + "': " + std::strerror(errno);
        close_can_socket();
        report_connection_issue(last_connection_error_message_);
        break;
      }
      continue;
    }

    handle_can_frame(frame);
  }
}

void BatteryNode::handle_can_frame(const can_frame & frame)
{
  if ((frame.can_id & CAN_EFF_FLAG) == 0U) {
    return;
  }

  const auto parsed = parse_bms_to_pc_id(frame.can_id & CAN_EFF_MASK);
  if (parsed.src_addr != bms_addr_ || parsed.dst_addr != pc_addr_ || parsed.priority != priority_) {
    return;
  }

  const uint8_t * data = frame.data;
  switch (parsed.data_id) {
    case 0x90:
      decode_0x90(data, frame.can_dlc);
      break;
    case 0x91:
      decode_0x91(data, frame.can_dlc);
      break;
    case 0x92:
      decode_0x92(data, frame.can_dlc);
      break;
    case 0x93:
      decode_0x93(data, frame.can_dlc);
      break;
    case 0x94:
      decode_0x94(data, frame.can_dlc);
      break;
    case 0x95:
      decode_0x95(data, frame.can_dlc);
      break;
    case 0x96:
      decode_0x96(data, frame.can_dlc);
      break;
    case 0x97:
      decode_0x97(data, frame.can_dlc);
      break;
    case 0x98:
      decode_0x98(data, frame.can_dlc);
      break;
    default:
      break;
  }
}

void BatteryNode::decode_0x90(const uint8_t * data, size_t len)
{
  if (len < 8) {
    return;
  }

  const auto pack_u16 = static_cast<uint16_t>((data[0] << 8) | data[1]);
  const auto curr_u16 = static_cast<uint16_t>((data[4] << 8) | data[5]);
  const auto soc_u16 = static_cast<uint16_t>((data[6] << 8) | data[7]);

  std::lock_guard<std::mutex> lock(state_mutex_);
  pack_voltage_ = pack_u16 / 10.0;
  pack_current_ = (static_cast<int>(curr_u16) - 30000) / 10.0;
  soc_percent_ = soc_u16 / 10.0;
}

void BatteryNode::decode_0x91(const uint8_t * data, size_t len)
{
  if (len < 6) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  max_cell_voltage_ = static_cast<uint16_t>((data[0] << 8) | data[1]) / 1000.0;
  max_cell_index_ = data[2];
  min_cell_voltage_ = static_cast<uint16_t>((data[3] << 8) | data[4]) / 1000.0;
  min_cell_index_ = data[5];
}

void BatteryNode::decode_0x92(const uint8_t * data, size_t len)
{
  if (len < 4) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  max_temp_ = static_cast<double>(static_cast<int>(data[0]) - 40);
  max_temp_index_ = data[1];
  min_temp_ = static_cast<double>(static_cast<int>(data[2]) - 40);
  min_temp_index_ = data[3];
}

void BatteryNode::decode_0x93(const uint8_t * data, size_t len)
{
  if (len < 8) {
    return;
  }

  const auto rem_m_ah =
    (static_cast<uint32_t>(data[4]) << 24) |
    (static_cast<uint32_t>(data[5]) << 16) |
    (static_cast<uint32_t>(data[6]) << 8) |
    static_cast<uint32_t>(data[7]);

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = data[0];
  charge_mos_ = data[1];
  discharge_mos_ = data[2];
  bms_life_cycles_ = data[3];
  remaining_capacity_m_ah_ = rem_m_ah;
}

void BatteryNode::decode_0x94(const uint8_t * data, size_t len)
{
  if (len < 5) {
    return;
  }

  std::vector<int> di_states(4, 0);
  std::vector<int> do_states(4, 0);
  for (size_t i = 0; i < 4; ++i) {
    di_states[i] = (data[4] >> i) & 0x1;
    do_states[i] = (data[4] >> (4 + i)) & 0x1;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  series_cells_ = data[0];
  temp_sensors_ = data[1];
  charger_connected_ = (data[2] != 0);
  load_connected_ = (data[3] != 0);
  di_states_ = std::move(di_states);
  do_states_ = std::move(do_states);

  if (series_cells_ && cell_voltages_.size() < *series_cells_) {
    cell_voltages_.resize(*series_cells_, 0.0);
  }
  if (temp_sensors_ && cell_temperatures_.size() < *temp_sensors_) {
    cell_temperatures_.resize(*temp_sensors_, 0.0);
  }
  if (series_cells_ && balance_state_.size() < *series_cells_) {
    balance_state_.resize(*series_cells_, 0);
  }
}

void BatteryNode::decode_0x95(const uint8_t * data, size_t len)
{
  if (len < 7 || data[0] == 0xFF) {
    return;
  }

  const auto frame = data[0];
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < 3; ++i) {
    const auto offset = 1 + i * 2;
    if (offset + 1 >= len) {
      break;
    }

    const auto raw_mv = static_cast<uint16_t>((data[offset] << 8) | data[offset + 1]);
    const auto cell_index = static_cast<size_t>((frame - 1) * 3 + i);

    if (cell_voltages_.size() <= cell_index) {
      cell_voltages_.resize(cell_index + 1, 0.0);
    }
    cell_voltages_[cell_index] = raw_mv / 1000.0;
  }
}

void BatteryNode::decode_0x96(const uint8_t * data, size_t len)
{
  if (len < 2) {
    return;
  }

  const auto frame = data[0];
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto max_sensors = temp_sensors_.value_or(0);

  for (size_t pos = 1; pos < len; ++pos) {
    const auto temp_index = static_cast<size_t>(frame * 7 + (pos - 1));
    if (max_sensors != 0 && temp_index >= max_sensors) {
      break;
    }

    const auto temp_raw = data[pos];
    if (temp_raw == 0xFF) {
      continue;
    }

    if (cell_temperatures_.size() <= temp_index) {
      cell_temperatures_.resize(temp_index + 1, 0.0);
    }
    cell_temperatures_[temp_index] = static_cast<double>(static_cast<int>(temp_raw) - 40);
  }
}

void BatteryNode::decode_0x97(const uint8_t * data, size_t len)
{
  if (len < 8) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto num_cells = std::max<size_t>(balance_state_.size(), 48);
  balance_state_.assign(num_cells, 0);

  for (size_t bit = 0; bit < 48; ++bit) {
    const auto byte_idx = bit / 8;
    const auto bit_idx = bit % 8;
    balance_state_[bit] = (data[byte_idx] >> bit_idx) & 0x1;
  }
}

void BatteryNode::decode_0x98(const uint8_t * data, size_t len)
{
  if (len < 8) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  failure_bytes_ = std::vector<uint8_t>(data, data + 8);
}

diagnostic_msgs::msg::KeyValue BatteryNode::make_kv(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

void BatteryNode::publish_battery_state()
{
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = now();

  std::optional<double> voltage;
  std::optional<double> current;
  std::optional<double> soc;
  std::optional<uint8_t> state;
  std::vector<double> cell_voltages;
  std::vector<double> cell_temperatures;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    voltage = pack_voltage_;
    current = pack_current_;
    soc = soc_percent_;
    state = state_;
    cell_voltages = cell_voltages_;
    cell_temperatures = cell_temperatures_;
  }

  if (voltage) {
    msg.voltage = static_cast<float>(*voltage);
  }
  if (current) {
    msg.current = static_cast<float>(*current);
  }
  if (soc) {
    msg.percentage = static_cast<float>(*soc / 100.0);
  }
  if (!cell_voltages.empty()) {
    msg.cell_voltage.assign(cell_voltages.begin(), cell_voltages.end());
  }
  if (!cell_temperatures.empty()) {
    msg.cell_temperature.assign(cell_temperatures.begin(), cell_temperatures.end());
    msg.temperature = static_cast<float>(*std::max_element(cell_temperatures.begin(), cell_temperatures.end()));
  }

  if (state) {
    if (*state == 1) {
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else if (*state == 2) {
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
  }

  batt_pub_->publish(msg);
}

void BatteryNode::publish_battery_health()
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "daly_bms_health";
  status.hardware_id = "daly_bms_can";

  std::optional<std::vector<uint8_t>> failure_bytes;
  std::optional<uint8_t> cycles;
  std::optional<uint32_t> rem_capacity;
  std::optional<double> max_v;
  std::optional<uint8_t> max_v_idx;
  std::optional<double> min_v;
  std::optional<uint8_t> min_v_idx;
  std::optional<double> max_t;
  std::optional<uint8_t> max_t_idx;
  std::optional<double> min_t;
  std::optional<uint8_t> min_t_idx;
  std::optional<bool> charger;
  std::optional<bool> load;
  std::optional<uint8_t> series_cells;
  std::optional<uint8_t> temp_sensors;
  std::vector<int> balances;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    failure_bytes = failure_bytes_;
    cycles = bms_life_cycles_;
    rem_capacity = remaining_capacity_m_ah_;
    max_v = max_cell_voltage_;
    max_v_idx = max_cell_index_;
    min_v = min_cell_voltage_;
    min_v_idx = min_cell_index_;
    max_t = max_temp_;
    max_t_idx = max_temp_index_;
    min_t = min_temp_;
    min_t_idx = min_temp_index_;
    charger = charger_connected_;
    load = load_connected_;
    series_cells = series_cells_;
    temp_sensors = temp_sensors_;
    balances = balance_state_;
  }

  std::vector<diagnostic_msgs::msg::KeyValue> values;

  if (series_cells) {
    values.push_back(make_kv("series_cells", std::to_string(*series_cells)));
  }
  if (temp_sensors) {
    values.push_back(make_kv("temp_sensors", std::to_string(*temp_sensors)));
  }
  if (cycles) {
    values.push_back(make_kv("bms_life_cycles", std::to_string(*cycles)));
  }
  if (rem_capacity) {
    values.push_back(make_kv("remaining_capacity_mAh", std::to_string(*rem_capacity)));
  }

  if (max_v) {
    values.push_back(make_kv("max_cell_voltage_V", format_number(*max_v, 3)));
  }
  if (max_v_idx) {
    values.push_back(make_kv("max_cell_index", std::to_string(*max_v_idx)));
  }
  if (min_v) {
    values.push_back(make_kv("min_cell_voltage_V", format_number(*min_v, 3)));
  }
  if (min_v_idx) {
    values.push_back(make_kv("min_cell_index", std::to_string(*min_v_idx)));
  }
  if (max_t) {
    values.push_back(make_kv("max_temperature_C", format_number(*max_t, 1)));
  }
  if (max_t_idx) {
    values.push_back(make_kv("max_temp_index", std::to_string(*max_t_idx)));
  }
  if (min_t) {
    values.push_back(make_kv("min_temperature_C", format_number(*min_t, 1)));
  }
  if (min_t_idx) {
    values.push_back(make_kv("min_temp_index", std::to_string(*min_t_idx)));
  }
  if (charger) {
    values.push_back(make_kv("charger_connected", *charger ? "True" : "False"));
  }
  if (load) {
    values.push_back(make_kv("load_connected", *load ? "True" : "False"));
  }
  for (auto type : {
      ProtectionType::OverVoltage,
      ProtectionType::UnderVoltage,
      ProtectionType::ChargingOverCurrent,
      ProtectionType::ChargingUnderCurrent,
      ProtectionType::ChargingOverTemperature,
      ProtectionType::ChargingUnderTemperature,
      ProtectionType::DischargingOverCurrent,
      ProtectionType::DischargingUnderCurrent,
      ProtectionType::DischargingOverTemperature,
      ProtectionType::DischargingUnderTemperature})
  {
    const auto & state = protection_states_[protection_index(type)];
    values.push_back(make_kv(
      std::string("protection.") + protection_key(type) + ".enabled",
      state.limit.enabled ? "True" : "False"));
    values.push_back(make_kv(
      std::string("protection.") + protection_key(type) + ".active",
      state.active ? "True" : "False"));
    if (state.limit.enabled) {
      values.push_back(make_kv(
        std::string("protection.") + protection_key(type) + ".threshold",
        format_number(state.limit.threshold, 2) + " " + state.limit.units));
    }
  }

  std::vector<std::string> balancing_cells;
  for (size_t i = 0; i < balances.size(); ++i) {
    if (balances[i] != 0) {
      balancing_cells.push_back(std::to_string(i + 1));
    }
  }
  if (!balancing_cells.empty()) {
    values.push_back(make_kv("balancing_cells", join_strings(balancing_cells, ",")));
  }

  if (failure_bytes) {
    const auto faults = decode_fault_messages(*failure_bytes);
    values.push_back(make_kv("failure_bytes_hex", bytes_to_hex(*failure_bytes)));
    if (!faults.empty()) {
      values.push_back(make_kv("active_faults", join_strings(faults, "; ")));
    }

    status.level = faults.empty() ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = faults.empty() ? "No faults" : "Fault(s) present";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "No failure frame (0x98) received yet";
  }

  status.values = std::move(values);
  diag_array.status.push_back(status);
  health_pub_->publish(diag_array);
}

void BatteryNode::evaluate_protections()
{
  std::optional<double> voltage;
  std::optional<double> current;
  std::optional<double> max_temperature;
  std::optional<double> min_temperature;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    voltage = pack_voltage_;
    current = pack_current_;
    max_temperature = max_temp_;
    min_temperature = min_temp_;
  }

  const bool charging_active = current.has_value() && *current > 0.0;
  const bool discharging_active = current.has_value() && *current < 0.0;

  const auto evaluate_limit = [this](
      ProtectionType type,
      const std::optional<double> & measured_value,
      const std::string & signal_name,
      const std::string & comparator_text,
      const auto & comparator) {
      auto & state = protection_states_[protection_index(type)];
      if (!state.limit.enabled || !measured_value.has_value()) {
        clear_protection_state(type);
        return;
      }

      if (comparator(*measured_value, state.limit.threshold)) {
        if (!state.active) {
          publish_safety_stop(
            type, signal_name, *measured_value, state.limit, comparator_text);
          state.active = true;
        }
        return;
      }

      state.active = false;
    };

  evaluate_limit(
    ProtectionType::OverVoltage, voltage, "pack_voltage", "exceeded",
    [](double measured, double threshold) { return measured > threshold; });
  evaluate_limit(
    ProtectionType::UnderVoltage, voltage, "pack_voltage", "fell below",
    [](double measured, double threshold) { return measured < threshold; });

  if (charging_active) {
    evaluate_limit(
      ProtectionType::ChargingOverCurrent, current, "pack_current", "exceeded",
      [](double measured, double threshold) { return measured > threshold; });
    evaluate_limit(
      ProtectionType::ChargingUnderCurrent, current, "pack_current", "fell below",
      [](double measured, double threshold) { return measured < threshold; });
    evaluate_limit(
      ProtectionType::ChargingOverTemperature, max_temperature, "max_temperature", "exceeded",
      [](double measured, double threshold) { return measured > threshold; });
    evaluate_limit(
      ProtectionType::ChargingUnderTemperature, min_temperature, "min_temperature", "fell below",
      [](double measured, double threshold) { return measured < threshold; });
  } else {
    clear_protection_state(ProtectionType::ChargingOverCurrent);
    clear_protection_state(ProtectionType::ChargingUnderCurrent);
    clear_protection_state(ProtectionType::ChargingOverTemperature);
    clear_protection_state(ProtectionType::ChargingUnderTemperature);
  }

  if (discharging_active) {
    evaluate_limit(
      ProtectionType::DischargingOverCurrent, current, "pack_current", "fell below",
      [](double measured, double threshold) { return measured < threshold; });
    evaluate_limit(
      ProtectionType::DischargingUnderCurrent, current, "pack_current", "rose above",
      [](double measured, double threshold) { return measured > threshold; });
    evaluate_limit(
      ProtectionType::DischargingOverTemperature, max_temperature, "max_temperature", "exceeded",
      [](double measured, double threshold) { return measured > threshold; });
    evaluate_limit(
      ProtectionType::DischargingUnderTemperature, min_temperature, "min_temperature", "fell below",
      [](double measured, double threshold) { return measured < threshold; });
  } else {
    clear_protection_state(ProtectionType::DischargingOverCurrent);
    clear_protection_state(ProtectionType::DischargingUnderCurrent);
    clear_protection_state(ProtectionType::DischargingOverTemperature);
    clear_protection_state(ProtectionType::DischargingUnderTemperature);
  }
}

void BatteryNode::clear_protection_state(ProtectionType type)
{
  protection_states_[protection_index(type)].active = false;
}

void BatteryNode::publish_safety_stop(
  ProtectionType type,
  const std::string & signal_name,
  double measured_value,
  const ProtectionLimit & limit,
  const std::string & comparator_text)
{
  amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
  stop_msg.stamp = to_builtin_time(now());
  stop_msg.sender = safety_stop_sender_name_;

  std::ostringstream reason;
  reason << "battery protection fault: " << protection_key(type)
         << " on " << signal_name
         << " measured=" << format_number(measured_value, 2) << " " << limit.units
         << " " << comparator_text
         << " threshold=" << format_number(limit.threshold, 2) << " " << limit.units;
  stop_msg.reason = reason.str();

  safety_stop_pub_->publish(stop_msg);
  RCLCPP_ERROR(get_logger(), "%s", stop_msg.reason.c_str());
}

std::string BatteryNode::format_number(double value, int precision)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

std::string BatteryNode::bytes_to_hex(const std::vector<uint8_t> & bytes)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (const auto byte : bytes) {
    oss << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

std::string BatteryNode::join_strings(
  const std::vector<std::string> & items,
  const std::string & delim)
{
  std::ostringstream oss;
  for (size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {
      oss << delim;
    }
    oss << items[i];
  }
  return oss.str();
}

std::vector<std::string> BatteryNode::decode_fault_messages(
  const std::vector<uint8_t> & failure_bytes)
{
  static const std::vector<std::pair<std::pair<int, int>, std::string>> bit_descriptions{
    {{0, 0}, "Cell voltage high level 1"},
    {{0, 1}, "Cell voltage high level 2"},
    {{0, 2}, "Cell voltage low level 1"},
    {{0, 3}, "Cell voltage low level 2"},
    {{0, 4}, "Pack voltage high level 1"},
    {{0, 5}, "Pack voltage high level 2"},
    {{0, 6}, "Pack voltage low level 1"},
    {{0, 7}, "Pack voltage low level 2"},
    {{1, 0}, "Charge temp high level 1"},
    {{1, 1}, "Charge temp high level 2"},
    {{1, 2}, "Charge temp low level 1"},
    {{1, 3}, "Charge temp low level 2"},
    {{1, 4}, "Discharge temp high level 1"},
    {{1, 5}, "Discharge temp high level 2"},
    {{1, 6}, "Discharge temp low level 1"},
    {{1, 7}, "Discharge temp low level 2"},
    {{2, 0}, "Charge overcurrent level 1"},
    {{2, 1}, "Charge overcurrent level 2"},
    {{2, 2}, "Discharge overcurrent level 1"},
    {{2, 3}, "Discharge overcurrent level 2"},
    {{2, 4}, "SOC high level 1"},
    {{2, 5}, "SOC high level 2"},
    {{2, 6}, "SOC low level 1"},
    {{2, 7}, "SOC low level 2"},
    {{3, 0}, "Voltage difference level 1"},
    {{3, 1}, "Voltage difference level 2"},
    {{3, 2}, "Temperature difference level 1"},
    {{3, 3}, "Temperature difference level 2"},
    {{4, 0}, "Charge MOS temp high alarm"},
    {{4, 1}, "Discharge MOS temp high alarm"},
    {{4, 2}, "Charge MOS temp sensor error"},
    {{4, 3}, "Discharge MOS temp sensor error"},
    {{4, 4}, "Charge MOS adhesion error"},
    {{4, 5}, "Discharge MOS adhesion error"},
    {{4, 6}, "Charge MOS open circuit error"},
    {{4, 7}, "Discharge MOS open circuit error"},
    {{5, 0}, "AFE collect chip error"},
    {{5, 1}, "Voltage collect dropped"},
    {{5, 2}, "Cell temp sensor error"},
    {{5, 3}, "EEPROM error"},
    {{5, 4}, "RTC error"},
    {{5, 5}, "Precharge failure"},
    {{5, 6}, "Communication failure"},
    {{5, 7}, "Internal communication failure"},
    {{6, 0}, "Current module fault"},
    {{6, 1}, "Pack voltage detect fault"},
    {{6, 2}, "Short circuit protection fault"},
    {{6, 3}, "Low voltage forbidden charge fault"},
  };

  std::vector<std::string> faults;
  for (size_t byte_index = 0; byte_index < std::min<size_t>(7, failure_bytes.size()); ++byte_index) {
    const auto byte = failure_bytes[byte_index];
    if (byte == 0) {
      continue;
    }

    for (int bit = 0; bit < 8; ++bit) {
      if ((byte & (1 << bit)) == 0) {
        continue;
      }

      const auto iter = std::find_if(
        bit_descriptions.begin(), bit_descriptions.end(),
        [byte_index, bit](const auto & entry) {
          return entry.first.first == static_cast<int>(byte_index) && entry.first.second == bit;
        });

      if (iter != bit_descriptions.end()) {
        faults.push_back(iter->second);
      } else {
        faults.push_back(
          "Unknown fault byte" + std::to_string(byte_index) + "_bit" + std::to_string(bit));
      }
    }
  }

  return faults;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
