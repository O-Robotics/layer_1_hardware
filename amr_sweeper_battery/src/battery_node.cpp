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
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class DalyBmsCanNode : public rclcpp::Node
{
public:
  DalyBmsCanNode()
  : Node("amr_sweeper_battery")
  {
    declare_parameter<std::string>("can_interface", "can0");
    declare_parameter<double>("timer_period", 15.0);
    declare_parameter<int64_t>("priority", 0x18);
    declare_parameter<int64_t>("bms_address", 0x01);
    declare_parameter<int64_t>("pc_address", 0x40);

    can_interface_ = get_parameter("can_interface").as_string();
    const auto timer_period = get_parameter("timer_period").as_double();
    priority_ = static_cast<uint8_t>(get_parameter("priority").as_int());
    bms_addr_ = static_cast<uint8_t>(get_parameter("bms_address").as_int());
    pc_addr_ = static_cast<uint8_t>(get_parameter("pc_address").as_int());

    RCLCPP_INFO(
      get_logger(),
      "Using interface %s, 29-bit IDs, prio=0x%02X, BMS=0x%02X, PC=0x%02X",
      can_interface_.c_str(), priority_, bms_addr_, pc_addr_);

    batt_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("battery_health", 10);

    if (!setup_can_socket(true)) {
      RCLCPP_WARN(
        get_logger(),
        "CAN interface '%s' is not available at startup; will keep retrying in the background.",
        can_interface_.c_str());
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timer_period));
    timer_ = create_wall_timer(period, std::bind(&DalyBmsCanNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "amr_sweeper_battery node started.");
  }

  ~DalyBmsCanNode() override
  {
    close_can_socket();
  }

private:
  static constexpr std::array<uint8_t, 9> kDataIds{{0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98}};

  struct ParsedId
  {
    uint8_t data_id;
    uint8_t src_addr;
    uint8_t dst_addr;
    uint8_t priority;
  };

  bool setup_can_socket(bool log_failure)
  {
    std::lock_guard<std::mutex> socket_lock(socket_mutex_);
    if (can_socket_ >= 0) {
      return true;
    }

    const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
      if (log_failure) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to create CAN socket for '%s': %s. Will retry periodically.",
          can_interface_.c_str(), std::strerror(errno));
      }
      return false;
    }

    ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      if (log_failure) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to resolve CAN interface '%s': %s. Will retry periodically.",
          can_interface_.c_str(), std::strerror(errno));
      }
      ::close(fd);
      return false;
    }

    sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      if (log_failure) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to bind CAN interface '%s': %s. Will retry periodically.",
          can_interface_.c_str(), std::strerror(errno));
      }
      ::close(fd);
      return false;
    }

    can_socket_ = fd;
    rx_running_.store(true);
    rx_thread_ = std::thread(&DalyBmsCanNode::rx_loop, this);
    missing_can_warned_ = false;

    RCLCPP_INFO(get_logger(), "Connected to CAN interface '%s'.", can_interface_.c_str());
    return true;
  }

  void close_can_socket()
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

    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
  }

  int current_socket() const
  {
    std::lock_guard<std::mutex> socket_lock(socket_mutex_);
    return can_socket_;
  }

  uint32_t make_pc_to_bms_id(uint8_t data_id) const
  {
    return
      (static_cast<uint32_t>(priority_) << 24) |
      (static_cast<uint32_t>(data_id) << 16) |
      (static_cast<uint32_t>(bms_addr_) << 8) |
      static_cast<uint32_t>(pc_addr_);
  }

  static ParsedId parse_bms_to_pc_id(uint32_t arb_id)
  {
    ParsedId parsed {};
    parsed.priority = static_cast<uint8_t>((arb_id >> 24) & 0xFF);
    parsed.data_id = static_cast<uint8_t>((arb_id >> 16) & 0xFF);
    parsed.dst_addr = static_cast<uint8_t>((arb_id >> 8) & 0xFF);
    parsed.src_addr = static_cast<uint8_t>(arb_id & 0xFF);
    return parsed;
  }

  void on_timer()
  {
    if (current_socket() < 0) {
      if (!missing_can_warned_) {
        RCLCPP_WARN(
          get_logger(),
          "No CAN interface '%s' detected yet; battery data will not be updated until it appears.",
          can_interface_.c_str());
        missing_can_warned_ = true;
      }

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

      setup_can_socket(false);
      return;
    }

    for (const auto data_id : kDataIds) {
      if (!send_request(data_id)) {
        RCLCPP_WARN(get_logger(), "CAN TX failed for 0x%02X", data_id);
      }
    }

    publish_battery_state();
    publish_battery_health();
  }

  bool send_request(uint8_t data_id)
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
      if (errno == ENETDOWN || errno == ENODEV || errno == EBADF) {
        close_can_socket();
      }
      return false;
    }
    return true;
  }

  void rx_loop()
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
      if (ready <= 0) {
        continue;
      }

      can_frame frame {};
      const auto bytes = read(fd, &frame, sizeof(frame));
      if (bytes != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }

      handle_can_frame(frame);
    }
  }

  void handle_can_frame(const can_frame & frame)
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

  void decode_0x90(const uint8_t * data, size_t len)
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

  void decode_0x91(const uint8_t * data, size_t len)
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

  void decode_0x92(const uint8_t * data, size_t len)
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

  void decode_0x93(const uint8_t * data, size_t len)
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

  void decode_0x94(const uint8_t * data, size_t len)
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

  void decode_0x95(const uint8_t * data, size_t len)
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

  void decode_0x96(const uint8_t * data, size_t len)
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

  void decode_0x97(const uint8_t * data, size_t len)
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

  void decode_0x98(const uint8_t * data, size_t len)
  {
    if (len < 8) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    failure_bytes_ = std::vector<uint8_t>(data, data + 8);
  }

  static diagnostic_msgs::msg::KeyValue make_kv(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  void publish_battery_state()
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

  void publish_battery_health()
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

  static std::string format_number(double value, int precision)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  static std::string bytes_to_hex(const std::vector<uint8_t> & bytes)
  {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (const auto byte : bytes) {
      oss << std::setw(2) << static_cast<int>(byte);
    }
    return oss.str();
  }

  static std::string join_strings(const std::vector<std::string> & items, const std::string & delim)
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

  static std::vector<std::string> decode_fault_messages(const std::vector<uint8_t> & failure_bytes)
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

  std::string can_interface_;
  uint8_t priority_ {};
  uint8_t bms_addr_ {};
  uint8_t pc_addr_ {};

  mutable std::mutex socket_mutex_;
  int can_socket_ {-1};
  std::atomic<bool> rx_running_ {false};
  std::thread rx_thread_;
  bool missing_can_warned_ {false};

  mutable std::mutex state_mutex_;
  std::optional<double> pack_voltage_;
  std::optional<double> pack_current_;
  std::optional<double> soc_percent_;
  std::optional<double> max_cell_voltage_;
  std::optional<uint8_t> max_cell_index_;
  std::optional<double> min_cell_voltage_;
  std::optional<uint8_t> min_cell_index_;
  std::optional<double> max_temp_;
  std::optional<uint8_t> max_temp_index_;
  std::optional<double> min_temp_;
  std::optional<uint8_t> min_temp_index_;
  std::optional<uint8_t> state_;
  std::optional<uint8_t> charge_mos_;
  std::optional<uint8_t> discharge_mos_;
  std::optional<uint8_t> bms_life_cycles_;
  std::optional<uint32_t> remaining_capacity_m_ah_;
  std::optional<uint8_t> series_cells_;
  std::optional<uint8_t> temp_sensors_;
  std::optional<bool> charger_connected_;
  std::optional<bool> load_connected_;
  std::vector<int> di_states_;
  std::vector<int> do_states_;
  std::vector<double> cell_voltages_;
  std::vector<double> cell_temperatures_;
  std::vector<int> balance_state_;
  std::optional<std::vector<uint8_t>> failure_bytes_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DalyBmsCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
