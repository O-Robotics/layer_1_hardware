#ifndef AMR_SWEEPER_BATTERY__BATTERY_NODE_HPP_
#define AMR_SWEEPER_BATTERY__BATTERY_NODE_HPP_

#include <linux/can.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "amr_sweeper_safety_msgs/msg/safety_stop.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode();
  ~BatteryNode() override;

  static constexpr std::array<uint8_t, 9> kDataIds{{0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98}};

  enum class ProtectionType : std::size_t
  {
    OverVoltage = 0,
    UnderVoltage = 1,
    ChargingOverCurrent = 2,
    ChargingUnderCurrent = 3,
    ChargingOverTemperature = 4,
    ChargingUnderTemperature = 5,
    DischargingOverCurrent = 6,
    DischargingUnderCurrent = 7,
    DischargingOverTemperature = 8,
    DischargingUnderTemperature = 9,
    Count = 10,
  };

  struct ParsedId
  {
    uint8_t data_id;
    uint8_t src_addr;
    uint8_t dst_addr;
    uint8_t priority;
  };

  struct ProtectionLimit
  {
    bool enabled{false};
    double threshold{0.0};
    std::string units;
  };

  struct ProtectionState
  {
    ProtectionLimit limit;
    bool active{false};
  };

private:

  bool setup_can_socket(bool log_failure);
  void close_can_socket();
  int current_socket() const;
  void enter_fatal_state(const std::string & message);
  void report_connection_issue(const std::string & message);
  void log_escalating_issue(int count, const std::string & message);
  void reset_issue_counters();
  uint32_t make_pc_to_bms_id(uint8_t data_id) const;
  static ParsedId parse_bms_to_pc_id(uint32_t arb_id);
  void on_timer();
  bool send_request(uint8_t data_id);
  void rx_loop();
  void evaluate_protections();
  void clear_protection_state(ProtectionType type);
  void publish_safety_stop(
    ProtectionType type,
    const std::string & signal_name,
    double measured_value,
    const ProtectionLimit & limit,
    const std::string & comparator_text);
  void handle_can_frame(const can_frame & frame);
  void decode_0x90(const uint8_t * data, size_t len);
  void decode_0x91(const uint8_t * data, size_t len);
  void decode_0x92(const uint8_t * data, size_t len);
  void decode_0x93(const uint8_t * data, size_t len);
  void decode_0x94(const uint8_t * data, size_t len);
  void decode_0x95(const uint8_t * data, size_t len);
  void decode_0x96(const uint8_t * data, size_t len);
  void decode_0x97(const uint8_t * data, size_t len);
  void decode_0x98(const uint8_t * data, size_t len);
  static diagnostic_msgs::msg::KeyValue make_kv(const std::string & key, const std::string & value);
  void publish_battery_state();
  void publish_battery_health();
  static std::string format_number(double value, int precision);
  static std::string bytes_to_hex(const std::vector<uint8_t> & bytes);
  static std::string join_strings(const std::vector<std::string> & items, const std::string & delim);
  static std::vector<std::string> decode_fault_messages(const std::vector<uint8_t> & failure_bytes);

  std::string can_interface_;
  uint8_t priority_ {};
  uint8_t bms_addr_ {};
  uint8_t pc_addr_ {};
  int retry_attempts_before_error_ {3};
  int fatal_after_consecutive_errors_ {10};
  int max_reconnect_attempts_ {10};

  mutable std::mutex socket_mutex_;
  int can_socket_ {-1};
  std::atomic<bool> rx_running_ {false};
  std::thread rx_thread_;
  bool missing_can_warned_ {false};
  mutable std::mutex issue_mutex_;
  int reconnect_attempt_count_ {0};
  int connection_issue_count_ {0};
  std::atomic<bool> fatal_error_ {false};
  std::string last_connection_error_message_;

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
  std::array<ProtectionState, static_cast<std::size_t>(ProtectionType::Count)> protection_states_{};

  std::string safety_stop_topic_name_{"safety_msgs/stop"};
  std::string safety_stop_sender_name_{"battery_node"};

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Publisher<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr safety_stop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // AMR_SWEEPER_BATTERY__BATTERY_NODE_HPP_
