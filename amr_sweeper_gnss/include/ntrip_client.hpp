#ifndef AMR_SWEEPER_GNSS__NTRIP_CLIENT_HPP_
#define AMR_SWEEPER_GNSS__NTRIP_CLIENT_HPP_

#include <openssl/ssl.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace amr_sweeper_gnss
{

class NtripClientNode : public rclcpp::Node
{
public:
  NtripClientNode();
  ~NtripClientNode() override;

  bool has_fatal_error() const;
  int fatal_after_consecutive_errors() const;
  double startup_retry_seconds() const;

private:
  void run();
  void connect_and_stream();
  int create_connected_socket() const;
  void configure_socket_timeouts(int socket_fd) const;
  void connect_ssl();
  void send_request();
  void handle_rtcm_bytes(const std::vector<std::uint8_t> & chunk);
  std::optional<std::vector<std::uint8_t>> extract_rtcm_packet();
  bool validate_rtcm_packet(const std::vector<std::uint8_t> & packet);
  bool looks_like_sourcetable(
    const std::string & header,
    const std::vector<std::uint8_t> & payload) const;
  void handle_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  std::string build_gga_sentence(const sensor_msgs::msg::NavSatFix & fix) const;
  std::string build_request() const;
  std::pair<std::string, std::vector<std::uint8_t>> split_response(
    const std::vector<std::uint8_t> & response) const;
  std::string active_mountpoint() const;
  void advance_mountpoint();
  void close_socket();
  bool stream_timed_out() const;
  void report_connection_issue(const std::string & message);
  void report_bad_rtcm_issue(const std::string & message);
  void log_escalating_issue(int count, const std::string & message, const std::string & issue_type);
  void reset_issue_counters();
  int recv_bytes(std::uint8_t * buffer, std::size_t length);
  void send_bytes(const std::uint8_t * data, std::size_t length);

  static std::string dd_to_dmm(double value, bool is_latitude);
  static std::uint8_t nmea_checksum(const std::string & sentence);
  static std::uint32_t crc24q(const std::vector<std::uint8_t> & data);

  std::string host_;
  int port_{2101};
  std::string mountpoint_;
  std::string alternate_mountpoint_;
  std::vector<std::string> mountpoints_;
  std::size_t active_mountpoint_index_{0};
  int mountpoint_failover_threshold_{2};
  int current_mountpoint_failure_count_{0};
  std::string ntrip_version_;
  bool authenticate_{false};
  std::string username_;
  std::string password_;
  bool use_ssl_{false};
  std::string cert_;
  std::string key_;
  std::string ca_cert_;
  std::string rtcm_frame_id_;
  double startup_retry_seconds_{5.0};
  double reconnect_wait_{5.0};
  double failed_connection_retry_{5.0};
  double socket_timeout_seconds_{10.0};
  double initial_rtcm_grace_seconds_{10.0};
  double rtcm_timeout_seconds_{4.0};
  int retry_attempts_before_error_{3};
  int fatal_after_consecutive_errors_{10};
  bool send_nmea_{false};

  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscription_;

  mutable std::mutex socket_mutex_;
  int socket_fd_{-1};
  SSL_CTX * ssl_context_{nullptr};
  SSL * ssl_handle_{nullptr};

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> fatal_error_{false};
  std::thread worker_thread_;
  std::vector<std::uint8_t> parser_buffer_;
  std::optional<std::chrono::steady_clock::time_point> last_valid_rtcm_time_;
  std::optional<std::chrono::steady_clock::time_point> connected_at_;
  int connection_issue_count_{0};
  int bad_rtcm_issue_count_{0};
  std::string fatal_error_message_;
};

double read_startup_retry_seconds_from_params(int argc, char ** argv);
int read_startup_fatal_threshold_from_params(int argc, char ** argv);

}  // namespace amr_sweeper_gnss

#endif  // AMR_SWEEPER_GNSS__NTRIP_CLIENT_HPP_
