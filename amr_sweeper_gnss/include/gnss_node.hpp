#ifndef AMR_SWEEPER_GNSS__GNSS_NODE_HPP_
#define AMR_SWEEPER_GNSS__GNSS_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <termios.h>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace amr_sweeper_gnss
{

class UbloxNode : public rclcpp::Node
{
public:
  explicit UbloxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~UbloxNode() override;

private:
  struct NavHpPosLlh
  {
    std::uint32_t itow{0};
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    double horizontal_accuracy_m{0.0};
    double vertical_accuracy_m{0.0};
    std::uint8_t invalid_flags{0};
  };

  struct NavStatus
  {
    std::uint32_t itow{0};
    std::uint8_t gps_fix{0};
    std::uint8_t flags{0};
    std::uint8_t fix_stat{0};
    std::uint8_t flags2{0};
  };

  struct NavCov
  {
    std::uint32_t itow{0};
    bool position_covariance_valid{false};
    double pos_cov_nn{0.0};
    double pos_cov_ne{0.0};
    double pos_cov_nd{0.0};
    double pos_cov_ee{0.0};
    double pos_cov_ed{0.0};
    double pos_cov_dd{0.0};
  };

  struct NavPvt
  {
    std::uint32_t itow{0};
    std::uint8_t fix_type{0};
    std::uint8_t flags{0};
    std::uint8_t flags2{0};
    std::uint8_t num_sv{0};
    double ground_speed_mps{0.0};
    double vertical_speed_mps{0.0};
    double heading_deg{0.0};
    double pdop{0.0};
  };

  enum class ConfigValueType
  {
    Bool,
    U1,
    U2,
  };

  struct ConfigItem
  {
    const char * name;
    std::uint32_t key;
    ConfigValueType type;
    std::uint32_t value;
  };

  struct ConstellationConfig
  {
    bool gps{true};
    bool sbas{true};
    bool galileo{true};
    bool beidou{true};
    bool qzss{false};
    bool glonass{true};
  };

  void loadParameters();
  void onRtcmMessage(const rtcm_msgs::msg::Message::SharedPtr msg);
  void enterFatalState(const std::string & message);
  void reportConnectionIssue(const std::string & message);
  void reportConfigurationIssue(const std::string & message);
  void logEscalatingIssue(int count, const std::string & message, const std::string & issue_type);
  void resetIssueCounters();

  void run();
  bool openDevice();
  void closeDevice();
  bool configureReceiver();
  void requestEssentialPolls();
  bool sendConfigBatch(const std::vector<ConfigItem> & items);
  bool sendConfigItem(const ConfigItem & item);
  bool sendFrame(std::uint8_t msg_class, std::uint8_t msg_id, const std::vector<std::uint8_t> & payload);
  bool writeRaw(const std::uint8_t * data, std::size_t size);

  void readFromDevice();
  void parseIncomingBytes(const std::uint8_t * data, std::size_t size);
  void processFrame(std::uint8_t msg_class, std::uint8_t msg_id, const std::vector<std::uint8_t> & payload);

  void handleAckAck(const std::vector<std::uint8_t> & payload);
  void handleAckNak(const std::vector<std::uint8_t> & payload);
  void handleNavHpPosLlh(const std::vector<std::uint8_t> & payload);
  void handleNavStatus(const std::vector<std::uint8_t> & payload);
  void handleNavCov(const std::vector<std::uint8_t> & payload);
  void handleNavPvt(const std::vector<std::uint8_t> & payload);
  void tryPublishNavSat();

  static std::uint16_t computeChecksumA(std::uint8_t msg_class, std::uint8_t msg_id, const std::vector<std::uint8_t> & payload);
  static std::uint32_t readU32(const std::vector<std::uint8_t> & data, std::size_t offset);
  static std::int32_t readI32(const std::vector<std::uint8_t> & data, std::size_t offset);
  static float readF32(const std::vector<std::uint8_t> & data, std::size_t offset);
  static speed_t toTermiosBaud(int baud_rate);
  static int dynamicModelIdFromName(const std::string & name, int fallback);
  static int fixModeIdFromName(const std::string & name, int fallback);
  std::uint16_t navSatServiceMask() const;
  void logReceiverConfigurationSummary() const;

  std::string device_path_;
  int baud_rate_{115200};
  std::string frame_id_{"gnss_link"};
  std::string navsat_topic_{"navsat"};
  std::string gpsfix_topic_{"fix"};
  std::string rtcm_topic_{"ntrip_client/rtcm"};
  double reconnect_delay_seconds_{2.0};
  double publish_timeout_seconds_{1.0};
  bool configure_on_connect_{true};
  double poll_interval_seconds_{1.0};
  int retry_attempts_before_error_{3};
  int fatal_after_consecutive_errors_{10};
  int max_reconnect_attempts_{10};

  bool usb_in_rtcm3x_{true};
  bool usb_out_ubx_{true};
  bool usb_out_nmea_{false};
  bool usb_out_rtcm3x_{false};
  int measurement_rate_ms_{200};
  int navigation_rate_cycles_{1};
  int fix_mode_{2};
  std::string fix_mode_name_;
  bool require_initial_3d_fix_{true};
  int dynamic_model_{4};
  std::string dynamic_model_name_;
  int nav_hpposllh_rate_{1};
  int nav_status_rate_{5};
  int nav_cov_rate_{1};
  int nav_pvt_rate_{1};
  ConstellationConfig constellations_{};

  int min_fix_type_{3};
  double min_horizontal_stddev_m_{1.5};
  double min_vertical_stddev_m_{3.0};
  double horizontal_covariance_scale_{4.0};
  double vertical_covariance_scale_{4.0};
  bool use_hacc_vacc_covariance_floor_{true};

  int device_fd_{-1};
  std::mutex device_mutex_;
  std::vector<std::uint8_t> parser_buffer_;
  std::deque<ConfigItem> pending_config_acks_;
  std::set<std::uint32_t> rejected_config_keys_;

  std::mutex nav_mutex_;
  std::optional<NavHpPosLlh> last_hpposllh_;
  std::optional<NavStatus> last_status_;
  std::optional<NavCov> last_cov_;
  std::optional<NavPvt> last_pvt_;
  rclcpp::Time last_fix_publish_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_poll_request_time_{};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_publisher_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gpsfix_publisher_;
  rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_subscription_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> fatal_error_{false};
  int reconnect_attempt_count_{0};
  int connection_issue_count_{0};
  int configuration_issue_count_{0};
  std::string fatal_error_message_;
  std::thread worker_thread_;
};

}  // namespace amr_sweeper_gnss

#endif  // AMR_SWEEPER_GNSS__GNSS_NODE_HPP_
