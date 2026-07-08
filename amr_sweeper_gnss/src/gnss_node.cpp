#include "gnss_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <system_error>

namespace amr_sweeper_gnss
{

namespace
{

constexpr std::uint8_t kUbxSync1 = 0xB5;
constexpr std::uint8_t kUbxSync2 = 0x62;
constexpr std::uint8_t kClassAck = 0x05;
constexpr std::uint8_t kClassCfg = 0x06;
constexpr std::uint8_t kClassNav = 0x01;
constexpr std::uint8_t kMsgAckAck = 0x01;
constexpr std::uint8_t kMsgAckNak = 0x00;
constexpr std::uint8_t kMsgCfgValset = 0x8A;
constexpr std::uint8_t kMsgNavStatus = 0x03;
constexpr std::uint8_t kMsgNavPvt = 0x07;
constexpr std::uint8_t kMsgNavHpPosLlh = 0x14;
constexpr std::uint8_t kMsgNavCov = 0x36;

constexpr std::uint32_t kCfgUsbInProtRtcm3x = 0x10770004;
constexpr std::uint32_t kCfgUsbOutProtUbx = 0x10780001;
constexpr std::uint32_t kCfgUsbOutProtNmea = 0x10780002;
constexpr std::uint32_t kCfgUsbOutProtRtcm3x = 0x10780004;
constexpr std::uint32_t kCfgNavSpgFixMode = 0x20110011;
constexpr std::uint32_t kCfgNavSpgIniFix3d = 0x10110013;
constexpr std::uint32_t kCfgNavSpgDynModel = 0x20110021;
constexpr std::uint32_t kCfgNavSpgDgnssMode = 0x20110061;
constexpr std::uint32_t kCfgRateMeas = 0x30210001;
constexpr std::uint32_t kCfgRateNav = 0x30210002;
constexpr std::uint32_t kCfgSignalGpsEna = 0x1031001FU;
constexpr std::uint32_t kCfgSignalSbasEna = 0x10310020U;
constexpr std::uint32_t kCfgSignalGalEna = 0x10310021U;
constexpr std::uint32_t kCfgSignalBdsEna = 0x10310022U;
constexpr std::uint32_t kCfgSignalQzssEna = 0x10310024U;
constexpr std::uint32_t kCfgSignalGloEna = 0x10310025U;
constexpr std::uint32_t kCfgMsgoutNavHpPosLlhUsb = 0x20910036;
constexpr std::uint32_t kCfgMsgoutNavPvtUsb = 0x20910009;
constexpr std::uint32_t kCfgMsgoutNavStatusUsb = 0x2091001D;
constexpr std::uint32_t kCfgMsgoutNavCovUsb = 0x20910086;

template<typename T>
void appendLittleEndian(std::vector<std::uint8_t> & buffer, T value)
{
  for (std::size_t index = 0; index < sizeof(T); ++index) {
    buffer.push_back(static_cast<std::uint8_t>((value >> (8U * index)) & 0xFFU));
  }
}

double clampPositive(double value, double fallback)
{
  return value > 0.0 ? value : fallback;
}

constexpr std::int64_t kGpsWeekMilliseconds = 7LL * 24LL * 60LL * 60LL * 1000LL;

int clampMinInt(int value, int fallback, int minimum)
{
  if (value < minimum) {
    return std::max(fallback, minimum);
  }
  return value;
}

}  // namespace

UbloxNode::UbloxNode(const rclcpp::NodeOptions & options)
: Node("gnss_node", options)
{
  loadParameters();

  navsat_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
    navsat_topic_, rclcpp::SystemDefaultsQoS());
  gpsfix_publisher_ = create_publisher<gps_msgs::msg::GPSFix>(
    gpsfix_topic_, rclcpp::SystemDefaultsQoS());
  rtcm_subscription_ = create_subscription<rtcm_msgs::msg::Message>(
    rtcm_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&UbloxNode::onRtcmMessage, this, std::placeholders::_1));

  worker_thread_ = std::thread(&UbloxNode::run, this);
}

UbloxNode::~UbloxNode()
{
  stop_requested_.store(true);
  closeDevice();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void UbloxNode::loadParameters()
{
  const std::string device = declare_parameter("device", std::string{"/dev/gnss_usb"});
  device_path_ = declare_parameter("device_path", device);
  const int official_uart_baudrate = declare_parameter("uart1.baudrate", 115200);
  baud_rate_ = declare_parameter("baud_rate", official_uart_baudrate);
  frame_id_ = declare_parameter("frame_id", std::string{"gnss_link"});
  navsat_topic_ = declare_parameter("navsat_topic", std::string{"navsat"});
  gpsfix_topic_ = declare_parameter("gpsfix_topic", std::string{"fix"});
  rtcm_topic_ = declare_parameter("rtcm_topic", std::string{"ntrip_client/rtcm"});
  reconnect_delay_seconds_ = clampPositive(declare_parameter("reconnect_delay_seconds", 2.0), 2.0);
  publish_timeout_seconds_ = clampPositive(declare_parameter("publish_timeout_seconds", 1.0), 1.0);
  configure_on_connect_ = declare_parameter("configure_on_connect", true);
  poll_interval_seconds_ = clampPositive(declare_parameter("poll_interval_seconds", 1.0), 1.0);
  retry_attempts_before_error_ = declare_parameter("retry_attempts_before_error", 3);
  fatal_after_consecutive_errors_ = declare_parameter("fatal_after_consecutive_errors", 10);
  max_reconnect_attempts_ = declare_parameter("max_reconnect_attempts", 10);

  usb_in_rtcm3x_ = declare_parameter("usb_in_rtcm3x", true);
  usb_out_ubx_ = declare_parameter("usb_out_ubx", true);
  usb_out_nmea_ = declare_parameter("usb_out_nmea", false);
  usb_out_rtcm3x_ = declare_parameter("usb_out_rtcm3x", false);
  const double rate_hz = clampPositive(declare_parameter("rate", 5.0), 5.0);
  const int measurement_rate_ms_from_rate = std::max(50, static_cast<int>(std::lround(1000.0 / rate_hz)));
  measurement_rate_ms_ = clampMinInt(
    declare_parameter("measurement_rate_ms", measurement_rate_ms_from_rate),
    measurement_rate_ms_from_rate,
    50);
  const int official_nav_rate = declare_parameter("nav_rate", 1);
  navigation_rate_cycles_ = clampMinInt(
    declare_parameter("navigation_rate_cycles", official_nav_rate),
    official_nav_rate,
    1);
  fix_mode_name_ = declare_parameter("fix_mode_name", std::string{""});
  fix_mode_ = declare_parameter("fix_mode", 2);
  require_initial_3d_fix_ = declare_parameter("require_initial_3d_fix", true);
  dynamic_model_name_ = declare_parameter("dynamic_model_name", std::string{"automotive"});
  dynamic_model_ = declare_parameter("dynamic_model", 4);
  nav_hpposllh_rate_ = declare_parameter("nav_hpposllh_rate", 1);
  nav_pvt_rate_ = declare_parameter("nav_pvt_rate", 1);
  nav_status_rate_ = declare_parameter("nav_status_rate", 5);
  nav_cov_rate_ = declare_parameter("nav_cov_rate", 1);
  constellations_.gps = declare_parameter("gnss.gps", true);
  constellations_.sbas = declare_parameter("gnss.sbas", true);
  constellations_.galileo = declare_parameter("gnss.galileo", true);
  constellations_.beidou = declare_parameter("gnss.beidou", true);
  constellations_.qzss = declare_parameter("gnss.qzss", false);
  constellations_.glonass = declare_parameter("gnss.glonass", true);

  min_fix_type_ = declare_parameter("min_fix_type", 3);
  min_horizontal_stddev_m_ = declare_parameter("min_horizontal_stddev_m", 1.5);
  min_vertical_stddev_m_ = declare_parameter("min_vertical_stddev_m", 3.0);
  horizontal_covariance_scale_ = declare_parameter("horizontal_covariance_scale", 4.0);
  vertical_covariance_scale_ = declare_parameter("vertical_covariance_scale", 4.0);
  use_hacc_vacc_covariance_floor_ = declare_parameter("use_hacc_vacc_covariance_floor", true);

  if (retry_attempts_before_error_ < 1) {
    retry_attempts_before_error_ = 1;
  }
  if (fatal_after_consecutive_errors_ < 1) {
    fatal_after_consecutive_errors_ = 1;
  }
  if (max_reconnect_attempts_ < 0) {
    max_reconnect_attempts_ = 0;
  }

  dynamic_model_ = dynamicModelIdFromName(dynamic_model_name_, dynamic_model_);
  fix_mode_ = fixModeIdFromName(fix_mode_name_, fix_mode_);
  nav_hpposllh_rate_ = clampMinInt(nav_hpposllh_rate_, 1, 0);
  nav_pvt_rate_ = clampMinInt(nav_pvt_rate_, 1, 0);
  nav_status_rate_ = clampMinInt(nav_status_rate_, 1, 0);
  nav_cov_rate_ = clampMinInt(nav_cov_rate_, 1, 0);
  logReceiverConfigurationSummary();
}

void UbloxNode::onRtcmMessage(const rtcm_msgs::msg::Message::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (device_fd_ < 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "GNSS device is not connected; dropping RTCM packet");
    return;
  }

  if (!writeRaw(msg->message.data(), msg->message.size())) {
    RCLCPP_WARN(get_logger(), "Failed to write RTCM correction packet to receiver");
  }
}

void UbloxNode::run()
{
  while (!stop_requested_.load() && !fatal_error_.load()) {
    if (!openDevice()) {
      reportConnectionIssue("Unable to open GNSS device '" + device_path_ + "'");
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_delay_seconds_));
      continue;
    }

    if (configure_on_connect_ && !configureReceiver()) {
      reportConfigurationIssue("Receiver configuration reported an issue; continuing with live stream");
    } else {
      resetIssueCounters();
    }
    requestEssentialPolls();
    last_poll_request_time_ = std::chrono::steady_clock::now();

    readFromDevice();
    closeDevice();

    if (!stop_requested_.load() && !fatal_error_.load()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_delay_seconds_));
    }
  }
}

void UbloxNode::reportConnectionIssue(const std::string & message)
{
  ++connection_issue_count_;
  ++reconnect_attempt_count_;

  if (max_reconnect_attempts_ > 0 && reconnect_attempt_count_ >= max_reconnect_attempts_) {
    enterFatalState(
      message + ". Reached reconnect limit after " + std::to_string(reconnect_attempt_count_) +
      " attempts");
    return;
  }

  logEscalatingIssue(connection_issue_count_, message, "connection");
}

void UbloxNode::enterFatalState(const std::string & message)
{
  fatal_error_message_ = message;
  RCLCPP_FATAL(get_logger(), "%s", fatal_error_message_.c_str());
  fatal_error_.store(true);
  stop_requested_.store(true);
  rclcpp::shutdown();
}

void UbloxNode::reportConfigurationIssue(const std::string & message)
{
  ++configuration_issue_count_;
  logEscalatingIssue(configuration_issue_count_, message, "configuration");
}

void UbloxNode::logEscalatingIssue(int count, const std::string & message, const std::string & issue_type)
{
  if (count < retry_attempts_before_error_) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return;
  }

  if (count < fatal_after_consecutive_errors_) {
    if (count == retry_attempts_before_error_) {
      RCLCPP_ERROR(
        get_logger(), "%s. Escalating after %d consecutive failures", message.c_str(), count);
      return;
    }

    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return;
  }

  enterFatalState(
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive " + issue_type + " failures");
}

void UbloxNode::resetIssueCounters()
{
  reconnect_attempt_count_ = 0;
  connection_issue_count_ = 0;
  configuration_issue_count_ = 0;
  fatal_error_.store(false);
  fatal_error_message_.clear();
}

bool UbloxNode::openDevice()
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (device_fd_ >= 0) {
    return true;
  }

  const int fd = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    return false;
  }

  termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed for '%s': %s", device_path_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  const speed_t baud = toTermiosBaud(baud_rate_);
  if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
    RCLCPP_ERROR(get_logger(), "Unsupported GNSS baud rate %d", baud_rate_);
    close(fd);
    return false;
  }

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed for '%s': %s", device_path_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  tcflush(fd, TCIOFLUSH);
  device_fd_ = fd;
  parser_buffer_.clear();

  RCLCPP_INFO(get_logger(), "Connected to GNSS receiver on %s", device_path_.c_str());
  return true;
}

void UbloxNode::closeDevice()
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (device_fd_ >= 0) {
    close(device_fd_);
    device_fd_ = -1;
  }
}

bool UbloxNode::configureReceiver()
{
  std::vector<ConfigItem> items = {
    {"CFG_USBINPROT_RTCM3X", kCfgUsbInProtRtcm3x, ConfigValueType::Bool, usb_in_rtcm3x_ ? 1U : 0U},
    {"CFG_USBOUTPROT_UBX", kCfgUsbOutProtUbx, ConfigValueType::Bool, usb_out_ubx_ ? 1U : 0U},
    {"CFG_USBOUTPROT_NMEA", kCfgUsbOutProtNmea, ConfigValueType::Bool, usb_out_nmea_ ? 1U : 0U},
    {"CFG_USBOUTPROT_RTCM3X", kCfgUsbOutProtRtcm3x, ConfigValueType::Bool, usb_out_rtcm3x_ ? 1U : 0U},
    {"CFG_RATE_MEAS", kCfgRateMeas, ConfigValueType::U2, static_cast<std::uint32_t>(measurement_rate_ms_)},
    {"CFG_RATE_NAV", kCfgRateNav, ConfigValueType::U2, static_cast<std::uint32_t>(navigation_rate_cycles_)},
    {"CFG_NAVSPG_FIXMODE", kCfgNavSpgFixMode, ConfigValueType::U1, static_cast<std::uint32_t>(fix_mode_)},
    {"CFG_NAVSPG_INIFIX3D", kCfgNavSpgIniFix3d, ConfigValueType::Bool, require_initial_3d_fix_ ? 1U : 0U},
    {"CFG_NAVSPG_DYNMODEL", kCfgNavSpgDynModel, ConfigValueType::U1, static_cast<std::uint32_t>(dynamic_model_)},
    {"CFG_SIGNAL_GPS_ENA", kCfgSignalGpsEna, ConfigValueType::Bool, constellations_.gps ? 1U : 0U},
    {"CFG_SIGNAL_SBAS_ENA", kCfgSignalSbasEna, ConfigValueType::Bool, constellations_.sbas ? 1U : 0U},
    {"CFG_SIGNAL_GAL_ENA", kCfgSignalGalEna, ConfigValueType::Bool, constellations_.galileo ? 1U : 0U},
    {"CFG_SIGNAL_BDS_ENA", kCfgSignalBdsEna, ConfigValueType::Bool, constellations_.beidou ? 1U : 0U},
    {"CFG_SIGNAL_QZSS_ENA", kCfgSignalQzssEna, ConfigValueType::Bool, constellations_.qzss ? 1U : 0U},
    {"CFG_SIGNAL_GLO_ENA", kCfgSignalGloEna, ConfigValueType::Bool, constellations_.glonass ? 1U : 0U},
    {"CFG_MSGOUT_NAV_HPPOSLLH_USB", kCfgMsgoutNavHpPosLlhUsb, ConfigValueType::U1, static_cast<std::uint32_t>(nav_hpposllh_rate_)},
    {"CFG_MSGOUT_NAV_PVT_USB", kCfgMsgoutNavPvtUsb, ConfigValueType::U1, static_cast<std::uint32_t>(nav_pvt_rate_)},
    {"CFG_MSGOUT_NAV_STATUS_USB", kCfgMsgoutNavStatusUsb, ConfigValueType::U1, static_cast<std::uint32_t>(nav_status_rate_)},
    {"CFG_MSGOUT_NAV_COV_USB", kCfgMsgoutNavCovUsb, ConfigValueType::U1, static_cast<std::uint32_t>(nav_cov_rate_)},
  };

  bool all_sent = true;
  for (const auto & item : items) {
    if (rejected_config_keys_.count(item.key) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        10000,
        "Skipping previously rejected receiver config item %s (0x%08x)",
        item.name,
        item.key);
      continue;
    }
    all_sent = sendConfigItem(item) && all_sent;
  }

  return all_sent;
}

void UbloxNode::requestEssentialPolls()
{
  (void)sendFrame(kClassNav, kMsgNavHpPosLlh, {});
  (void)sendFrame(kClassNav, kMsgNavStatus, {});
  (void)sendFrame(kClassNav, kMsgNavCov, {});
}

bool UbloxNode::sendConfigBatch(const std::vector<ConfigItem> & items)
{
  std::vector<std::uint8_t> payload;
  payload.reserve(4U + items.size() * 6U);

  payload.push_back(0x00);  // version
  payload.push_back(0x01);  // RAM layer
  payload.push_back(0x00);
  payload.push_back(0x00);

  for (const auto & item : items) {
    appendLittleEndian<std::uint32_t>(payload, item.key);
    switch (item.type) {
      case ConfigValueType::Bool:
      case ConfigValueType::U1:
        payload.push_back(static_cast<std::uint8_t>(item.value & 0xFFU));
        break;
      case ConfigValueType::U2:
        appendLittleEndian<std::uint16_t>(payload, static_cast<std::uint16_t>(item.value & 0xFFFFU));
        break;
    }
  }

  return sendFrame(kClassCfg, kMsgCfgValset, payload);
}

bool UbloxNode::sendConfigItem(const ConfigItem & item)
{
  pending_config_acks_.push_back(item);
  const bool sent = sendConfigBatch({item});
  if (!sent) {
    pending_config_acks_.pop_back();
  }
  return sent;
}

bool UbloxNode::sendFrame(
  std::uint8_t msg_class,
  std::uint8_t msg_id,
  const std::vector<std::uint8_t> & payload)
{
  std::vector<std::uint8_t> frame;
  frame.reserve(payload.size() + 8U);
  frame.push_back(kUbxSync1);
  frame.push_back(kUbxSync2);
  frame.push_back(msg_class);
  frame.push_back(msg_id);
  appendLittleEndian<std::uint16_t>(frame, static_cast<std::uint16_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  const std::uint16_t checksum = computeChecksumA(msg_class, msg_id, payload);
  frame.push_back(static_cast<std::uint8_t>(checksum & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>((checksum >> 8U) & 0xFFU));

  std::lock_guard<std::mutex> lock(device_mutex_);
  if (device_fd_ < 0) {
    return false;
  }
  return writeRaw(frame.data(), frame.size());
}

bool UbloxNode::writeRaw(const std::uint8_t * data, std::size_t size)
{
  std::size_t written = 0;
  while (written < size) {
    const ssize_t result = write(device_fd_, data + written, size - written);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(get_logger(), "GNSS write failed: %s", std::strerror(errno));
      return false;
    }
    written += static_cast<std::size_t>(result);
  }
  return true;
}

void UbloxNode::readFromDevice()
{
  std::array<std::uint8_t, 1024> buffer{};
  while (!stop_requested_.load()) {
    ssize_t bytes_read = -1;
    {
      std::lock_guard<std::mutex> lock(device_mutex_);
      if (device_fd_ < 0) {
        return;
      }
      bytes_read = read(device_fd_, buffer.data(), buffer.size());
    }

    if (bytes_read < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      reportConnectionIssue("GNSS read failed on '" + device_path_ + "': " + std::strerror(errno));
      return;
    }

    if (bytes_read == 0) {
      const auto now_steady = std::chrono::steady_clock::now();
      if (
        now_steady - last_poll_request_time_ >=
        std::chrono::duration<double>(poll_interval_seconds_))
      {
        requestEssentialPolls();
        last_poll_request_time_ = now_steady;
      }
      continue;
    }

    last_poll_request_time_ = std::chrono::steady_clock::now();
    parseIncomingBytes(buffer.data(), static_cast<std::size_t>(bytes_read));
  }
}

void UbloxNode::parseIncomingBytes(const std::uint8_t * data, std::size_t size)
{
  parser_buffer_.insert(parser_buffer_.end(), data, data + size);
  static constexpr std::array<std::uint8_t, 2> sync_bytes{{kUbxSync1, kUbxSync2}};

  while (parser_buffer_.size() >= 8U) {
    auto sync_it = std::search(
      parser_buffer_.begin(),
      parser_buffer_.end(),
      sync_bytes.begin(),
      sync_bytes.end());
    if (sync_it != parser_buffer_.begin()) {
      if (sync_it == parser_buffer_.end()) {
        parser_buffer_.clear();
        return;
      }
      parser_buffer_.erase(parser_buffer_.begin(), sync_it);
    }

    if (parser_buffer_.size() < 8U) {
      return;
    }

    const std::uint16_t payload_length = static_cast<std::uint16_t>(
      parser_buffer_[4] | (static_cast<std::uint16_t>(parser_buffer_[5]) << 8U));
    const std::size_t frame_length = 6U + payload_length + 2U;
    if (parser_buffer_.size() < frame_length) {
      return;
    }

    const std::uint8_t msg_class = parser_buffer_[2];
    const std::uint8_t msg_id = parser_buffer_[3];
    std::vector<std::uint8_t> payload(
      parser_buffer_.begin() + 6,
      parser_buffer_.begin() + 6 + payload_length);

    const std::uint16_t checksum = computeChecksumA(msg_class, msg_id, payload);
    if (
      parser_buffer_[frame_length - 2] != static_cast<std::uint8_t>(checksum & 0xFFU) ||
      parser_buffer_[frame_length - 1] != static_cast<std::uint8_t>((checksum >> 8U) & 0xFFU))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Discarding UBX frame with invalid checksum");
      parser_buffer_.erase(parser_buffer_.begin(), parser_buffer_.begin() + 2);
      continue;
    }

    processFrame(msg_class, msg_id, payload);
    parser_buffer_.erase(parser_buffer_.begin(), parser_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length));
  }
}

void UbloxNode::processFrame(
  std::uint8_t msg_class,
  std::uint8_t msg_id,
  const std::vector<std::uint8_t> & payload)
{
  if (msg_class == kClassAck && msg_id == kMsgAckAck) {
    handleAckAck(payload);
    return;
  }
  if (msg_class == kClassAck && msg_id == kMsgAckNak) {
    handleAckNak(payload);
    return;
  }
  if (msg_class == kClassNav && msg_id == kMsgNavHpPosLlh) {
    handleNavHpPosLlh(payload);
    return;
  }
  if (msg_class == kClassNav && msg_id == kMsgNavStatus) {
    handleNavStatus(payload);
    return;
  }
  if (msg_class == kClassNav && msg_id == kMsgNavPvt) {
    handleNavPvt(payload);
    return;
  }
  if (msg_class == kClassNav && msg_id == kMsgNavCov) {
    handleNavCov(payload);
  }
}

void UbloxNode::handleAckAck(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 2U) {
    return;
  }
  if (
    payload[0] == kClassCfg && payload[1] == kMsgCfgValset &&
    !pending_config_acks_.empty())
  {
    const ConfigItem item = pending_config_acks_.front();
    pending_config_acks_.pop_front();
    RCLCPP_DEBUG(
      get_logger(),
      "Receiver accepted config item %s (0x%08x)",
      item.name,
      item.key);
    return;
  }
  RCLCPP_DEBUG(
    get_logger(),
    "ubx class: 0x%02x id: 0x%02x ack ack payload - class: 0x%02x id: 0x%02x",
    kClassAck,
    kMsgAckAck,
    payload[0],
    payload[1]);
}

void UbloxNode::handleAckNak(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 2U) {
    return;
  }
  if (
    payload[0] == kClassCfg && payload[1] == kMsgCfgValset &&
    !pending_config_acks_.empty())
  {
    const ConfigItem item = pending_config_acks_.front();
    pending_config_acks_.pop_front();
    rejected_config_keys_.insert(item.key);
    RCLCPP_WARN(
      get_logger(),
      "Receiver rejected config item %s (0x%08x); it will be skipped on future reconnects",
      item.name,
      item.key);
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "ubx class: 0x%02x id: 0x%02x ack nak payload - class: 0x%02x id: 0x%02x",
    kClassAck,
    kMsgAckNak,
    payload[0],
    payload[1]);
}

void UbloxNode::handleNavHpPosLlh(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 36U) {
    return;
  }

  NavHpPosLlh fix;
  fix.invalid_flags = payload[3];
  fix.itow = readU32(payload, 4);
  const double lon_base = static_cast<double>(readI32(payload, 8)) * 1e-7;
  const double lat_base = static_cast<double>(readI32(payload, 12)) * 1e-7;
  const double lon_hp = static_cast<double>(static_cast<std::int8_t>(payload[24])) * 1e-9;
  const double lat_hp = static_cast<double>(static_cast<std::int8_t>(payload[25])) * 1e-9;
  fix.longitude_deg = lon_base + lon_hp;
  fix.latitude_deg = lat_base + lat_hp;
  fix.altitude_m =
    static_cast<double>(readI32(payload, 16)) * 1e-3 +
    static_cast<double>(static_cast<std::int8_t>(payload[26])) * 1e-4;
  fix.horizontal_accuracy_m = static_cast<double>(readU32(payload, 28)) * 1e-4;
  fix.vertical_accuracy_m = static_cast<double>(readU32(payload, 32)) * 1e-4;

  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    last_hpposllh_ = fix;
  }
  tryPublishNavSat();
}

void UbloxNode::handleNavStatus(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 16U) {
    return;
  }

  NavStatus status;
  status.itow = readU32(payload, 0);
  status.gps_fix = payload[4];
  status.flags = payload[5];
  status.fix_stat = payload[6];
  status.flags2 = payload[7];

  std::lock_guard<std::mutex> lock(nav_mutex_);
  last_status_ = status;
}

void UbloxNode::handleNavCov(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 64U) {
    return;
  }

  NavCov cov;
  cov.itow = readU32(payload, 0);
  cov.position_covariance_valid = payload[5] != 0U;
  cov.pos_cov_nn = static_cast<double>(readF32(payload, 16));
  cov.pos_cov_ne = static_cast<double>(readF32(payload, 20));
  cov.pos_cov_nd = static_cast<double>(readF32(payload, 24));
  cov.pos_cov_ee = static_cast<double>(readF32(payload, 28));
  cov.pos_cov_ed = static_cast<double>(readF32(payload, 32));
  cov.pos_cov_dd = static_cast<double>(readF32(payload, 36));

  std::lock_guard<std::mutex> lock(nav_mutex_);
  last_cov_ = cov;
}

void UbloxNode::handleNavPvt(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 92U) {
    return;
  }

  NavPvt pvt;
  pvt.itow = readU32(payload, 0);
  pvt.fix_type = payload[20];
  pvt.flags = payload[21];
  pvt.flags2 = payload[22];
  pvt.num_sv = payload[23];
  pvt.ground_speed_mps = static_cast<double>(readI32(payload, 60)) * 1e-3;
  pvt.vertical_speed_mps = -static_cast<double>(readI32(payload, 56)) * 1e-3;
  pvt.heading_deg = static_cast<double>(readI32(payload, 64)) * 1e-5;
  pvt.pdop = static_cast<double>(readU32(payload, 76) & 0xFFFFU) * 0.01;

  std::lock_guard<std::mutex> lock(nav_mutex_);
  last_pvt_ = pvt;
}

void UbloxNode::tryPublishNavSat()
{
  std::optional<NavHpPosLlh> hpposllh;
  std::optional<NavStatus> status;
  std::optional<NavCov> cov;
  std::optional<NavPvt> pvt;
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    hpposllh = last_hpposllh_;
    status = last_status_;
    cov = last_cov_;
    pvt = last_pvt_;
  }

  if (!hpposllh.has_value() || !status.has_value()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for both NAV-HPPOSLLH and NAV-STATUS before publishing navsat");
    return;
  }

  if ((hpposllh->invalid_flags & 0x03U) != 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping navsat publish because the receiver flagged the high-precision position as invalid (flags=0x%02x)",
      hpposllh->invalid_flags);
    return;
  }

  const bool gps_fix_ok = (status->flags & 0x01U) != 0U;
  const std::uint8_t carrier_solution = static_cast<std::uint8_t>((status->flags2 >> 6U) & 0x03U);
  const rclcpp::Time fix_stamp = resolveFixStamp(hpposllh->itow);

  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = fix_stamp;
  msg.header.frame_id = frame_id_;
  msg.latitude = hpposllh->latitude_deg;
  msg.longitude = hpposllh->longitude_deg;
  msg.altitude = hpposllh->altitude_m;

  if (!gps_fix_ok || status->gps_fix < 2U) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  } else if (carrier_solution == 2U) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  } else {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
  msg.status.service = navSatServiceMask();

  const double min_horizontal_stddev =
    std::max(0.0, min_horizontal_stddev_m_);
  const double min_vertical_stddev =
    std::max(0.0, min_vertical_stddev_m_);
  const double hacc_floor_stddev =
    use_hacc_vacc_covariance_floor_ ? std::max(0.0, hpposllh->horizontal_accuracy_m) : 0.0;
  const double vacc_floor_stddev =
    use_hacc_vacc_covariance_floor_ ? std::max(0.0, hpposllh->vertical_accuracy_m) : 0.0;
  const double horizontal_variance_floor =
    std::pow(std::max(min_horizontal_stddev, hacc_floor_stddev), 2.0);
  const double vertical_variance_floor =
    std::pow(std::max(min_vertical_stddev, vacc_floor_stddev), 2.0);
  const double horizontal_scale = std::max(1.0, horizontal_covariance_scale_);
  const double vertical_scale = std::max(1.0, vertical_covariance_scale_);

  if (cov.has_value() && cov->position_covariance_valid && cov->itow == hpposllh->itow) {
    // UBX NAV-COV is reported in NED. NavSatFix expects ENU covariance.
    const double scaled_cov_ne = cov->pos_cov_ne * horizontal_scale;
    const double scaled_cov_nd = cov->pos_cov_nd * std::sqrt(horizontal_scale * vertical_scale);
    const double scaled_cov_ed = cov->pos_cov_ed * std::sqrt(horizontal_scale * vertical_scale);

    msg.position_covariance[0] = std::max(cov->pos_cov_ee * horizontal_scale, horizontal_variance_floor);
    msg.position_covariance[1] = scaled_cov_ne;
    msg.position_covariance[2] = -scaled_cov_ed;
    msg.position_covariance[3] = scaled_cov_ne;
    msg.position_covariance[4] = std::max(cov->pos_cov_nn * horizontal_scale, horizontal_variance_floor);
    msg.position_covariance[5] = -scaled_cov_nd;
    msg.position_covariance[6] = -scaled_cov_ed;
    msg.position_covariance[7] = -scaled_cov_nd;
    msg.position_covariance[8] = std::max(cov->pos_cov_dd * vertical_scale, vertical_variance_floor);
    msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
  } else {
    const double base_horizontal_variance = std::pow(std::max(0.0, hpposllh->horizontal_accuracy_m), 2.0);
    const double base_vertical_variance = std::pow(std::max(0.0, hpposllh->vertical_accuracy_m), 2.0);
    msg.position_covariance[0] = std::max(base_horizontal_variance * horizontal_scale, horizontal_variance_floor);
    msg.position_covariance[4] = std::max(base_horizontal_variance * horizontal_scale, horizontal_variance_floor);
    msg.position_covariance[8] = std::max(base_vertical_variance * vertical_scale, vertical_variance_floor);
    msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }

  navsat_publisher_->publish(msg);

  if (pvt.has_value() && pvt->itow == hpposllh->itow) {
    gps_msgs::msg::GPSFix gps_msg;
    gps_msg.header = msg.header;
    gps_msg.latitude = msg.latitude;
    gps_msg.longitude = msg.longitude;
    gps_msg.altitude = msg.altitude;
    gps_msg.position_covariance = msg.position_covariance;
    gps_msg.position_covariance_type = msg.position_covariance_type;
    gps_msg.speed = pvt->ground_speed_mps;
    gps_msg.climb = pvt->vertical_speed_mps;
    gps_msg.track = pvt->heading_deg;
    gps_msg.pdop = pvt->pdop;
    gps_msg.hdop = -1.0;
    gps_msg.vdop = -1.0;
    gps_msg.err_horz = hpposllh->horizontal_accuracy_m;
    gps_msg.err_vert = hpposllh->vertical_accuracy_m;

    gps_msg.status.header = gps_msg.header;
    gps_msg.status.satellites_used = pvt->num_sv;
    gps_msg.status.position_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;
    gps_msg.status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_DOPPLER;
    gps_msg.status.orientation_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;

    const bool differential_solution = (status->flags & 0x02U) != 0U;
    if (!gps_fix_ok || status->gps_fix < 2U) {
      gps_msg.status.status = gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
    } else if (carrier_solution == 2U) {
      gps_msg.status.status = gps_msgs::msg::GPSStatus::STATUS_RTK_FIX;
    } else if (carrier_solution == 1U) {
      gps_msg.status.status = gps_msgs::msg::GPSStatus::STATUS_RTK_FLOAT;
    } else if (differential_solution) {
      gps_msg.status.status = gps_msgs::msg::GPSStatus::STATUS_DGPS_FIX;
    } else {
      gps_msg.status.status = gps_msgs::msg::GPSStatus::STATUS_FIX;
    }

    gpsfix_publisher_->publish(gps_msg);
  }
}

rclcpp::Time UbloxNode::resolveFixStamp(std::uint32_t itow_ms)
{
  const rclcpp::Time receipt_stamp = now();
  if (!last_fix_itow_ms_.has_value() || last_fix_measurement_stamp_.nanoseconds() == 0) {
    last_fix_itow_ms_ = itow_ms;
    last_fix_measurement_stamp_ = receipt_stamp;
    return receipt_stamp;
  }

  std::int64_t delta_ms = static_cast<std::int64_t>(itow_ms) -
    static_cast<std::int64_t>(*last_fix_itow_ms_);
  if (delta_ms < 0) {
    delta_ms += kGpsWeekMilliseconds;
  }
  if (delta_ms < 0 || delta_ms > 10000) {
    delta_ms = 0;
  }

  rclcpp::Time stamp = last_fix_measurement_stamp_ +
    rclcpp::Duration::from_nanoseconds(delta_ms * 1000000LL);
  if (stamp > receipt_stamp) {
    stamp = receipt_stamp;
  }
  if (stamp <= last_fix_measurement_stamp_) {
    stamp = last_fix_measurement_stamp_ + rclcpp::Duration::from_nanoseconds(1);
  }

  last_fix_itow_ms_ = itow_ms;
  last_fix_measurement_stamp_ = stamp;
  return stamp;
}

std::uint16_t UbloxNode::computeChecksumA(
  std::uint8_t msg_class,
  std::uint8_t msg_id,
  const std::vector<std::uint8_t> & payload)
{
  std::uint8_t ck_a = 0U;
  std::uint8_t ck_b = 0U;

  auto add = [&ck_a, &ck_b](std::uint8_t byte) {
    ck_a = static_cast<std::uint8_t>(ck_a + byte);
    ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
  };

  add(msg_class);
  add(msg_id);
  add(static_cast<std::uint8_t>(payload.size() & 0xFFU));
  add(static_cast<std::uint8_t>((payload.size() >> 8U) & 0xFFU));
  for (std::uint8_t byte : payload) {
    add(byte);
  }

  return static_cast<std::uint16_t>(ck_a | (static_cast<std::uint16_t>(ck_b) << 8U));
}

std::uint32_t UbloxNode::readU32(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint32_t>(data[offset]) |
         (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
}

std::int32_t UbloxNode::readI32(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::int32_t>(readU32(data, offset));
}

float UbloxNode::readF32(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  const auto bits = readU32(data, offset);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(float));
  return value;
}

speed_t UbloxNode::toTermiosBaud(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:
      throw std::runtime_error("Unsupported baud rate");
  }
}

int UbloxNode::dynamicModelIdFromName(const std::string & name, int fallback)
{
  if (name.empty()) {
    return fallback;
  }

  std::string lowered = name;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });

  if (lowered == "portable") return 0;
  if (lowered == "stationary") return 2;
  if (lowered == "pedestrian") return 3;
  if (lowered == "automotive" || lowered == "ground_vehicle") return 4;
  if (lowered == "sea") return 5;
  if (lowered == "airborne1" || lowered == "airborne_1g") return 6;
  if (lowered == "airborne2" || lowered == "airborne_2g") return 7;
  if (lowered == "airborne4" || lowered == "airborne_4g") return 8;
  if (lowered == "wristwatch") return 9;
  if (lowered == "bike" || lowered == "bicycle") return 10;
  return fallback;
}

int UbloxNode::fixModeIdFromName(const std::string & name, int fallback)
{
  if (name.empty()) {
    return fallback;
  }

  std::string lowered = name;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });

  if (lowered == "2d" || lowered == "2d_only") return 1;
  if (lowered == "3d" || lowered == "3d_only") return 2;
  if (lowered == "auto" || lowered == "both") return 3;
  return fallback;
}

std::uint16_t UbloxNode::navSatServiceMask() const
{
  std::uint16_t mask = 0U;
  if (constellations_.gps) {
    mask |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
  if (constellations_.glonass) {
    mask |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
  }
  if (constellations_.galileo) {
    mask |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
  }
  if (constellations_.beidou) {
    mask |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  }
  return mask != 0U ? mask : sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
}

void UbloxNode::logReceiverConfigurationSummary() const
{
  RCLCPP_INFO(
    get_logger(),
    "GNSS receiver config: device=%s baud=%d rate=%.2fHz nav_rate=%d dyn_model=%d fix_mode=%d "
    "constellations[gps=%s sbas=%s galileo=%s beidou=%s qzss=%s glonass=%s]",
    device_path_.c_str(),
    baud_rate_,
    1000.0 / static_cast<double>(measurement_rate_ms_),
    navigation_rate_cycles_,
    dynamic_model_,
    fix_mode_,
    constellations_.gps ? "on" : "off",
    constellations_.sbas ? "on" : "off",
    constellations_.galileo ? "on" : "off",
    constellations_.beidou ? "on" : "off",
    constellations_.qzss ? "on" : "off",
    constellations_.glonass ? "on" : "off");
}

}  // namespace amr_sweeper_gnss

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_gnss::UbloxNode>());
  rclcpp::shutdown();
  return 0;
}
