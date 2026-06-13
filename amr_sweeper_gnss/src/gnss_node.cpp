#include "gnss_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
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
constexpr std::uint8_t kMsgNavHpPosLlh = 0x14;
constexpr std::uint8_t kMsgNavCov = 0x36;

constexpr std::uint32_t kCfgUsbInProtRtcm3x = 0x10770004;
constexpr std::uint32_t kCfgUsbOutProtUbx = 0x10780001;
constexpr std::uint32_t kCfgUsbOutProtNmea = 0x10780002;
constexpr std::uint32_t kCfgUsbOutProtRtcm3x = 0x10780004;
constexpr std::uint32_t kCfgNavSpgFixMode = 0x20110011;
constexpr std::uint32_t kCfgNavSpgIniFix3d = 0x10110013;
constexpr std::uint32_t kCfgNavSpgDynModel = 0x20110021;
constexpr std::uint32_t kCfgRateMeas = 0x30210001;
constexpr std::uint32_t kCfgRateNav = 0x30210002;
constexpr std::uint32_t kCfgMsgoutNavHpPosLlhUsb = 0x20910036;
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

}  // namespace

UbloxNode::UbloxNode(const rclcpp::NodeOptions & options)
: Node("gnss_node", options)
{
  loadParameters();

  navsat_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
    navsat_topic_, rclcpp::SystemDefaultsQoS());
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
  device_path_ = declare_parameter("device_path", std::string{"/dev/gnss_usb"});
  baud_rate_ = declare_parameter("baud_rate", 115200);
  frame_id_ = declare_parameter("frame_id", std::string{"gnss_link"});
  navsat_topic_ = declare_parameter("navsat_topic", std::string{"navsat"});
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
  measurement_rate_ms_ = declare_parameter("measurement_rate_ms", 200);
  navigation_rate_cycles_ = declare_parameter("navigation_rate_cycles", 1);
  fix_mode_ = declare_parameter("fix_mode", 2);
  require_initial_3d_fix_ = declare_parameter("require_initial_3d_fix", true);
  dynamic_model_ = declare_parameter("dynamic_model", 4);
  nav_hpposllh_rate_ = declare_parameter("nav_hpposllh_rate", 1);
  nav_status_rate_ = declare_parameter("nav_status_rate", 5);
  nav_cov_rate_ = declare_parameter("nav_cov_rate", 1);

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
    {kCfgUsbInProtRtcm3x, ConfigValueType::Bool, usb_in_rtcm3x_ ? 1U : 0U},
    {kCfgUsbOutProtUbx, ConfigValueType::Bool, usb_out_ubx_ ? 1U : 0U},
    {kCfgUsbOutProtNmea, ConfigValueType::Bool, usb_out_nmea_ ? 1U : 0U},
    {kCfgUsbOutProtRtcm3x, ConfigValueType::Bool, usb_out_rtcm3x_ ? 1U : 0U},
    {kCfgRateMeas, ConfigValueType::U2, static_cast<std::uint32_t>(measurement_rate_ms_)},
    {kCfgRateNav, ConfigValueType::U2, static_cast<std::uint32_t>(navigation_rate_cycles_)},
    {kCfgNavSpgFixMode, ConfigValueType::U1, static_cast<std::uint32_t>(fix_mode_)},
    {kCfgNavSpgIniFix3d, ConfigValueType::Bool, require_initial_3d_fix_ ? 1U : 0U},
    {kCfgNavSpgDynModel, ConfigValueType::U1, static_cast<std::uint32_t>(dynamic_model_)},
    {kCfgMsgoutNavHpPosLlhUsb, ConfigValueType::U1, static_cast<std::uint32_t>(nav_hpposllh_rate_)},
    {kCfgMsgoutNavStatusUsb, ConfigValueType::U1, static_cast<std::uint32_t>(nav_status_rate_)},
    {kCfgMsgoutNavCovUsb, ConfigValueType::U1, static_cast<std::uint32_t>(nav_cov_rate_)},
  };

  return sendConfigBatch(items);
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
  if (msg_class == kClassNav && msg_id == kMsgNavCov) {
    handleNavCov(payload);
  }
}

void UbloxNode::handleAckAck(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 2U) {
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

void UbloxNode::tryPublishNavSat()
{
  std::optional<NavHpPosLlh> hpposllh;
  std::optional<NavStatus> status;
  std::optional<NavCov> cov;

  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    hpposllh = last_hpposllh_;
    status = last_status_;
    cov = last_cov_;
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
  const bool diff_solution = (status->flags & 0x02U) != 0U;
  const std::uint8_t carrier_solution = static_cast<std::uint8_t>((status->flags2 >> 6U) & 0x03U);

  int fix_quality = 0;
  if (gps_fix_ok && status->gps_fix >= 3U) {
    fix_quality = 1;
    if (diff_solution) {
      fix_quality = 2;
    }
    if (carrier_solution == 1U) {
      fix_quality = 3;
    } else if (carrier_solution == 2U) {
      fix_quality = 4;
    }
  }

  if (fix_quality < min_fix_type_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping navsat publish because fix quality %d is below the configured minimum %d",
      fix_quality, min_fix_type_);
    return;
  }
  if (hpposllh->horizontal_accuracy_m > min_horizontal_stddev_m_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping navsat publish because horizontal accuracy %.3fm exceeds the %.3fm limit",
      hpposllh->horizontal_accuracy_m, min_horizontal_stddev_m_);
    return;
  }
  if (hpposllh->vertical_accuracy_m > min_vertical_stddev_m_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping navsat publish because vertical accuracy %.3fm exceeds the %.3fm limit",
      hpposllh->vertical_accuracy_m, min_vertical_stddev_m_);
    return;
  }

  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.latitude = hpposllh->latitude_deg;
  msg.longitude = hpposllh->longitude_deg;
  msg.altitude = hpposllh->altitude_m;

  if (fix_quality >= 2) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  } else {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  double cov_nn = std::pow(hpposllh->horizontal_accuracy_m, 2.0);
  double cov_ee = cov_nn;
  double cov_dd = std::pow(hpposllh->vertical_accuracy_m, 2.0);
  double cov_ne = 0.0;
  double cov_nd = 0.0;
  double cov_ed = 0.0;

  if (
    cov.has_value() &&
    cov->position_covariance_valid &&
    cov->itow == hpposllh->itow)
  {
    cov_nn = cov->pos_cov_nn;
    cov_ne = cov->pos_cov_ne;
    cov_nd = cov->pos_cov_nd;
    cov_ee = cov->pos_cov_ee;
    cov_ed = cov->pos_cov_ed;
    cov_dd = cov->pos_cov_dd;
  }

  if (use_hacc_vacc_covariance_floor_) {
    cov_nn = std::max(cov_nn, std::pow(hpposllh->horizontal_accuracy_m, 2.0));
    cov_ee = std::max(cov_ee, std::pow(hpposllh->horizontal_accuracy_m, 2.0));
    cov_dd = std::max(cov_dd, std::pow(hpposllh->vertical_accuracy_m, 2.0));
  }

  msg.position_covariance[0] = cov_nn * horizontal_covariance_scale_;
  msg.position_covariance[1] = cov_ne * horizontal_covariance_scale_;
  msg.position_covariance[2] = cov_nd * vertical_covariance_scale_;
  msg.position_covariance[3] = cov_ne * horizontal_covariance_scale_;
  msg.position_covariance[4] = cov_ee * horizontal_covariance_scale_;
  msg.position_covariance[5] = cov_ed * vertical_covariance_scale_;
  msg.position_covariance[6] = cov_nd * vertical_covariance_scale_;
  msg.position_covariance[7] = cov_ed * vertical_covariance_scale_;
  msg.position_covariance[8] = cov_dd * vertical_covariance_scale_;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;

  navsat_publisher_->publish(msg);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Published navsat fix: lat=%.8f lon=%.8f alt=%.3f quality=%d",
    msg.latitude, msg.longitude, msg.altitude, fix_quality);
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

}  // namespace amr_sweeper_gnss

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_gnss::UbloxNode>());
  rclcpp::shutdown();
  return 0;
}
