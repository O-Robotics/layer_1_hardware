#include "ntrip_client.hpp"

#include <openssl/err.h>
#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace amr_sweeper_gnss
{

namespace
{

std::optional<YAML::Node> load_ros_parameters(const std::string & file_path)
{
  YAML::Node root = YAML::LoadFile(file_path);
  if (!root.IsMap()) {
    return std::nullopt;
  }

  for (const auto & node_entry : root) {
    if (!node_entry.second.IsMap()) {
      continue;
    }
    YAML::Node parameters = node_entry.second["ros__parameters"];
    if (parameters && parameters.IsMap()) {
      return parameters;
    }
  }

  return std::nullopt;
}

template<typename T>
T read_startup_parameter_from_params(
  int argc,
  char ** argv,
  const std::string & parameter_name,
  const T & default_value)
{
  T value = default_value;

  for (int index = 1; index < argc - 1; ++index) {
    if (std::string(argv[index]) != "--params-file") {
      continue;
    }

    auto parameters = load_ros_parameters(argv[index + 1]);
    if (!parameters.has_value()) {
      continue;
    }

    YAML::Node parameter_node = (*parameters)[parameter_name];
    if (parameter_node) {
      value = parameter_node.as<T>();
    }
  }

  return value;
}

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

}  // namespace

NtripClientNode::NtripClientNode()
: Node("ntrip_client")
{
  declare_parameter<std::string>("host", "127.0.0.1");
  declare_parameter<int>("port", 2101);
  declare_parameter<std::string>("mountpoint", "mount");
  declare_parameter<std::string>("alternate_mountpoint", "");
  declare_parameter<std::string>("ntrip_version", "Ntrip/2.0");
  declare_parameter<bool>("authenticate", false);
  declare_parameter<std::string>("username", "");
  declare_parameter<std::string>("password", "");
  declare_parameter<bool>("ssl", false);
  declare_parameter<std::string>("cert", "None");
  declare_parameter<std::string>("key", "None");
  declare_parameter<std::string>("ca_cert", "None");
  declare_parameter<std::string>("rtcm_frame_id", "gnss_link");
  declare_parameter<double>("startup_retry_seconds", 5.0);
  declare_parameter<double>("failed_connection_retry_seconds", 5.0);
  declare_parameter<double>("reconnect_attempt_wait_seconds", 5.0);
  declare_parameter<double>("socket_timeout_seconds", 10.0);
  declare_parameter<double>("rtcm_timeout_seconds", 4.0);
  declare_parameter<int>("retry_attempts_before_error", 3);
  declare_parameter<int>("fatal_after_consecutive_errors", 10);
  declare_parameter<bool>("send_nmea", false);

  host_ = get_parameter("host").as_string();
  port_ = get_parameter("port").as_int();
  mountpoint_ = get_parameter("mountpoint").as_string();
  alternate_mountpoint_ = get_parameter("alternate_mountpoint").as_string();
  ntrip_version_ = get_parameter("ntrip_version").as_string();
  authenticate_ = get_parameter("authenticate").as_bool();
  username_ = get_parameter("username").as_string();
  password_ = get_parameter("password").as_string();
  use_ssl_ = get_parameter("ssl").as_bool();
  cert_ = get_parameter("cert").as_string();
  key_ = get_parameter("key").as_string();
  ca_cert_ = get_parameter("ca_cert").as_string();
  rtcm_frame_id_ = get_parameter("rtcm_frame_id").as_string();
  startup_retry_seconds_ = get_parameter("startup_retry_seconds").as_double();
  reconnect_wait_ = get_parameter("reconnect_attempt_wait_seconds").as_double();
  failed_connection_retry_ = get_parameter("failed_connection_retry_seconds").as_double();
  socket_timeout_seconds_ = get_parameter("socket_timeout_seconds").as_double();
  rtcm_timeout_seconds_ = get_parameter("rtcm_timeout_seconds").as_double();
  retry_attempts_before_error_ = get_parameter("retry_attempts_before_error").as_int();
  fatal_after_consecutive_errors_ = get_parameter("fatal_after_consecutive_errors").as_int();
  send_nmea_ = get_parameter("send_nmea").as_bool();

  if (cert_ == "None") {
    cert_.clear();
  }
  if (key_ == "None") {
    key_.clear();
  }
  if (ca_cert_ == "None") {
    ca_cert_.clear();
  }
  if (authenticate_ && (username_.empty() || password_.empty())) {
    throw std::runtime_error("authenticate is true, but username or password is empty");
  }

  mountpoints_.push_back(mountpoint_);
  if (!alternate_mountpoint_.empty() && alternate_mountpoint_ != mountpoint_) {
    mountpoints_.push_back(alternate_mountpoint_);
  }

  if (failed_connection_retry_ <= 0.0) {
    failed_connection_retry_ = reconnect_wait_;
  }
  if (startup_retry_seconds_ <= 0.0) {
    startup_retry_seconds_ = failed_connection_retry_;
  }
  if (retry_attempts_before_error_ < 1) {
    retry_attempts_before_error_ = 1;
  }
  if (fatal_after_consecutive_errors_ < 1) {
    fatal_after_consecutive_errors_ = 1;
  }

  publisher_ = create_publisher<rtcm_msgs::msg::Message>("rtcm", 10);

  if (send_nmea_) {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    fix_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix",
      qos,
      std::bind(&NtripClientNode::handle_fix, this, std::placeholders::_1));
  }

  worker_thread_ = std::thread(&NtripClientNode::run, this);
}

NtripClientNode::~NtripClientNode()
{
  stop_requested_.store(true);
  close_socket();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool NtripClientNode::has_fatal_error() const
{
  return fatal_error_.load();
}

int NtripClientNode::fatal_after_consecutive_errors() const
{
  return fatal_after_consecutive_errors_;
}

double NtripClientNode::startup_retry_seconds() const
{
  return startup_retry_seconds_;
}

void NtripClientNode::run()
{
  while (!stop_requested_.load()) {
    try {
      connect_and_stream();
    } catch (const std::exception & exc) {
      if (stop_requested_.load()) {
        break;
      }

      report_connection_issue(
        "NTRIP reconnect after error on mountpoint " + active_mountpoint() + ": " +
        exc.what() + ". Retrying in " + std::to_string(failed_connection_retry_) + "s");

      if (fatal_error_.load()) {
        break;
      }

      close_socket();
      advance_mountpoint();
    }

    if (!stop_requested_.load()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(failed_connection_retry_));
    }
  }
}

void NtripClientNode::connect_and_stream()
{
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    socket_fd_ = create_connected_socket();
  }

  if (use_ssl_) {
    connect_ssl();
  }

  send_request();

  std::array<std::uint8_t, 4096> response_buffer{};
  int response_length = recv_bytes(response_buffer.data(), response_buffer.size());
  if (response_length <= 0) {
    throw std::runtime_error("No response received from NTRIP caster");
  }

  std::vector<std::uint8_t> response(
    response_buffer.begin(), response_buffer.begin() + response_length);
  auto [header, payload] = split_response(response);
  if (
    header.find("ICY 200 OK") == std::string::npos &&
    header.find("HTTP/1.0 200 OK") == std::string::npos &&
    header.find("HTTP/1.1 200 OK") == std::string::npos)
  {
    throw std::runtime_error("Invalid caster response: " + header);
  }

  parser_buffer_.clear();
  last_valid_rtcm_time_.reset();
  connected_at_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_logger(),
    "Connected to http://%s:%d/%s",
    host_.c_str(),
    port_,
    active_mountpoint().c_str());

  if (!payload.empty()) {
    handle_rtcm_bytes(payload);
  }

  while (!stop_requested_.load()) {
    std::array<std::uint8_t, 4096> chunk_buffer{};
    int bytes_read = recv_bytes(chunk_buffer.data(), chunk_buffer.size());
    if (bytes_read == 0) {
      throw std::runtime_error("Socket closed by caster");
    }
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (stream_timed_out()) {
          throw std::runtime_error(
                  "No valid RTCM received for " + std::to_string(rtcm_timeout_seconds_) + "s");
        }
        continue;
      }
      throw std::runtime_error(errno_message("Error while reading NTRIP stream"));
    }

    std::vector<std::uint8_t> chunk(
      chunk_buffer.begin(), chunk_buffer.begin() + bytes_read);
    handle_rtcm_bytes(chunk);
  }
}

int NtripClientNode::create_connected_socket() const
{
  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo * result = nullptr;
  const std::string port_string = std::to_string(port_);
  int address_status = getaddrinfo(host_.c_str(), port_string.c_str(), &hints, &result);
  if (address_status != 0) {
    throw std::runtime_error(
            "Failed to resolve host " + host_ + ": " + gai_strerror(address_status));
  }

  int connected_socket = -1;

  for (struct addrinfo * address = result; address != nullptr; address = address->ai_next) {
    connected_socket = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (connected_socket < 0) {
      continue;
    }

    int flags = fcntl(connected_socket, F_GETFL, 0);
    if (flags < 0 || fcntl(connected_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
      close(connected_socket);
      connected_socket = -1;
      continue;
    }

    int connect_status = connect(connected_socket, address->ai_addr, address->ai_addrlen);
    if (connect_status < 0 && errno != EINPROGRESS) {
      close(connected_socket);
      connected_socket = -1;
      continue;
    }

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(connected_socket, &write_fds);

    struct timeval timeout {};
    timeout.tv_sec = static_cast<long>(socket_timeout_seconds_);
    timeout.tv_usec = static_cast<long>((socket_timeout_seconds_ - timeout.tv_sec) * 1e6);

    int select_status = select(connected_socket + 1, nullptr, &write_fds, nullptr, &timeout);
    if (select_status <= 0) {
      close(connected_socket);
      connected_socket = -1;
      continue;
    }

    int socket_error = 0;
    socklen_t socket_error_length = sizeof(socket_error);
    if (
      getsockopt(
        connected_socket, SOL_SOCKET, SO_ERROR, &socket_error,
        &socket_error_length) < 0 ||
      socket_error != 0)
    {
      close(connected_socket);
      connected_socket = -1;
      continue;
    }

    if (fcntl(connected_socket, F_SETFL, flags) < 0) {
      close(connected_socket);
      connected_socket = -1;
      continue;
    }

    configure_socket_timeouts(connected_socket);
    break;
  }

  freeaddrinfo(result);

  if (connected_socket < 0) {
    throw std::runtime_error(
            "Failed to connect to " + host_ + ":" + std::to_string(port_));
  }

  return connected_socket;
}

void NtripClientNode::configure_socket_timeouts(int socket_fd) const
{
  struct timeval timeout {};
  timeout.tv_sec = static_cast<long>(socket_timeout_seconds_);
  timeout.tv_usec = static_cast<long>((socket_timeout_seconds_ - timeout.tv_sec) * 1e6);

  if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
    throw std::runtime_error(errno_message("Failed to set receive timeout"));
  }
  if (setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) != 0) {
    throw std::runtime_error(errno_message("Failed to set send timeout"));
  }
}

void NtripClientNode::connect_ssl()
{
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ < 0) {
    throw std::runtime_error("Cannot enable SSL without an open socket");
  }

  ssl_context_ = SSL_CTX_new(TLS_client_method());
  if (ssl_context_ == nullptr) {
    throw std::runtime_error("Failed to create SSL context");
  }

  if (!cert_.empty()) {
    if (SSL_CTX_use_certificate_file(ssl_context_, cert_.c_str(), SSL_FILETYPE_PEM) != 1) {
      throw std::runtime_error("Failed to load SSL certificate");
    }
  }
  if (!key_.empty()) {
    if (SSL_CTX_use_PrivateKey_file(ssl_context_, key_.c_str(), SSL_FILETYPE_PEM) != 1) {
      throw std::runtime_error("Failed to load SSL private key");
    }
  }
  if (!ca_cert_.empty()) {
    if (SSL_CTX_load_verify_locations(ssl_context_, ca_cert_.c_str(), nullptr) != 1) {
      throw std::runtime_error("Failed to load SSL CA certificate");
    }
  }

  ssl_handle_ = SSL_new(ssl_context_);
  if (ssl_handle_ == nullptr) {
    throw std::runtime_error("Failed to allocate SSL handle");
  }

  SSL_set_fd(ssl_handle_, socket_fd_);
  SSL_set_tlsext_host_name(ssl_handle_, host_.c_str());

  if (SSL_connect(ssl_handle_) != 1) {
    throw std::runtime_error("Failed to complete SSL handshake");
  }
}

void NtripClientNode::send_request()
{
  const std::string request = build_request();
  send_bytes(reinterpret_cast<const std::uint8_t *>(request.data()), request.size());
}

void NtripClientNode::handle_rtcm_bytes(const std::vector<std::uint8_t> & chunk)
{
  parser_buffer_.insert(parser_buffer_.end(), chunk.begin(), chunk.end());

  while (true) {
    auto packet = extract_rtcm_packet();
    if (!packet.has_value()) {
      break;
    }

    if (!validate_rtcm_packet(*packet)) {
      continue;
    }

    rtcm_msgs::msg::Message msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = rtcm_frame_id_;
    msg.message = *packet;
    publisher_->publish(msg);
    last_valid_rtcm_time_ = std::chrono::steady_clock::now();
    reset_issue_counters();
  }
}

std::optional<std::vector<std::uint8_t>> NtripClientNode::extract_rtcm_packet()
{
  while (!parser_buffer_.empty() && parser_buffer_.front() != 0xD3) {
    parser_buffer_.erase(parser_buffer_.begin());
  }

  if (parser_buffer_.size() < 3) {
    return std::nullopt;
  }

  const std::size_t payload_length =
    ((parser_buffer_[1] & 0x03U) << 8U) | parser_buffer_[2];
  const std::size_t total_length = 3U + payload_length + 3U;
  if (parser_buffer_.size() < total_length) {
    return std::nullopt;
  }

  std::vector<std::uint8_t> packet(
    parser_buffer_.begin(), parser_buffer_.begin() + static_cast<std::ptrdiff_t>(total_length));
  parser_buffer_.erase(
    parser_buffer_.begin(),
    parser_buffer_.begin() + static_cast<std::ptrdiff_t>(total_length));
  return packet;
}

bool NtripClientNode::validate_rtcm_packet(const std::vector<std::uint8_t> & packet)
{
  if (packet.size() < 6) {
    report_bad_rtcm_issue("Discarding malformed RTCM packet");
    return false;
  }

  const auto payload_begin = packet.begin() + 3;
  const auto payload_end = packet.end() - 3;
  std::vector<std::uint8_t> payload(payload_begin, payload_end);

  if (payload.empty()) {
    report_bad_rtcm_issue("Discarding empty RTCM packet");
    return false;
  }

  bool all_zero_payload = true;
  for (std::uint8_t byte : payload) {
    if (byte != 0U) {
      all_zero_payload = false;
      break;
    }
  }
  if (all_zero_payload) {
    report_bad_rtcm_issue("Discarding RTCM packet with all-zero payload");
    return false;
  }

  const std::uint32_t expected_crc =
    (static_cast<std::uint32_t>(packet[packet.size() - 3]) << 16U) |
    (static_cast<std::uint32_t>(packet[packet.size() - 2]) << 8U) |
    static_cast<std::uint32_t>(packet[packet.size() - 1]);
  const std::uint32_t actual_crc = crc24q(
    std::vector<std::uint8_t>(packet.begin(), packet.end() - 3));

  if (expected_crc != actual_crc) {
    std::ostringstream stream;
    stream << "Discarding RTCM packet with failed CRC (expected=0x" <<
      std::uppercase << std::hex << std::setw(6) << std::setfill('0') << expected_crc <<
      ", actual=0x" << std::setw(6) << actual_crc << ")";
    report_bad_rtcm_issue(stream.str());
    return false;
  }

  return true;
}

void NtripClientNode::handle_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  const std::string sentence = build_gga_sentence(*msg);

  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ < 0) {
    return;
  }

  try {
    send_bytes(reinterpret_cast<const std::uint8_t *>(sentence.data()), sentence.size());
  } catch (const std::exception & exc) {
    RCLCPP_WARN(get_logger(), "Unable to send NMEA sentence to server: %s", exc.what());
    close_socket();
  }
}

std::string NtripClientNode::build_gga_sentence(const sensor_msgs::msg::NavSatFix & fix) const
{
  std::time_t seconds = static_cast<std::time_t>(fix.header.stamp.sec);
  std::tm utc_time {};
  gmtime_r(&seconds, &utc_time);
  const int hundredths = static_cast<int>(fix.header.stamp.nanosec / 10000000U);

  const char latitude_direction = fix.latitude >= 0.0 ? 'N' : 'S';
  const char longitude_direction = fix.longitude >= 0.0 ? 'E' : 'W';
  const std::string latitude = dd_to_dmm(std::abs(fix.latitude), true);
  const std::string longitude = dd_to_dmm(std::abs(fix.longitude), false);

  int quality = 0;
  if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    quality = 1;
  } else if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
    quality = 2;
  } else if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    quality = 5;
  }

  std::ostringstream body;
  body << "$GPGGA,"
       << std::setw(2) << std::setfill('0') << utc_time.tm_hour
       << std::setw(2) << utc_time.tm_min
       << std::setw(2) << utc_time.tm_sec
       << "." << std::setw(2) << hundredths
       << "," << latitude
       << "," << latitude_direction
       << "," << longitude
       << "," << longitude_direction
       << "," << quality
       << ",05,1.0," << std::fixed << std::setprecision(1) << fix.altitude
       << ",M,-32.0,M,,0000";

  const std::string body_string = body.str();
  std::ostringstream sentence;
  sentence << body_string << "*" << std::uppercase << std::hex <<
    std::setw(2) << std::setfill('0') <<
    static_cast<int>(nmea_checksum(body_string)) << "\r\n";
  return sentence.str();
}

std::string NtripClientNode::build_request() const
{
  std::ostringstream request;
  request << "GET /" << active_mountpoint() << " HTTP/1.0\r\n";
  if (!ntrip_version_.empty() && ntrip_version_ != "None") {
    request << "Ntrip-Version: " << ntrip_version_ << "\r\n";
  }
  request << "User-Agent: NTRIP amr_sweeper_gnss\r\n";
  if (authenticate_) {
    const std::string credentials = username_ + ":" + password_;
    const int encoded_length = 4 * ((credentials.size() + 2) / 3);
    std::string encoded(encoded_length, '\0');
    EVP_EncodeBlock(
      reinterpret_cast<unsigned char *>(&encoded[0]),
      reinterpret_cast<const unsigned char *>(credentials.data()),
      static_cast<int>(credentials.size()));
    request << "Authorization: Basic " << encoded << "\r\n";
  }
  request << "\r\n";
  return request.str();
}

std::pair<std::string, std::vector<std::uint8_t>> NtripClientNode::split_response(
  const std::vector<std::uint8_t> & response) const
{
  static constexpr std::array<std::uint8_t, 4> marker{{'\r', '\n', '\r', '\n'}};

  auto it = std::search(response.begin(), response.end(), marker.begin(), marker.end());
  if (it == response.end()) {
    return {
      std::string(response.begin(), response.end()),
      {}
    };
  }

  const auto payload_begin = it + static_cast<std::ptrdiff_t>(marker.size());
  return {
    std::string(response.begin(), it),
    std::vector<std::uint8_t>(payload_begin, response.end())
  };
}

std::string NtripClientNode::active_mountpoint() const
{
  return mountpoints_[active_mountpoint_index_];
}

void NtripClientNode::advance_mountpoint()
{
  if (mountpoints_.size() < 2) {
    return;
  }

  active_mountpoint_index_ = (active_mountpoint_index_ + 1U) % mountpoints_.size();
  RCLCPP_WARN(
    get_logger(),
    "Switching NTRIP mountpoint to %s for the next reconnect attempt",
    active_mountpoint().c_str());
}

void NtripClientNode::close_socket()
{
  std::lock_guard<std::mutex> lock(socket_mutex_);

  if (ssl_handle_ != nullptr) {
    SSL_shutdown(ssl_handle_);
    SSL_free(ssl_handle_);
    ssl_handle_ = nullptr;
  }
  if (ssl_context_ != nullptr) {
    SSL_CTX_free(ssl_context_);
    ssl_context_ = nullptr;
  }
  if (socket_fd_ >= 0) {
    shutdown(socket_fd_, SHUT_RDWR);
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool NtripClientNode::stream_timed_out() const
{
  if (rtcm_timeout_seconds_ <= 0.0) {
    return false;
  }

  auto reference_time = last_valid_rtcm_time_;
  if (!reference_time.has_value()) {
    reference_time = connected_at_;
  }
  if (!reference_time.has_value()) {
    return false;
  }

  const auto elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - *reference_time).count();
  return elapsed >= rtcm_timeout_seconds_;
}

void NtripClientNode::report_connection_issue(const std::string & message)
{
  ++connection_issue_count_;
  log_escalating_issue(connection_issue_count_, message, "connection");
}

void NtripClientNode::report_bad_rtcm_issue(const std::string & message)
{
  ++bad_rtcm_issue_count_;
  log_escalating_issue(bad_rtcm_issue_count_, message, "rtcm");
}

void NtripClientNode::log_escalating_issue(
  int count,
  const std::string & message,
  const std::string & issue_type)
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

  fatal_error_message_ =
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive " + issue_type + " failures";
  RCLCPP_FATAL(get_logger(), "%s", fatal_error_message_.c_str());
  fatal_error_.store(true);
  stop_requested_.store(true);
}

void NtripClientNode::reset_issue_counters()
{
  connection_issue_count_ = 0;
  bad_rtcm_issue_count_ = 0;
}

int NtripClientNode::recv_bytes(std::uint8_t * buffer, std::size_t length)
{
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ < 0) {
    errno = EBADF;
    return -1;
  }

  if (ssl_handle_ != nullptr) {
    const int bytes_read = SSL_read(ssl_handle_, buffer, static_cast<int>(length));
    if (bytes_read > 0) {
      return bytes_read;
    }

    const int ssl_error = SSL_get_error(ssl_handle_, bytes_read);
    if (ssl_error == SSL_ERROR_ZERO_RETURN) {
      return 0;
    }
    if (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE) {
      errno = EAGAIN;
      return -1;
    }

    throw std::runtime_error("SSL read failed");
  }

  return static_cast<int>(recv(socket_fd_, buffer, length, 0));
}

void NtripClientNode::send_bytes(const std::uint8_t * data, std::size_t length)
{
  std::size_t sent = 0;
  while (sent < length) {
    int bytes_written = 0;

    if (ssl_handle_ != nullptr) {
      bytes_written = SSL_write(
        ssl_handle_,
        data + sent,
        static_cast<int>(length - sent));
      if (bytes_written <= 0) {
        throw std::runtime_error("SSL write failed");
      }
    } else {
      if (socket_fd_ < 0) {
        throw std::runtime_error("Socket is not connected");
      }
      bytes_written = static_cast<int>(send(socket_fd_, data + sent, length - sent, 0));
      if (bytes_written < 0) {
        throw std::runtime_error(errno_message("Socket write failed"));
      }
    }

    sent += static_cast<std::size_t>(bytes_written);
  }
}

std::string NtripClientNode::dd_to_dmm(double value, bool is_latitude)
{
  const int degrees = static_cast<int>(value);
  const double minutes = (value - static_cast<double>(degrees)) * 60.0;

  std::ostringstream output;
  output << std::setw(is_latitude ? 2 : 3) << std::setfill('0') << degrees
         << std::fixed << std::setprecision(4) << std::setw(7) << minutes;
  return output.str();
}

std::uint8_t NtripClientNode::nmea_checksum(const std::string & sentence)
{
  std::uint8_t checksum = 0U;
  for (std::size_t index = 1; index < sentence.size(); ++index) {
    checksum ^= static_cast<std::uint8_t>(sentence[index]);
  }
  return checksum;
}

std::uint32_t NtripClientNode::crc24q(const std::vector<std::uint8_t> & data)
{
  std::uint32_t crc = 0U;
  for (std::uint8_t byte : data) {
    crc ^= static_cast<std::uint32_t>(byte) << 16U;
    for (int index = 0; index < 8; ++index) {
      crc <<= 1U;
      if ((crc & 0x1000000U) != 0U) {
        crc ^= 0x1864CFBU;
      }
    }
  }
  return crc & 0xFFFFFFU;
}

double read_startup_retry_seconds_from_params(int argc, char ** argv)
{
  double retry_seconds = read_startup_parameter_from_params<double>(
    argc, argv, "startup_retry_seconds", 5.0);
  if (retry_seconds <= 0.0) {
    retry_seconds = 5.0;
  }
  return retry_seconds;
}

int read_startup_fatal_threshold_from_params(int argc, char ** argv)
{
  int threshold = read_startup_parameter_from_params<int>(
    argc, argv, "fatal_after_consecutive_errors", 10);
  if (threshold < 1) {
    threshold = 1;
  }
  return threshold;
}

}  // namespace amr_sweeper_gnss

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int startup_attempt = 0;
  int startup_fatal_threshold =
    amr_sweeper_gnss::read_startup_fatal_threshold_from_params(argc, argv);
  double startup_retry_seconds =
    amr_sweeper_gnss::read_startup_retry_seconds_from_params(argc, argv);

  std::shared_ptr<amr_sweeper_gnss::NtripClientNode> node;

  while (rclcpp::ok()) {
    try {
      node = std::make_shared<amr_sweeper_gnss::NtripClientNode>();
      startup_fatal_threshold = node->fatal_after_consecutive_errors();
      startup_retry_seconds = node->startup_retry_seconds();
      break;
    } catch (const std::exception & exc) {
      ++startup_attempt;
      if (startup_attempt >= startup_fatal_threshold) {
        std::cerr << "FATAL: Failed to start NTRIP client after " << startup_attempt <<
          " attempts: " << exc.what() << std::endl;
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
        return 1;
      }

      std::cerr << "ERROR: Failed to start NTRIP client (attempt " <<
        startup_attempt << "/" << startup_fatal_threshold << "): " <<
        exc.what() << ". Retrying..." << std::endl;
      if (!rclcpp::ok()) {
        break;
      }

      const auto retry_until =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(startup_retry_seconds);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < retry_until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  if (!rclcpp::ok() || node == nullptr) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !node->has_fatal_error()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  executor.remove_node(node);
  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return rclcpp::ok() ? 0 : 0;
}
