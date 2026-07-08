#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

#include "usb_cameras_node.hpp"

namespace
{

// Resolve a `/dev` symlink to its canonical target so the node talks to the
// actual device even when the parameter file uses a stable symlink name.
std::string resolve_device_path(const std::string & path)
{
  if (!std::filesystem::is_symlink(path)) {
    return path;
  }

  std::filesystem::path target_path = std::filesystem::read_symlink(path);
  if (target_path.is_relative()) {
    target_path = std::filesystem::absolute(path).parent_path() / target_path;
    target_path = std::filesystem::canonical(target_path);
  }
  return target_path.string();
}

}  // namespace

namespace amr_sweeper_usb_cameras
{

UsbCameraNode::UsbCameraNode(const rclcpp::NodeOptions & options)
: Node("usb_cameras_node", options),
  m_camera(std::make_unique<UsbCamera>()),
  m_compressed_image_msg(new sensor_msgs::msg::CompressedImage()),
  m_image_msg(new sensor_msgs::msg::Image()),
  m_image_publisher(),
  m_compressed_image_publisher(),
  m_camera_info_publisher(),
  m_parameters(),
  m_camera_info_msg(new sensor_msgs::msg::CameraInfo())
{
  declare_parameter("camera_name", "default_camera");
  declare_parameter("camera_info_url", "");
  declare_parameter("framerate", 5.0);
  declare_parameter("frame_id", "camera");
  declare_parameter("image_height", 240);
  declare_parameter("image_width", 320);
  declare_parameter("video_device", "/dev/video0");
  declare_parameter("reconnect_attempt_interval_ms", 1000);
  declare_parameter("retry_attempts_before_error", 3);
  declare_parameter("fatal_after_consecutive_errors", 10);
  declare_parameter("max_reconnect_attempts", 10);

  // Parameters are loaded once during startup. This lean node keeps its runtime
  // behavior fixed after construction instead of supporting live reconfiguration.
  get_params();
  init();
}

UsbCameraNode::~UsbCameraNode()
{
  m_timer.reset();
  m_camera_info.reset();
  m_camera.reset();
}

void UsbCameraNode::init()
{
  // Each camera instance keeps a short node name while publishing into a
  // dedicated camera topic prefix such as `rear_left_camera/image_raw`.
  const std::string image_topic = m_parameters.camera_name + "/image_raw";
  const std::string compressed_topic = image_topic + "/compressed";
  const std::string camera_info_topic = m_parameters.camera_name + "/" + m_parameters.camera_name + "_info";

  m_image_publisher =
    create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::QoS(1));
  m_compressed_image_publisher =
    create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic, rclcpp::QoS(1));
  m_camera_info_publisher =
    create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, rclcpp::QoS(1));

  // Load calibration once and fall back to the configured image size when a
  // calibration file does not yet exist for this camera.
  m_camera_info = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, m_parameters.camera_name, m_parameters.camera_info_url);

  if (!m_camera_info->isCalibrated()) {
    m_camera_info->setCameraName(m_parameters.camera_name);
    m_camera_info_msg->header.frame_id = m_parameters.frame_id;
    m_camera_info_msg->width = m_parameters.image_width;
    m_camera_info_msg->height = m_parameters.image_height;
    m_camera_info->setCameraInfo(*m_camera_info_msg);
  }

  connect_camera();

  const auto period = std::chrono::duration<double>(1.0 / m_parameters.framerate);
  m_timer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&UsbCameraNode::update, this));
}

bool UsbCameraNode::connect_camera()
{
  if (m_fatal_error || m_camera_connected) {
    return m_camera_connected;
  }

  try {
    const auto available_devices = utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
      throw std::runtime_error("Camera device is not available: " + m_parameters.device_name);
    }

    // The low-level driver owns the V4L2 stream. The node focuses on parameter
    // handling, topic naming, and publication plus reconnect policy.
    m_camera->configure(m_parameters);
    m_camera->start();
    m_camera_connected = true;
    m_retry_elapsed_ms = 0;
    reset_connection_issue_counters();

    if (std::abs(m_camera->get_configured_framerate() - m_parameters.framerate) > 0.01) {
      RCLCPP_INFO(
        get_logger(),
        "Camera driver accepted %.3f fps; publishing %s at requested %.3f fps",
        m_camera->get_configured_framerate(),
        m_parameters.camera_name.c_str(),
        m_parameters.framerate);
    }

    RCLCPP_INFO(
      get_logger(),
      "Configured %s to %dx%d at %.3f fps on %s",
      m_parameters.camera_name.c_str(),
      m_parameters.image_width,
      m_parameters.image_height,
      m_parameters.framerate,
      m_parameters.device_name.c_str());
    return true;
  } catch (const std::exception & error) {
    disconnect_camera();
    report_connection_issue(error.what());
    return false;
  }
}

void UsbCameraNode::disconnect_camera()
{
  try {
    m_camera->shutdown();
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Camera shutdown encountered an error: %s", error.what());
  }
  m_camera_connected = false;
}

void UsbCameraNode::get_params()
{
  // Fetching them first and assigning them in one place keeps the conversion
  // from ROS parameters to plain C++ values easy to follow.
  assign_params({
      get_parameter("camera_name"),
      get_parameter("camera_info_url"),
      get_parameter("frame_id"),
      get_parameter("framerate"),
      get_parameter("image_height"),
      get_parameter("image_width"),
      get_parameter("video_device"),
      get_parameter("reconnect_attempt_interval_ms"),
      get_parameter("retry_attempts_before_error"),
      get_parameter("fatal_after_consecutive_errors"),
      get_parameter("max_reconnect_attempts"),
    });
}

void UsbCameraNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "camera_name") {
      m_parameters.camera_name = parameter.as_string();
    } else if (name == "camera_info_url") {
      m_parameters.camera_info_url = parameter.as_string();
    } else if (name == "frame_id") {
      m_parameters.frame_id = parameter.as_string();
    } else if (name == "framerate") {
      m_parameters.framerate = parameter.as_double();
    } else if (name == "image_height") {
      m_parameters.image_height = parameter.as_int();
    } else if (name == "image_width") {
      m_parameters.image_width = parameter.as_int();
    } else if (name == "video_device") {
      m_parameters.device_name = resolve_device_path(parameter.as_string());
    } else if (name == "reconnect_attempt_interval_ms") {
      m_reconnect_attempt_interval_ms = parameter.as_int();
    } else if (name == "retry_attempts_before_error") {
      m_retry_attempts_before_error = parameter.as_int();
    } else if (name == "fatal_after_consecutive_errors") {
      m_fatal_after_consecutive_errors = parameter.as_int();
    } else if (name == "max_reconnect_attempts") {
      m_max_reconnect_attempts = parameter.as_int();
    }
  }

  if (m_reconnect_attempt_interval_ms < 1) {
    m_reconnect_attempt_interval_ms = 1;
  }
  if (m_retry_attempts_before_error < 1) {
    m_retry_attempts_before_error = 1;
  }
  if (m_fatal_after_consecutive_errors < 1) {
    m_fatal_after_consecutive_errors = 1;
  }
  if (m_max_reconnect_attempts < 0) {
    m_max_reconnect_attempts = 0;
  }
}

void UsbCameraNode::report_connection_issue(const std::string & message)
{
  ++m_connection_issue_count;
  ++m_reconnect_attempt_count;

  if (m_max_reconnect_attempts > 0 && m_reconnect_attempt_count >= m_max_reconnect_attempts) {
    enter_fatal_state(
      message + ". Reached reconnect limit after " + std::to_string(m_reconnect_attempt_count) +
      " attempts");
    return;
  }

  log_escalating_issue(m_connection_issue_count, message);
}

void UsbCameraNode::enter_fatal_state(const std::string & message)
{
  m_fatal_error = true;
  RCLCPP_FATAL(get_logger(), "%s", message.c_str());
  if (m_timer) {
    m_timer->cancel();
  }
  rclcpp::shutdown();
}

void UsbCameraNode::log_escalating_issue(int count, const std::string & message)
{
  if (count < m_retry_attempts_before_error) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return;
  }

  if (count < m_fatal_after_consecutive_errors) {
    if (count == m_retry_attempts_before_error) {
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

void UsbCameraNode::reset_connection_issue_counters()
{
  m_reconnect_attempt_count = 0;
  m_connection_issue_count = 0;
  m_fatal_error = false;
}

bool UsbCameraNode::take_and_send_image()
{
  // The compressed topic is always published. Raw decode is optional and only
  // happens when the raw image topic actually has subscribers.
  const bool needs_raw = m_image_publisher->get_subscription_count() > 0;

  // Allocate the RGB output buffer only when a raw subscriber needs decoded frames.
  if (needs_raw && m_image_msg->data.size() != m_camera->get_image_size_in_bytes()) {
    m_image_msg->width = m_camera->get_image_width();
    m_image_msg->height = m_camera->get_image_height();
    m_image_msg->encoding = m_camera->get_encoding();
    m_image_msg->step = m_camera->get_image_step();
    m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  m_camera->capture_frame(
    m_compressed_image_msg->data,
    needs_raw ? reinterpret_cast<char *>(m_image_msg->data.data()) : nullptr);

  // If the driver did not produce a frame this cycle, skip publication quietly.
  if (m_compressed_image_msg->data.empty()) {
    return false;
  }

  // Camera info is republished with the same timestamp as the image so
  // downstream consumers can synchronize calibration with image data.
  const auto stamp = m_camera->get_image_timestamp();
  *m_camera_info_msg = m_camera_info->getCameraInfo();

  m_compressed_image_msg->header.frame_id = m_parameters.frame_id;
  m_compressed_image_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_image_msg->header.stamp.nanosec = stamp.tv_nsec;
  m_compressed_image_msg->format = "jpeg";
  m_camera_info_msg->header = m_compressed_image_msg->header;

  m_compressed_image_publisher->publish(*m_compressed_image_msg);
  m_camera_info_publisher->publish(*m_camera_info_msg);

  if (needs_raw) {
    m_image_msg->header = m_compressed_image_msg->header;
    m_image_publisher->publish(*m_image_msg);
  }

  return true;
}

void UsbCameraNode::update()
{
  if (m_fatal_error) {
    return;
  }

  if (!m_camera_connected) {
    const auto period_ms = std::max(
      1, static_cast<int>(std::lround(1000.0 / m_parameters.framerate)));
    m_retry_elapsed_ms += period_ms;
    if (m_retry_elapsed_ms >= m_reconnect_attempt_interval_ms) {
      m_retry_elapsed_ms = 0;
      connect_camera();
    }
    return;
  }

  try {
    take_and_send_image();
  } catch (const std::exception & error) {
    disconnect_camera();
    report_connection_issue("Frame capture failed on '" + m_parameters.device_name + "': " + error.what());
  }
}

}  // namespace amr_sweeper_usb_cameras

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(amr_sweeper_usb_cameras::UsbCameraNode)
