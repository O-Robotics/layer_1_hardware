#ifndef AMR_SWEEPER_USB_CAMERAS__AMR_SWEEPER_USB_CAMERAS_NODE_HPP_
#define AMR_SWEEPER_USB_CAMERAS__AMR_SWEEPER_USB_CAMERAS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "amr_sweeper_usb_camera.hpp"

namespace amr_sweeper_usb_cameras
{

// ROS 2 node wrapper around one `UsbCamera` instance.
// The node always publishes the incoming MJPEG stream and only publishes a raw
// RGB image when something subscribes to the raw topic.
class UsbCameraNode : public rclcpp::Node
{
public:
  explicit UsbCameraNode(const rclcpp::NodeOptions & options);
  ~UsbCameraNode() override;

private:
  // Create publishers, load camera calibration, validate the device, and start streaming.
  void init();
  // Try to configure and start the camera if it is currently disconnected.
  bool connect_camera();
  // Stop streaming and release the current device handle after a failure.
  void disconnect_camera();
  // Read declared ROS parameters into a single local vector for assignment.
  void get_params();
  // Convert ROS parameter values into the compact `CameraParameters` struct.
  void assign_params(const std::vector<rclcpp::Parameter> & parameters);
  // Emit warn/error/fatal logs that escalate like the IMU reconnect path.
  void report_connection_issue(const std::string & message);
  void log_escalating_issue(int count, const std::string & message);
  void reset_connection_issue_counters();
  // Timer callback entry point.
  void update();
  // Capture one frame and publish compressed, camera_info, and optional raw outputs.
  bool take_and_send_image();

  // Driver for the active USB camera device.
  std::unique_ptr<UsbCamera> m_camera;
  // Reused compressed-image message to avoid reallocating the message object every frame.
  sensor_msgs::msg::CompressedImage::UniquePtr m_compressed_image_msg;
  // Reused raw-image message. Its data buffer is resized only when raw output is needed.
  sensor_msgs::msg::Image::UniquePtr m_image_msg;
  // Raw image publisher under `image_raw` in the node namespace.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;
  // Compressed image publisher under `image_raw/compressed` in the node namespace.
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_compressed_image_publisher;
  // Camera info publisher under `<camera_name>_info` in the node namespace.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_camera_info_publisher;
  // Cached startup parameters for the selected camera instance.
  CameraParameters m_parameters;
  // Reused camera_info message populated from the calibration manager each frame.
  sensor_msgs::msg::CameraInfo::SharedPtr m_camera_info_msg;
  // Loads and serves the calibration YAML for the current camera.
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info;
  // Periodic capture timer running at the configured frame rate.
  rclcpp::TimerBase::SharedPtr m_timer;
  // Tracks whether the V4L2 device is currently configured and streaming.
  bool m_camera_connected{false};
  // Delay between reconnect attempts after a USB or V4L2 failure.
  int m_reconnect_attempt_interval_ms{1000};
  // Consecutive failures allowed before WARN escalates to ERROR.
  int m_retry_attempts_before_error{3};
  // Consecutive failures allowed before the node stops retrying.
  int m_fatal_after_consecutive_errors{10};
  // Hard cap on reconnect attempts. Set to 0 for unlimited retries.
  int m_max_reconnect_attempts{10};
  // Accumulated timer time while waiting to attempt a reconnect.
  int m_retry_elapsed_ms{0};
  // Total number of reconnect attempts since the last successful camera open.
  int m_reconnect_attempt_count{0};
  // Consecutive connection-related failures used for warn/error/fatal escalation.
  int m_connection_issue_count{0};
  // Stops retry attempts after the fatal threshold is reached.
  bool m_fatal_error{false};
};

}  // namespace amr_sweeper_usb_cameras

#endif  // AMR_SWEEPER_USB_CAMERAS__AMR_SWEEPER_USB_CAMERAS_NODE_HPP_
