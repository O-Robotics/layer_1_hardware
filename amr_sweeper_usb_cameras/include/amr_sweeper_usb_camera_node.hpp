#ifndef AMR_SWEEPER_USB_CAMERAS__NODE_HPP_
#define AMR_SWEEPER_USB_CAMERAS__NODE_HPP_

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
  // Read declared ROS parameters into a single local vector for assignment.
  void get_params();
  // Convert ROS parameter values into the compact `CameraParameters` struct.
  void assign_params(const std::vector<rclcpp::Parameter> & parameters);
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
  // Raw image publisher under `<camera_name>/image_raw`.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;
  // Compressed image publisher under `<camera_name>/image_raw/compressed`.
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_compressed_image_publisher;
  // Camera info publisher under `<camera_name>/<camera_name>_info`.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_camera_info_publisher;
  // Cached startup parameters for the selected camera instance.
  CameraParameters m_parameters;
  // Reused camera_info message populated from the calibration manager each frame.
  sensor_msgs::msg::CameraInfo::SharedPtr m_camera_info_msg;
  // Loads and serves the calibration YAML for the current camera.
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info;
  // Periodic capture timer running at the configured frame rate.
  rclcpp::TimerBase::SharedPtr m_timer;
};

}  // namespace amr_sweeper_usb_cameras

#endif  // AMR_SWEEPER_USB_CAMERAS__NODE_HPP_
