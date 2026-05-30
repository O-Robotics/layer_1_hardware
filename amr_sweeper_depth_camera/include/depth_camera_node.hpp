// Copyright (c) 2026 O-Robotics

#ifndef AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_NODE_HPP_
#define AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace amr_sweeper_depth_camera
{

class DepthCameraNode final : public rclcpp::Node
{
public:
  explicit DepthCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DepthCameraNode() override;

private:
  struct StreamProfile
  {
    int width{0};
    int height{0};
    int fps{0};
  };

  void configureParameters();
  void configurePublishers();
  void startPipeline();
  void stopPipeline();
  void captureLoop();

  sensor_msgs::msg::CameraInfo buildCameraInfo(
    const rs2::video_stream_profile & profile,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;
  sensor_msgs::msg::Image buildImageMessage(
    const rs2::video_frame & frame,
    const std::string & encoding,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;
  sensor_msgs::msg::PointCloud2 buildPointCloudMessage(
    const rs2::depth_frame & depth_frame,
    const rs2::video_frame & color_frame,
    const rclcpp::Time & stamp);
  void publishMotionFrame(const rs2::motion_frame & motion_frame, const rclcpp::Time & stamp);

  std::string serial_no_;
  bool use_color_{true};
  bool use_depth_{true};
  bool use_infra1_{false};
  bool use_infra2_{false};
  bool use_motion_{true};
  bool publish_pointcloud_{true};
  bool align_depth_to_color_{true};
  int wait_for_frames_timeout_ms_{2000};
  StreamProfile color_profile_{848, 480, 15};
  StreamProfile depth_profile_{848, 480, 15};
  StreamProfile infra_profile_{848, 480, 15};
  int accel_fps_{100};
  int gyro_fps_{200};
  std::string color_frame_id_{"depth_camera_color_optical_frame"};
  std::string depth_frame_id_{"depth_camera_depth_optical_frame"};
  std::string infra1_frame_id_{"depth_camera_depth_optical_frame"};
  std::string infra2_frame_id_{"depth_camera_depth_optical_frame"};
  std::string imu_frame_id_{"depth_camera_imu_frame"};
  std::string pointcloud_frame_id_{"depth_camera_color_optical_frame"};

  rs2::pipeline pipeline_;
  rs2::config pipeline_config_;
  std::optional<rs2::pipeline_profile> pipeline_profile_;
  std::unique_ptr<rs2::align> align_to_color_processor_;
  rs2::pointcloud pointcloud_processor_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::mutex imu_mutex_;
  sensor_msgs::msg::Imu latest_imu_;
  bool have_accel_{false};
  bool have_gyro_{false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infra1_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infra2_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

}  // namespace amr_sweeper_depth_camera

#endif  // AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_NODE_HPP_
