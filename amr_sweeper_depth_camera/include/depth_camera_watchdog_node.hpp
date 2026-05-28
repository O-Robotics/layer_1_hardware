// Copyright (c) 2026 O-Robotics

#ifndef AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_WATCHDOG_NODE_HPP_
#define AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_WATCHDOG_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace amr_sweeper_depth_camera
{

class DepthCameraWatchdogNode final : public rclcpp::Node
{
public:
  explicit DepthCameraWatchdogNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void depthCb(const sensor_msgs::msg::Image::SharedPtr image);
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr info);
  void watchdogTimerCb();
  void reportConnectionIssue(const std::string & message);
  void logEscalatingIssue(int count, const std::string & message);
  void resetConnectionIssueCounters();
  bool shouldLogIssue(const std::string & level, const std::string & message);
  bool isTopicStale(const rclcpp::Time & last_message_time) const;
  bool startupGraceActive() const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_depth_message_time_;
  rclcpp::Time last_camera_info_message_time_;
  bool received_depth_message_{false};
  bool received_camera_info_message_{false};
  bool require_camera_info_{true};
  bool fatal_error_{false};
  bool was_healthy_{false};

  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  double stale_data_timeout_sec_{8.0};
  double startup_grace_sec_{0.0};
  int reconnect_attempt_interval_ms_{1000};
  int retry_attempts_before_error_{3};
  int fatal_after_consecutive_errors_{10};
  int max_reconnect_attempts_{10};
  int reconnect_attempt_count_{0};
  int connection_issue_count_{0};
  std::string last_issue_log_level_;
  std::string last_issue_message_;
};

}  // namespace amr_sweeper_depth_camera

#endif  // AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_WATCHDOG_NODE_HPP_
