// Copyright (c) 2026 O-Robotics

#include "depth_camera_watchdog_node.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>

namespace amr_sweeper_depth_camera
{

DepthCameraWatchdogNode::DepthCameraWatchdogNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_camera_watchdog", options)
{
  startup_time_ = now();
  stale_data_timeout_sec_ = declare_parameter("stale_data_timeout_sec", 8.0);
  startup_grace_sec_ = declare_parameter("startup_grace_sec", 12.0);
  require_camera_info_ = declare_parameter("require_camera_info", true);
  require_pointcloud_ = declare_parameter("require_pointcloud", true);
  reconnect_attempt_interval_ms_ = declare_parameter("reconnect_attempt_interval_ms", 1000);
  retry_attempts_before_error_ = declare_parameter("retry_attempts_before_error", 3);
  fatal_after_consecutive_errors_ = declare_parameter("fatal_after_consecutive_errors", 10);
  max_reconnect_attempts_ = declare_parameter("max_reconnect_attempts", 10);

  stale_data_timeout_sec_ = std::max(stale_data_timeout_sec_, 0.1);
  startup_grace_sec_ = std::max(startup_grace_sec_, 0.0);
  reconnect_attempt_interval_ms_ = std::max(reconnect_attempt_interval_ms_, 1);
  retry_attempts_before_error_ = std::max(retry_attempts_before_error_, 1);
  fatal_after_consecutive_errors_ = std::max(fatal_after_consecutive_errors_, 1);
  max_reconnect_attempts_ = std::max(max_reconnect_attempts_, 0);

  const auto qos = rclcpp::SystemDefaultsQoS();
  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "depth",
    qos,
    std::bind(&DepthCameraWatchdogNode::depthCb, this, std::placeholders::_1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "depth_camera_info",
    qos,
    std::bind(&DepthCameraWatchdogNode::cameraInfoCb, this, std::placeholders::_1));
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud",
    qos,
    std::bind(&DepthCameraWatchdogNode::pointCloudCb, this, std::placeholders::_1));

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(reconnect_attempt_interval_ms_),
    std::bind(&DepthCameraWatchdogNode::watchdogTimerCb, this));
}

void DepthCameraWatchdogNode::depthCb(const sensor_msgs::msg::Image::SharedPtr image)
{
  (void)image;
  last_depth_message_time_ = now();
  received_depth_message_ = true;

  if (require_camera_info_ && !received_camera_info_message_) {
    return;
  }
  if (require_pointcloud_ && !received_pointcloud_message_) {
    return;
  }

  if (!was_healthy_) {
    if (connection_issue_count_ > 0 || reconnect_attempt_count_ > 0) {
      RCLCPP_INFO(get_logger(), "Depth camera data stream recovered.");
    }
    resetConnectionIssueCounters();
    was_healthy_ = true;
  }
}

void DepthCameraWatchdogNode::cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  (void)info;
  last_camera_info_message_time_ = now();
  received_camera_info_message_ = true;
}

void DepthCameraWatchdogNode::pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  (void)cloud;
  last_pointcloud_message_time_ = now();
  received_pointcloud_message_ = true;

  if (!received_depth_message_) {
    return;
  }
  if (require_camera_info_ && !received_camera_info_message_) {
    return;
  }

  if (!was_healthy_) {
    if (connection_issue_count_ > 0 || reconnect_attempt_count_ > 0) {
      RCLCPP_INFO(get_logger(), "Depth camera point cloud stream recovered.");
    }
    resetConnectionIssueCounters();
    was_healthy_ = true;
  }
}

bool DepthCameraWatchdogNode::isTopicStale(const rclcpp::Time & last_message_time) const
{
  return (now() - last_message_time).seconds() > stale_data_timeout_sec_;
}

bool DepthCameraWatchdogNode::startupGraceActive() const
{
  if (startup_grace_sec_ <= 0.0) {
    return false;
  }
  return (now() - startup_time_).seconds() < startup_grace_sec_;
}

void DepthCameraWatchdogNode::watchdogTimerCb()
{
  if (fatal_error_) {
    return;
  }

  if (
    startupGraceActive() &&
    (!received_depth_message_ ||
    (require_camera_info_ && !received_camera_info_message_) ||
    (require_pointcloud_ && !received_pointcloud_message_)))
  {
    return;
  }

  const bool depth_healthy = received_depth_message_ && !isTopicStale(last_depth_message_time_);
  const bool camera_info_healthy =
    !require_camera_info_ ||
    (received_camera_info_message_ && !isTopicStale(last_camera_info_message_time_));
  const bool pointcloud_healthy =
    !require_pointcloud_ ||
    (received_pointcloud_message_ && !isTopicStale(last_pointcloud_message_time_));

  if (depth_healthy && camera_info_healthy && pointcloud_healthy) {
    if (!was_healthy_) {
      if (connection_issue_count_ > 0 || reconnect_attempt_count_ > 0) {
        RCLCPP_INFO(get_logger(), "Depth camera topics are healthy again.");
      }
      resetConnectionIssueCounters();
      was_healthy_ = true;
    }
    return;
  }

  was_healthy_ = false;

  std::ostringstream issue;
  issue << "Depth camera data is unavailable or stale";
  if (!received_depth_message_) {
    issue << "; no depth frames received yet";
  } else if (!depth_healthy) {
    issue << "; depth frames stopped arriving on 'depth'";
  }

  if (require_camera_info_) {
    if (!received_camera_info_message_) {
      issue << "; no camera info received yet";
    } else if (!camera_info_healthy) {
      issue << "; camera info stopped arriving on 'depth_camera_info'";
    }
  }
  if (require_pointcloud_) {
    if (!received_pointcloud_message_) {
      issue << "; no point cloud received yet on 'pointcloud'";
    } else if (!pointcloud_healthy) {
      issue << "; point cloud stopped arriving on 'pointcloud'";
    }
  }

  reportConnectionIssue(issue.str());
}

void DepthCameraWatchdogNode::reportConnectionIssue(const std::string & message)
{
  ++connection_issue_count_;
  ++reconnect_attempt_count_;

  if (max_reconnect_attempts_ > 0 && reconnect_attempt_count_ >= max_reconnect_attempts_) {
    fatal_error_ = true;
    const std::string fatal_message =
      message + ". Reached reconnect limit after " + std::to_string(reconnect_attempt_count_) +
      " attempts";
    if (shouldLogIssue("fatal", fatal_message)) {
      RCLCPP_FATAL(get_logger(), "%s", fatal_message.c_str());
    }
    if (watchdog_timer_) {
      watchdog_timer_->cancel();
    }
    return;
  }

  logEscalatingIssue(connection_issue_count_, message);
}

void DepthCameraWatchdogNode::logEscalatingIssue(int count, const std::string & message)
{
  if (count < retry_attempts_before_error_) {
    if (shouldLogIssue("warn", message)) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
    return;
  }

  if (count < fatal_after_consecutive_errors_) {
    if (count == retry_attempts_before_error_) {
      const std::string error_message =
        message + ". Escalating after " + std::to_string(count) + " consecutive failures";
      if (shouldLogIssue("error", error_message)) {
        RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
      }
      return;
    }
    if (shouldLogIssue("error", message)) {
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    }
    return;
  }

  fatal_error_ = true;
  const std::string fatal_message =
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive connection failures";
  if (shouldLogIssue("fatal", fatal_message)) {
    RCLCPP_FATAL(get_logger(), "%s", fatal_message.c_str());
  }
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
  }
}

void DepthCameraWatchdogNode::resetConnectionIssueCounters()
{
  reconnect_attempt_count_ = 0;
  connection_issue_count_ = 0;
  fatal_error_ = false;
  last_issue_log_level_.clear();
  last_issue_message_.clear();
}

bool DepthCameraWatchdogNode::shouldLogIssue(const std::string & level, const std::string & message)
{
  if (last_issue_log_level_ == level && last_issue_message_ == message) {
    return false;
  }
  last_issue_log_level_ = level;
  last_issue_message_ = message;
  return true;
}

}  // namespace amr_sweeper_depth_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_depth_camera::DepthCameraWatchdogNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
