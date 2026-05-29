// Copyright (c) 2026 O-Robotics

#include "depth_camera_watchdog_node.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>

#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace amr_sweeper_depth_camera
{

DepthCameraWatchdogNode::DepthCameraWatchdogNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_camera_watchdog", options)
{
  startup_time_ = now();
  stale_data_timeout_sec_ = declare_parameter("stale_data_timeout_sec", 8.0);
  startup_grace_sec_ = declare_parameter("startup_grace_sec", 12.0);
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

  configureTopics();

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(reconnect_attempt_interval_ms_),
    std::bind(&DepthCameraWatchdogNode::watchdogTimerCb, this));
}

void DepthCameraWatchdogNode::configureTopics()
{
  registerTopic(
    "monitor_depth_image", "depth", "depth", "sensor_msgs/msg/Image",
    declare_parameter("monitor_depth_image", true));
  registerTopic(
    "monitor_depth_camera_info", "depth_camera_info", "depth_camera_info",
    "sensor_msgs/msg/CameraInfo", declare_parameter("monitor_depth_camera_info", true));
  registerTopic(
    "monitor_color_image", "color_image", "color/image_raw", "sensor_msgs/msg/Image",
    declare_parameter("monitor_color_image", true));
  registerTopic(
    "monitor_color_camera_info", "color_camera_info", "color/camera_info",
    "sensor_msgs/msg/CameraInfo", declare_parameter("monitor_color_camera_info", true));
  registerTopic(
    "monitor_color_compressed", "color_compressed", "color/image_raw/compressed",
    "sensor_msgs/msg/CompressedImage", declare_parameter("monitor_color_compressed", true));
  registerTopic(
    "monitor_infra1_image", "infra1_image", "infra1/image_rect_raw", "sensor_msgs/msg/Image",
    declare_parameter("monitor_infra1_image", true));
  registerTopic(
    "monitor_infra1_camera_info", "infra1_camera_info", "infra1/camera_info",
    "sensor_msgs/msg/CameraInfo", declare_parameter("monitor_infra1_camera_info", true));
  registerTopic(
    "monitor_infra2_image", "infra2_image", "infra2/image_rect_raw", "sensor_msgs/msg/Image",
    declare_parameter("monitor_infra2_image", true));
  registerTopic(
    "monitor_infra2_camera_info", "infra2_camera_info", "infra2/camera_info",
    "sensor_msgs/msg/CameraInfo", declare_parameter("monitor_infra2_camera_info", true));
  registerTopic(
    "monitor_motion_imu", "motion_imu", "motion/imu", "sensor_msgs/msg/Imu",
    declare_parameter("monitor_motion_imu", true));
}

void DepthCameraWatchdogNode::registerTopic(
  const std::string & parameter_name,
  const std::string & internal_topic_name,
  const std::string & display_topic_name,
  const std::string & type_name,
  bool enabled)
{
  TopicState topic;
  topic.parameter_name = parameter_name;
  topic.internal_topic_name = internal_topic_name;
  topic.display_topic_name = display_topic_name;
  topic.type_name = type_name;
  topic.enabled = enabled;
  monitored_topics_.push_back(std::move(topic));

  if (!enabled) {
    return;
  }

  const std::size_t topic_index = monitored_topics_.size() - 1;
  monitored_topics_[topic_index].subscription = create_generic_subscription(
    monitored_topics_[topic_index].internal_topic_name,
    monitored_topics_[topic_index].type_name,
    rclcpp::SensorDataQoS(),
    [this, topic_index](std::shared_ptr<rclcpp::SerializedMessage>) {
      topicMessageCb(topic_index);
    });
}

void DepthCameraWatchdogNode::topicMessageCb(std::size_t topic_index)
{
  if (topic_index >= monitored_topics_.size()) {
    return;
  }

  auto & topic = monitored_topics_[topic_index];
  topic.last_message_time = now();
  topic.received = true;
  markRecoveredIfHealthy("Depth camera topics are healthy again.");
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

bool DepthCameraWatchdogNode::waitingForInitialTopics() const
{
  for (const auto & topic : monitored_topics_) {
    if (topic.enabled && !topic.received) {
      return true;
    }
  }
  return false;
}

std::vector<std::size_t> DepthCameraWatchdogNode::collectUnhealthyTopics() const
{
  std::vector<std::size_t> unhealthy_topics;
  for (std::size_t index = 0; index < monitored_topics_.size(); ++index) {
    const auto & topic = monitored_topics_[index];
    if (!topic.enabled) {
      continue;
    }
    if (!topic.received || isTopicStale(topic.last_message_time)) {
      unhealthy_topics.push_back(index);
    }
  }
  return unhealthy_topics;
}

bool DepthCameraWatchdogNode::allTopicsHealthy() const
{
  return collectUnhealthyTopics().empty();
}

std::string DepthCameraWatchdogNode::buildHealthMessage(
  const std::vector<std::size_t> & unhealthy_topics,
  const std::vector<std::size_t> & healthy_topics) const
{
  std::ostringstream issue;
  issue << "Depth camera topics are unavailable or stale";

  if (!unhealthy_topics.empty()) {
    issue << "; unhealthy: ";
    for (std::size_t list_index = 0; list_index < unhealthy_topics.size(); ++list_index) {
      const auto & topic = monitored_topics_[unhealthy_topics[list_index]];
      if (list_index > 0) {
        issue << ", ";
      }
      issue << "'" << topic.display_topic_name << "'";
      if (!topic.received) {
        issue << " (no messages yet)";
      } else {
        issue << " (stale)";
      }
    }
  }

  if (!healthy_topics.empty()) {
    issue << "; healthy: ";
    for (std::size_t list_index = 0; list_index < healthy_topics.size(); ++list_index) {
      if (list_index > 0) {
        issue << ", ";
      }
      issue << "'" << monitored_topics_[healthy_topics[list_index]].display_topic_name << "'";
    }
  }

  return issue.str();
}

void DepthCameraWatchdogNode::markRecoveredIfHealthy(const char * recovery_message)
{
  if (!allTopicsHealthy() || was_healthy_) {
    return;
  }

  if (full_outage_count_ > 0 || full_outage_attempt_count_ > 0) {
    RCLCPP_INFO(get_logger(), "%s", recovery_message);
  }
  resetFullOutageCounters();
  was_healthy_ = true;
}

void DepthCameraWatchdogNode::watchdogTimerCb()
{
  if (fatal_error_) {
    return;
  }

  if (startupGraceActive() && waitingForInitialTopics()) {
    return;
  }

  const auto unhealthy_topics = collectUnhealthyTopics();
  if (unhealthy_topics.empty()) {
    markRecoveredIfHealthy("Depth camera topics are healthy again.");
    return;
  }

  was_healthy_ = false;

  std::vector<std::size_t> healthy_topics;
  healthy_topics.reserve(monitored_topics_.size() - unhealthy_topics.size());
  for (std::size_t index = 0; index < monitored_topics_.size(); ++index) {
    const auto & topic = monitored_topics_[index];
    if (!topic.enabled) {
      continue;
    }
    if (std::find(unhealthy_topics.begin(), unhealthy_topics.end(), index) == unhealthy_topics.end()) {
      healthy_topics.push_back(index);
    }
  }

  const std::string issue_message = buildHealthMessage(unhealthy_topics, healthy_topics);
  const bool all_topics_unhealthy = healthy_topics.empty();
  if (!all_topics_unhealthy) {
    resetFullOutageCounters();
    if (shouldLogIssue("warn", issue_message)) {
      RCLCPP_WARN(get_logger(), "%s", issue_message.c_str());
    }
    return;
  }

  reportFullOutage(issue_message);
}

void DepthCameraWatchdogNode::reportFullOutage(const std::string & message)
{
  ++full_outage_count_;
  ++full_outage_attempt_count_;

  if (max_reconnect_attempts_ > 0 && full_outage_attempt_count_ >= max_reconnect_attempts_) {
    const std::string fatal_message =
      message + ". Reached reconnect limit after " + std::to_string(full_outage_attempt_count_) +
      " attempts";
    enterFatalState(fatal_message);
    return;
  }

  logEscalatingFullOutage(full_outage_count_, message);
}

void DepthCameraWatchdogNode::enterFatalState(const std::string & message)
{
  fatal_error_ = true;
  if (shouldLogIssue("fatal", message)) {
    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
  }
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
  }
  rclcpp::shutdown();
}

void DepthCameraWatchdogNode::logEscalatingFullOutage(int count, const std::string & message)
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
        message + ". Escalating after " + std::to_string(count) + " consecutive full outages";
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

  const std::string fatal_message =
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive full outages";
  enterFatalState(fatal_message);
}

void DepthCameraWatchdogNode::resetFullOutageCounters()
{
  full_outage_attempt_count_ = 0;
  full_outage_count_ = 0;
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
