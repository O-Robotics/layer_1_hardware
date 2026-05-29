// Copyright (c) 2026 O-Robotics

#ifndef AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_WATCHDOG_NODE_HPP_
#define AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_WATCHDOG_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace amr_sweeper_depth_camera
{

class DepthCameraWatchdogNode final : public rclcpp::Node
{
public:
  explicit DepthCameraWatchdogNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct TopicState
  {
    std::string parameter_name;
    std::string internal_topic_name;
    std::string display_topic_name;
    std::string type_name;
    bool enabled{true};
    bool received{false};
    rclcpp::Time last_message_time{0, 0, RCL_ROS_TIME};
    rclcpp::SubscriptionBase::SharedPtr subscription;
  };

  void configureTopics();
  void registerTopic(
    const std::string & parameter_name,
    const std::string & internal_topic_name,
    const std::string & display_topic_name,
    const std::string & type_name,
    bool enabled);
  void topicMessageCb(std::size_t topic_index);
  void watchdogTimerCb();
  void enterFatalState(const std::string & message);
  void reportFullOutage(const std::string & message);
  void logEscalatingFullOutage(int count, const std::string & message);
  bool allTopicsHealthy() const;
  void markRecoveredIfHealthy(const char * recovery_message);
  void resetFullOutageCounters();
  bool shouldLogIssue(const std::string & level, const std::string & message);
  bool isTopicStale(const rclcpp::Time & last_message_time) const;
  bool startupGraceActive() const;
  bool waitingForInitialTopics() const;
  std::string buildHealthMessage(
    const std::vector<std::size_t> & unhealthy_topics,
    const std::vector<std::size_t> & healthy_topics) const;
  std::vector<std::size_t> collectUnhealthyTopics() const;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::vector<TopicState> monitored_topics_;

  bool fatal_error_{false};
  bool was_healthy_{false};

  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  double stale_data_timeout_sec_{8.0};
  double startup_grace_sec_{0.0};
  int reconnect_attempt_interval_ms_{1000};
  int retry_attempts_before_error_{3};
  int fatal_after_consecutive_errors_{10};
  int max_reconnect_attempts_{10};
  int full_outage_attempt_count_{0};
  int full_outage_count_{0};
  std::string last_issue_log_level_;
  std::string last_issue_message_;
};

}  // namespace amr_sweeper_depth_camera

#endif  // AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_WATCHDOG_NODE_HPP_
