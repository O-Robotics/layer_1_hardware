// Copyright (c) 2026 O-Robotics

#ifndef AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_NODE_HPP_
#define AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_NODE_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <domain_bridge/domain_bridge.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amr_sweeper_depth_camera
{

class DepthCameraNode final : public rclcpp::Node
{
public:
  explicit DepthCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  domain_bridge::DomainBridge & bridge();

private:
  struct TopicSpec
  {
    std::string source_topic_name;
    std::string target_topic_name;
    std::string type_name;
    bool bridge_enabled{true};
    bool monitor_enabled{false};
    bool received{false};
    rclcpp::Time last_message_time{0, 0, RCL_ROS_TIME};
    rclcpp::SubscriptionBase::SharedPtr subscription;
  };

  void configureBridge();
  void configureSourceStreamControl();
  void configureServiceBridges();
  void configureTopicBridges();
  void addTopicBridge(
    const std::string & source_topic_name,
    const std::string & target_topic_name,
    const std::string & type_name,
    bool enabled,
    bool monitor_enabled);
  void registerMonitorSubscription(std::size_t topic_index);
  void topicMessageCb(std::size_t topic_index);

  template<typename ServiceT>
  void bridgeService(const std::string & suffix, bool enabled);

  static std::string normalizeRoot(const std::string & root);
  static std::string joinName(const std::string & root, const std::string & suffix);
  static std::string packageNameFromType(const std::string & type_name);
  bool interfacePackageAvailable(const std::string & type_name) const;

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
  std::size_t externalSubscriberCount(const TopicSpec & topic) const;
  bool isTopicDemanded(const TopicSpec & topic) const;
  std::vector<std::size_t> collectDemandedTopics() const;
  std::vector<std::size_t> collectUnhealthyTopics() const;
  std::string buildHealthMessage(
    const std::vector<std::size_t> & unhealthy_topics,
    const std::vector<std::size_t> & healthy_topics) const;
  void queueSourceBoolParameter(
    const std::string & parameter_name,
    bool value);
  void queueSourceStringParameter(
    const std::string & parameter_name,
    const std::string & value);
  void sourceStreamControlTimerCb();
  static std::string normalizeOptionalParameterName(const std::string & parameter_name);

  std::size_t source_domain_id_{5};
  std::size_t target_domain_id_{0};
  std::string source_root_namespace_{"/realsense"};
  std::string source_camera_id_;
  std::string source_pointcloud_topic_;
  std::string target_namespace_root_{"/amr_sweeper/depth_camera"};

  bool enable_watchdog_{true};
  bool watchdog_shutdown_on_fatal_{false};
  bool fatal_error_{false};
  bool was_healthy_{false};

  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  double stale_data_timeout_sec_{8.0};
  double startup_grace_sec_{12.0};
  int reconnect_attempt_interval_ms_{1000};
  int retry_attempts_before_error_{3};
  int fatal_after_consecutive_errors_{10};
  int max_reconnect_attempts_{10};
  int full_outage_attempt_count_{0};
  int full_outage_count_{0};
  std::string last_issue_log_level_;
  std::string last_issue_message_;

  domain_bridge::DomainBridge bridge_;
  std::vector<TopicSpec> monitored_topics_;
  std::vector<std::string> skipped_bridge_topics_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  bool apply_source_stream_control_on_startup_{false};
  int source_stream_control_attempts_{0};
  int source_stream_control_max_attempts_{5};
  int source_stream_control_retry_interval_ms_{1000};
  bool source_stream_control_applied_{false};
  std::vector<rclcpp::Parameter> source_stream_control_parameters_;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr
    source_stream_control_client_;
  rclcpp::TimerBase::SharedPtr source_stream_control_timer_;
};

}  // namespace amr_sweeper_depth_camera

#endif  // AMR_SWEEPER_DEPTH_CAMERA__DEPTH_CAMERA_NODE_HPP_
