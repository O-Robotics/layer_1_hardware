// Copyright (c) 2026 O-Robotics

#include "depth_camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <utility>

#include <ament_index_cpp/ament_index_cpp/get_package_prefix.hpp>
#include <domain_bridge/domain_bridge_options.hpp>
#include <domain_bridge/service_bridge_options.hpp>
#include <domain_bridge/topic_bridge_options.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/serialized_message.hpp>
#include <realsense2_camera_msgs/srv/application_config_read.hpp>
#include <realsense2_camera_msgs/srv/application_config_write.hpp>
#include <realsense2_camera_msgs/srv/calib_config_read.hpp>
#include <realsense2_camera_msgs/srv/calib_config_write.hpp>
#include <realsense2_camera_msgs/srv/device_info.hpp>
#include <realsense2_camera_msgs/srv/hardware_monitor_command_send.hpp>
#include <realsense2_camera_msgs/srv/safety_interface_config_read.hpp>
#include <realsense2_camera_msgs/srv/safety_interface_config_write.hpp>
#include <realsense2_camera_msgs/srv/safety_preset_read.hpp>
#include <realsense2_camera_msgs/srv/safety_preset_write.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace amr_sweeper_depth_camera
{

DepthCameraNode::DepthCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_camera", options),
  bridge_([]() {
    domain_bridge::DomainBridgeOptions bridge_options;
    bridge_options.name("amr_sweeper_depth_camera_bridge");
    return bridge_options;
  }())
{
  source_domain_id_ = static_cast<std::size_t>(declare_parameter<int>("camera_domain_id", 5));
  target_domain_id_ = static_cast<std::size_t>(declare_parameter<int>("workspace_domain_id", 0));
  source_root_namespace_ = normalizeRoot(
    declare_parameter<std::string>("source_root_namespace", "/realsense"));
  source_camera_id_ = declare_parameter<std::string>("source_camera_id", "");
  source_pointcloud_topic_ = declare_parameter<std::string>("source_pointcloud_topic", "");
  target_namespace_root_ = normalizeRoot(
    declare_parameter<std::string>("target_namespace_root", get_namespace()));

  enable_watchdog_ = declare_parameter("enable_watchdog", true);
  watchdog_shutdown_on_fatal_ = declare_parameter("watchdog_shutdown_on_fatal", false);
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

  startup_time_ = now();
  configureBridge();
  configureSourceStreamControl();

  if (!skipped_bridge_topics_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Skipping optional bridge topics due to unavailable interfaces: %s",
      [&]() {
        std::ostringstream stream;
        for (std::size_t i = 0; i < skipped_bridge_topics_.size(); ++i) {
          if (i > 0) {
            stream << ", ";
          }
          stream << skipped_bridge_topics_[i];
        }
        return stream.str();
      }().c_str());
  }

  if (enable_watchdog_) {
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(reconnect_attempt_interval_ms_),
      std::bind(&DepthCameraNode::watchdogTimerCb, this));
  }
}

domain_bridge::DomainBridge & DepthCameraNode::bridge()
{
  return bridge_;
}

void DepthCameraNode::configureBridge()
{
  configureTopicBridges();
  configureServiceBridges();
}

void DepthCameraNode::configureSourceStreamControl()
{
  apply_source_stream_control_on_startup_ = declare_parameter(
    "apply_source_stream_control_on_startup", false);
  const bool apply_source_profiles_on_startup = declare_parameter(
    "apply_source_profiles_on_startup", false);
  source_stream_control_retry_interval_ms_ = std::max(
    static_cast<int>(declare_parameter("source_stream_control_retry_interval_ms", 1000L)),
    100);
  source_stream_control_max_attempts_ = std::max(
    static_cast<int>(declare_parameter("source_stream_control_max_attempts", 5L)),
    1);

  const auto get_or_declare_bool =
    [this](const char * name, bool default_value)
    {
      if (has_parameter(name)) {
        return get_parameter(name).as_bool();
      }
      return declare_parameter(name, default_value);
    };

  const auto get_or_declare_string =
    [this](const char * name, const std::string & default_value)
    {
      if (has_parameter(name)) {
        return get_parameter(name).as_string();
      }
      return declare_parameter(name, default_value);
    };

  const auto queue_stream_parameters =
    [this, &get_or_declare_bool, &get_or_declare_string, apply_source_profiles_on_startup](
      const std::string & use_parameter_name, const std::string & default_enable_parameter_name,
      const std::string & profile_name_parameter_name,
      const std::string & profile_parameter_parameter_name,
      const std::string & default_profile_parameter_name)
    {
      const bool enabled = get_or_declare_bool(use_parameter_name.c_str(), true);
      const std::string profile_parameter_name = normalizeOptionalParameterName(
        get_or_declare_string(
          profile_parameter_parameter_name.c_str(),
          default_profile_parameter_name));
      const std::string profile = get_or_declare_string(
        profile_name_parameter_name.c_str(),
        std::string(""));

      queueSourceBoolParameter(default_enable_parameter_name, enabled);
      if (enabled && apply_source_profiles_on_startup) {
        queueSourceStringParameter(profile_parameter_name, profile);
      }
    };

  const bool use_color = get_or_declare_bool("use_color", true);
  const bool use_compressed_color = get_or_declare_bool("use_compressed_color", true);
  const bool source_color_enabled = use_color || use_compressed_color;
  const std::string color_profile_name = normalizeOptionalParameterName(
    get_or_declare_string("color_profile_name", std::string("rgb_camera.profile")));
  const std::string color_profile_parameter = get_or_declare_string(
    "color_profile_parameter", std::string(""));
  const std::string compressed_color_profile_name = normalizeOptionalParameterName(
    get_or_declare_string("compressed_color_profile_name", std::string("rgb_camera.profile")));
  const std::string compressed_color_profile_parameter = get_or_declare_string(
    "compressed_color_profile_parameter", std::string(""));

  queueSourceBoolParameter("enable_color", source_color_enabled);
  if (source_color_enabled && apply_source_profiles_on_startup) {
    if (use_color) {
      queueSourceStringParameter(color_profile_name, color_profile_parameter);
      if (use_compressed_color &&
        color_profile_name == compressed_color_profile_name &&
        !compressed_color_profile_parameter.empty() &&
        compressed_color_profile_parameter != color_profile_parameter)
      {
        RCLCPP_WARN(
          get_logger(),
          "Compressed color profile '%s' differs from raw color profile '%s' but both map to '%s'; "
          "keeping the raw color profile to match the native RGB stream.",
          compressed_color_profile_parameter.c_str(),
          color_profile_parameter.c_str(),
          color_profile_name.c_str());
      }
    } else {
      queueSourceStringParameter(
        compressed_color_profile_name,
        compressed_color_profile_parameter);
    }
  }

  queue_stream_parameters(
    "use_depth",
    "enable_depth",
    "depth_profile_name",
    "depth_profile_parameter",
    "depth_module.depth_profile");
  queue_stream_parameters(
    "use_infra1",
    "enable_infra1",
    "infra1_profile_name",
    "infra1_profile_parameter",
    "depth_module.infra1_profile");
  queue_stream_parameters(
    "use_infra2",
    "enable_infra2",
    "infra2_profile_name",
    "infra2_profile_parameter",
    "depth_module.infra2_profile");
  queue_stream_parameters(
    "use_motion",
    "enable_motion",
    "motion_profile_name",
    "motion_profile_parameter",
    "motion_module.profile");

  if (!apply_source_stream_control_on_startup_ || source_stream_control_parameters_.empty()) {
    return;
  }

  source_stream_control_client_ = create_client<rcl_interfaces::srv::SetParametersAtomically>(
    joinName(target_namespace_root_, "set_parameters_atomically"));
  source_stream_control_timer_ = create_wall_timer(
    std::chrono::milliseconds(source_stream_control_retry_interval_ms_),
    std::bind(&DepthCameraNode::sourceStreamControlTimerCb, this));
}

void DepthCameraNode::configureServiceBridges()
{
  bridgeService<rcl_interfaces::srv::DescribeParameters>(
    "describe_parameters", declare_parameter("bridge_describe_parameters", true));
  bridgeService<realsense2_camera_msgs::srv::ApplicationConfigRead>(
    "application_config_read", declare_parameter("bridge_application_config_read", true));
  bridgeService<realsense2_camera_msgs::srv::ApplicationConfigWrite>(
    "application_config_write", declare_parameter("bridge_application_config_write", true));
  bridgeService<realsense2_camera_msgs::srv::CalibConfigRead>(
    "calib_config_read", declare_parameter("bridge_calib_config_read", true));
  bridgeService<realsense2_camera_msgs::srv::CalibConfigWrite>(
    "calib_config_write", declare_parameter("bridge_calib_config_write", true));
  bridgeService<realsense2_camera_msgs::srv::DeviceInfo>(
    "get_device_info", declare_parameter("bridge_get_device_info", true));
  bridgeService<std_srvs::srv::Trigger>(
    "get_device_info_std", declare_parameter("bridge_get_device_info_std", true));
  bridgeService<rcl_interfaces::srv::GetParameterTypes>(
    "get_parameter_types", declare_parameter("bridge_get_parameter_types", true));
  bridgeService<rcl_interfaces::srv::GetParameters>(
    "get_parameters", declare_parameter("bridge_get_parameters", true));
  bridgeService<realsense2_camera_msgs::srv::HardwareMonitorCommandSend>(
    "hardware_monitor_command", declare_parameter("bridge_hardware_monitor_command", true));
  bridgeService<std_srvs::srv::Trigger>(
    "help", declare_parameter("bridge_help", true));
  bridgeService<std_srvs::srv::Empty>(
    "hw_reset", declare_parameter("bridge_hw_reset", true));
  bridgeService<rcl_interfaces::srv::ListParameters>(
    "list_parameters", declare_parameter("bridge_list_parameters", true));
  bridgeService<realsense2_camera_msgs::srv::SafetyInterfaceConfigRead>(
    "safety_interface_config_read",
    declare_parameter("bridge_safety_interface_config_read", true));
  bridgeService<realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite>(
    "safety_interface_config_write",
    declare_parameter("bridge_safety_interface_config_write", true));
  bridgeService<realsense2_camera_msgs::srv::SafetyPresetRead>(
    "safety_preset_read", declare_parameter("bridge_safety_preset_read", true));
  bridgeService<realsense2_camera_msgs::srv::SafetyPresetWrite>(
    "safety_preset_write", declare_parameter("bridge_safety_preset_write", true));
  bridgeService<rcl_interfaces::srv::SetParameters>(
    "set_parameters", declare_parameter("bridge_set_parameters", true));
  bridgeService<rcl_interfaces::srv::SetParametersAtomically>(
    "set_parameters_atomically", declare_parameter("bridge_set_parameters_atomically", true));
}

void DepthCameraNode::configureTopicBridges()
{
  const std::string source_camera_root = joinName(source_root_namespace_, source_camera_id_);
  const auto get_or_declare_bool =
    [this](const char * name, bool default_value)
    {
      if (has_parameter(name)) {
        return get_parameter(name).as_bool();
      }
      return declare_parameter(name, default_value);
    };

  const bool use_tf_static = get_or_declare_bool("use_tf_static", true);
  const bool use_color = get_or_declare_bool("use_color", true);
  const bool use_compressed_color = get_or_declare_bool("use_compressed_color", true);
  const bool use_depth = get_or_declare_bool("use_depth", true);
  const bool use_infra1 = get_or_declare_bool("use_infra1", true);
  const bool use_infra2 = get_or_declare_bool("use_infra2", true);
  const bool use_motion = get_or_declare_bool("use_motion", true);
  auto sensor_bridge_options = []() {
      domain_bridge::TopicBridgeOptions options;
      domain_bridge::QosOptions qos_options;
      qos_options.reliability(rclcpp::ReliabilityPolicy::BestEffort);
      qos_options.durability(rclcpp::DurabilityPolicy::Volatile);
      qos_options.history(rclcpp::HistoryPolicy::KeepLast);
      qos_options.depth(10);
      options.qos_options(qos_options);
      options.wait_for_subscription(true);
      options.wait_for_publisher(false);
      options.auto_remove(domain_bridge::TopicBridgeOptions::AutoRemove::OnNoSubscription);
      return options;
    };

  auto tf_static_bridge_options = []() {
      domain_bridge::TopicBridgeOptions options;
      domain_bridge::QosOptions qos_options;
      qos_options.reliability(rclcpp::ReliabilityPolicy::Reliable);
      qos_options.durability(rclcpp::DurabilityPolicy::TransientLocal);
      qos_options.history(rclcpp::HistoryPolicy::KeepLast);
      qos_options.depth(1);
      options.qos_options(qos_options);
      options.wait_for_subscription(true);
      options.wait_for_publisher(false);
      options.auto_remove(domain_bridge::TopicBridgeOptions::AutoRemove::OnNoSubscription);
      return options;
    };

  addTopicBridge(
    joinName(source_camera_root, "tf_static"),
    joinName(target_namespace_root_, "tf_static"),
    "tf2_msgs/msg/TFMessage", use_tf_static, false, tf_static_bridge_options());
  addTopicBridge(
    source_camera_root + "_Color",
    joinName(target_namespace_root_, "color/image"),
    "sensor_msgs/msg/Image", use_color, use_color, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Color/camera_info",
    joinName(target_namespace_root_, "color/camera_info"),
    "sensor_msgs/msg/CameraInfo", use_color, use_color, sensor_bridge_options());
  addTopicBridge(
    // Keep compressed color separately switchable so downstream consumers can opt out of raw color.
    source_camera_root + "_CompressedColor",
    joinName(target_namespace_root_, "compressed/image"),
    "sensor_msgs/msg/CompressedImage", use_compressed_color, use_compressed_color, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_CompressedColor/camera_info",
    joinName(target_namespace_root_, "compressed/camera_info"),
    "sensor_msgs/msg/CameraInfo", use_compressed_color, false, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Depth",
    joinName(target_namespace_root_, "depth/image"),
    "sensor_msgs/msg/Image", use_depth, use_depth, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Depth/camera_info",
    joinName(target_namespace_root_, "depth/camera_info"),
    "sensor_msgs/msg/CameraInfo", use_depth, use_depth, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Infrared_1",
    joinName(target_namespace_root_, "infra1/image"),
    "sensor_msgs/msg/Image", use_infra1, use_infra1, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Infrared_1/camera_info",
    joinName(target_namespace_root_, "infra1/camera_info"),
    "sensor_msgs/msg/CameraInfo", use_infra1, use_infra1, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Infrared_2",
    joinName(target_namespace_root_, "infra2/image"),
    "sensor_msgs/msg/Image", use_infra2, use_infra2, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Infrared_2/camera_info",
    joinName(target_namespace_root_, "infra2/camera_info"),
    "sensor_msgs/msg/CameraInfo", use_infra2, use_infra2, sensor_bridge_options());
  addTopicBridge(
    source_camera_root + "_Motion",
    joinName(target_namespace_root_, "motion/imu"),
    "sensor_msgs/msg/Imu", use_motion, use_motion, sensor_bridge_options());
}

void DepthCameraNode::addTopicBridge(
  const std::string & source_topic_name,
  const std::string & target_topic_name,
  const std::string & type_name,
  bool enabled,
  bool monitor_enabled,
  const domain_bridge::TopicBridgeOptions & bridge_options)
{
  TopicSpec topic;
  topic.source_topic_name = source_topic_name;
  topic.target_topic_name = target_topic_name;
  topic.type_name = type_name;
  topic.bridge_enabled = enabled;
  topic.monitor_enabled = monitor_enabled;
  monitored_topics_.push_back(topic);

  TopicSpec & stored_topic = monitored_topics_.back();
  if (stored_topic.bridge_enabled) {
    if (!interfacePackageAvailable(stored_topic.type_name)) {
      stored_topic.bridge_enabled = false;
      stored_topic.monitor_enabled = false;
      skipped_bridge_topics_.push_back(
        stored_topic.source_topic_name + " (" + stored_topic.type_name + ")");
    } else {
      domain_bridge::TopicBridgeOptions options = bridge_options;
      options.remap_name(stored_topic.target_topic_name);
      bridge_.bridge_topic(
        stored_topic.source_topic_name,
        stored_topic.type_name,
        source_domain_id_,
        target_domain_id_,
        options);
    }
  }

  if (enable_watchdog_ && stored_topic.monitor_enabled) {
    registerMonitorSubscription(monitored_topics_.size() - 1);
  }
}

void DepthCameraNode::registerMonitorSubscription(std::size_t topic_index)
{
  auto & topic = monitored_topics_[topic_index];
  topic.subscription = create_generic_subscription(
    topic.target_topic_name,
    topic.type_name,
    rclcpp::SensorDataQoS(),
    [this, topic_index](std::shared_ptr<rclcpp::SerializedMessage>) {
      topicMessageCb(topic_index);
    });
}

void DepthCameraNode::topicMessageCb(std::size_t topic_index)
{
  if (topic_index >= monitored_topics_.size()) {
    return;
  }

  auto & topic = monitored_topics_[topic_index];
  topic.last_message_time = now();
  topic.received = true;
  markRecoveredIfHealthy("Depth camera topics are healthy again.");
}

std::string DepthCameraNode::normalizeRoot(const std::string & root)
{
  if (root.empty()) {
    return "/";
  }

  std::string normalized = root;
  if (normalized.front() != '/') {
    normalized.insert(normalized.begin(), '/');
  }
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized;
}

std::string DepthCameraNode::joinName(const std::string & root, const std::string & suffix)
{
  const std::string normalized_root = normalizeRoot(root);
  if (normalized_root == "/") {
    return normalized_root + suffix;
  }
  return normalized_root + "/" + suffix;
}

std::string DepthCameraNode::packageNameFromType(const std::string & type_name)
{
  const std::size_t separator = type_name.find('/');
  if (separator == std::string::npos) {
    return {};
  }
  return type_name.substr(0, separator);
}

std::string DepthCameraNode::normalizeOptionalParameterName(const std::string & parameter_name)
{
  if (parameter_name.empty()) {
    return {};
  }

  std::string normalized = parameter_name;
  while (!normalized.empty() && normalized.front() == '/') {
    normalized.erase(normalized.begin());
  }
  return normalized;
}

void DepthCameraNode::queueSourceBoolParameter(
  const std::string & parameter_name,
  bool value)
{
  if (parameter_name.empty()) {
    return;
  }
  const auto existing = std::find_if(
    source_stream_control_parameters_.begin(),
    source_stream_control_parameters_.end(),
    [&parameter_name](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == parameter_name;
    });
  if (existing != source_stream_control_parameters_.end()) {
    *existing = rclcpp::Parameter(parameter_name, value);
    return;
  }
  source_stream_control_parameters_.emplace_back(parameter_name, value);
}

void DepthCameraNode::queueSourceStringParameter(
  const std::string & parameter_name,
  const std::string & value)
{
  if (parameter_name.empty() || value.empty()) {
    return;
  }
  const auto existing = std::find_if(
    source_stream_control_parameters_.begin(),
    source_stream_control_parameters_.end(),
    [&parameter_name](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == parameter_name;
    });
  if (existing != source_stream_control_parameters_.end()) {
    *existing = rclcpp::Parameter(parameter_name, value);
    return;
  }
  source_stream_control_parameters_.emplace_back(parameter_name, value);
}

void DepthCameraNode::sourceStreamControlTimerCb()
{
  if (source_stream_control_applied_ || !source_stream_control_client_ || !source_stream_control_timer_) {
    return;
  }

  if (!source_stream_control_client_->wait_for_service(std::chrono::seconds(0))) {
    ++source_stream_control_attempts_;
    if (source_stream_control_attempts_ >= source_stream_control_max_attempts_) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping startup stream control because '%s' did not become available after %d attempts.",
        joinName(target_namespace_root_, "set_parameters_atomically").c_str(),
        source_stream_control_attempts_);
      source_stream_control_timer_->cancel();
    }
    return;
  }

  auto request =
    std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
  request->parameters.reserve(source_stream_control_parameters_.size());
  for (const auto & parameter : source_stream_control_parameters_) {
    request->parameters.push_back(parameter.to_parameter_msg());
  }

  source_stream_control_applied_ = true;
  source_stream_control_timer_->cancel();
  source_stream_control_client_->async_send_request(
    request,
    [this](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (!response->result.successful) {
          RCLCPP_WARN(
            get_logger(),
            "Source stream control request was rejected: %s",
            response->result.reason.c_str());
          return;
        }
        RCLCPP_INFO(
          get_logger(),
          "Applied %zu startup source stream control parameter(s) to the D555.",
          source_stream_control_parameters_.size());
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to apply startup source stream control parameters: %s",
          exception.what());
      }
    });
}

bool DepthCameraNode::interfacePackageAvailable(const std::string & type_name) const
{
  const std::string package_name = packageNameFromType(type_name);
  if (package_name.empty()) {
    return false;
  }

  try {
    (void)ament_index_cpp::get_package_prefix(package_name);
    return true;
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return false;
  }
}

bool DepthCameraNode::isTopicStale(const rclcpp::Time & last_message_time) const
{
  return (now() - last_message_time).seconds() > stale_data_timeout_sec_;
}

bool DepthCameraNode::startupGraceActive() const
{
  if (startup_grace_sec_ <= 0.0) {
    return false;
  }
  return (now() - startup_time_).seconds() < startup_grace_sec_;
}

std::size_t DepthCameraNode::externalSubscriberCount(const TopicSpec & topic) const
{
  std::size_t subscribers = count_subscribers(topic.target_topic_name);
  if (topic.subscription && subscribers > 0U) {
    --subscribers;
  }
  return subscribers;
}

bool DepthCameraNode::isTopicDemanded(const TopicSpec & topic) const
{
  if (!topic.monitor_enabled) {
    return false;
  }
  return externalSubscriberCount(topic) > 0U;
}

bool DepthCameraNode::waitingForInitialTopics() const
{
  for (const auto & topic : monitored_topics_) {
    if (isTopicDemanded(topic) && !topic.received) {
      return true;
    }
  }
  return false;
}

std::vector<std::size_t> DepthCameraNode::collectDemandedTopics() const
{
  std::vector<std::size_t> demanded_topics;
  demanded_topics.reserve(monitored_topics_.size());
  for (std::size_t index = 0; index < monitored_topics_.size(); ++index) {
    if (isTopicDemanded(monitored_topics_[index])) {
      demanded_topics.push_back(index);
    }
  }
  return demanded_topics;
}

std::vector<std::size_t> DepthCameraNode::collectUnhealthyTopics() const
{
  std::vector<std::size_t> unhealthy_topics;
  for (const std::size_t index : collectDemandedTopics()) {
    const auto & topic = monitored_topics_[index];
    if (!topic.received || isTopicStale(topic.last_message_time)) {
      unhealthy_topics.push_back(index);
    }
  }
  return unhealthy_topics;
}

bool DepthCameraNode::allTopicsHealthy() const
{
  return collectUnhealthyTopics().empty();
}

std::string DepthCameraNode::buildHealthMessage(
  const std::vector<std::size_t> & unhealthy_topics,
  const std::vector<std::size_t> & healthy_topics) const
{
  std::ostringstream issue;
  issue << "Depth camera topics are unavailable or stale";

  if (!unhealthy_topics.empty()) {
    issue << "; unhealthy: ";
    for (std::size_t index = 0; index < unhealthy_topics.size(); ++index) {
      const auto & topic = monitored_topics_[unhealthy_topics[index]];
      if (index > 0) {
        issue << ", ";
      }
      issue << "'" << topic.target_topic_name << "'";
      issue << (topic.received ? " (stale)" : " (no messages yet)");
    }
  }

  if (!healthy_topics.empty()) {
    issue << "; healthy: ";
    for (std::size_t index = 0; index < healthy_topics.size(); ++index) {
      if (index > 0) {
        issue << ", ";
      }
      issue << "'" << monitored_topics_[healthy_topics[index]].target_topic_name << "'";
    }
  }

  return issue.str();
}

void DepthCameraNode::watchdogTimerCb()
{
  if (fatal_error_) {
    return;
  }

  const auto demanded_topics = collectDemandedTopics();
  if (demanded_topics.empty()) {
    resetFullOutageCounters();
    was_healthy_ = false;
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
  healthy_topics.reserve(demanded_topics.size() - unhealthy_topics.size());
  for (const std::size_t index : demanded_topics) {
    if (std::find(unhealthy_topics.begin(), unhealthy_topics.end(), index) == unhealthy_topics.end()) {
      healthy_topics.push_back(index);
    }
  }

  const std::string issue_message = buildHealthMessage(unhealthy_topics, healthy_topics);
  if (!healthy_topics.empty()) {
    resetFullOutageCounters();
    if (shouldLogIssue("warn", issue_message)) {
      RCLCPP_WARN(get_logger(), "%s", issue_message.c_str());
    }
    return;
  }

  reportFullOutage(issue_message);
}

void DepthCameraNode::enterFatalState(const std::string & message)
{
  fatal_error_ = true;
  if (shouldLogIssue("fatal", message)) {
    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
  }
  if (!watchdog_shutdown_on_fatal_) {
    RCLCPP_WARN(
      get_logger(),
      "Watchdog fatal shutdown is paused by parameter; keeping depth_camera_node alive for inspection.");
    return;
  }
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
  }
  rclcpp::shutdown();
}

void DepthCameraNode::reportFullOutage(const std::string & message)
{
  ++full_outage_count_;
  ++full_outage_attempt_count_;

  if (max_reconnect_attempts_ > 0 && full_outage_attempt_count_ >= max_reconnect_attempts_) {
    enterFatalState(
      message + ". Reached reconnect limit after " + std::to_string(full_outage_attempt_count_) +
      " attempts");
    return;
  }

  logEscalatingFullOutage(full_outage_count_, message);
}

void DepthCameraNode::logEscalatingFullOutage(int count, const std::string & message)
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

  enterFatalState(
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive full outages");
}

void DepthCameraNode::markRecoveredIfHealthy(const char * recovery_message)
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

void DepthCameraNode::resetFullOutageCounters()
{
  full_outage_attempt_count_ = 0;
  full_outage_count_ = 0;
  fatal_error_ = false;
  last_issue_log_level_.clear();
  last_issue_message_.clear();
}

bool DepthCameraNode::shouldLogIssue(const std::string & level, const std::string & message)
{
  if (last_issue_log_level_ == level && last_issue_message_ == message) {
    return false;
  }
  last_issue_log_level_ = level;
  last_issue_message_ = message;
  return true;
}

template<typename ServiceT>
void DepthCameraNode::bridgeService(const std::string & suffix, bool enabled)
{
  if (!enabled) {
    return;
  }

  const std::string source_camera_root = joinName("/", source_camera_id_);
  domain_bridge::ServiceBridgeOptions options;
  options.remap_name(joinName(target_namespace_root_, suffix));
  bridge_.bridge_service<ServiceT>(
    joinName(source_camera_root, suffix), source_domain_id_, target_domain_id_, options);
}

}  // namespace amr_sweeper_depth_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amr_sweeper_depth_camera::DepthCameraNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  node->bridge().add_to_executor(executor);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
