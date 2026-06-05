#include "layer_1_hardware_bringup_node.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>

namespace amr_sweeper_layer_1_hardware_bringup
{

namespace
{

const std::vector<std::string> kStageOrder = {
  "robot_description",
  "system_info",
  "battery",
  "gnss",
  "imu",
  "usb_cameras",
  "depth_camera",
  "ros2_control",
};

std::string trim(const std::string & value)
{
  std::size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
    ++start;
  }
  std::size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(start, end - start);
}

constexpr const char * kConsoleOutputFormat = "[{severity}] [{time}] [{name}] : {message}";

}  // namespace

ProcessManager::~ProcessManager()
{
  stop_all();
}

bool ProcessManager::pid_alive(pid_t pid)
{
  if (pid <= 0) {
    return false;
  }
  if (::kill(pid, 0) == 0) {
    return true;
  }
  return errno == EPERM;
}

bool ProcessManager::wait_dead(pid_t pid, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    const pid_t result = ::waitpid(pid, &status, WNOHANG);
    if (result == pid) {
      return true;
    }
    if (!pid_alive(pid)) {
      return true;
    }
    ::usleep(20 * 1000);
  }
  return !pid_alive(pid);
}

bool ProcessManager::start(const std::string & command, std::string & err_out)
{
  err_out.clear();
  if (command.empty()) {
    err_out = "Empty command";
    return false;
  }
  if (is_running(command)) {
    return true;
  }

  const pid_t pid = ::fork();
  if (pid < 0) {
    err_out = std::string("fork() failed: ") + std::strerror(errno);
    return false;
  }
  if (pid == 0) {
    ::setpgid(0, 0);
    const std::filesystem::path ros_log_dir =
      std::filesystem::temp_directory_path() /
      ("amr_sweeper_layer_1_roslog_" + std::to_string(::getpid()));
    std::error_code ec;
    std::filesystem::create_directories(ros_log_dir, ec);
    ::setenv("ROS_LOG_DIR", ros_log_dir.c_str(), 1);
    ::setenv("RCUTILS_COLORIZED_OUTPUT", "1", 1);
    ::setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", kConsoleOutputFormat, 1);
    ::setenv("RMW_FASTRTPS_USE_SHM", "0", 1);
    ::execl("/bin/sh", "sh", "-c", command.c_str(), (char *)nullptr);
    _exit(127);
  }

  ::setpgid(pid, pid);
  Proc proc;
  proc.pid = pid;
  proc.command = command;
  proc.started_at = std::chrono::steady_clock::now();
  procs_[command] = proc;
  return true;
}

bool ProcessManager::stop(const std::string & command, std::string & err_out)
{
  return stop(command, err_out, StopPolicy{});
}

bool ProcessManager::stop(
  const std::string & command,
  std::string & err_out,
  const StopPolicy & policy)
{
  err_out.clear();
  const auto it = procs_.find(command);
  if (it == procs_.end()) {
    return true;
  }

  const pid_t pid = it->second.pid;
  if (pid_alive(pid)) {
    ::kill(-pid, SIGTERM);
    if (!wait_dead(pid, policy.sigterm_timeout)) {
      ::kill(-pid, SIGINT);
      if (!wait_dead(pid, policy.sigint_timeout)) {
        ::kill(-pid, SIGKILL);
        (void)wait_dead(pid, policy.sigkill_timeout);
      }
    }
  }

  int status = 0;
  (void)::waitpid(pid, &status, WNOHANG);
  procs_.erase(it);
  return true;
}

void ProcessManager::stop_all()
{
  stop_all(StopPolicy{});
}

void ProcessManager::stop_all(const StopPolicy & policy)
{
  std::vector<std::string> commands;
  commands.reserve(procs_.size());
  for (const auto & entry : procs_) {
    commands.push_back(entry.first);
  }
  for (const auto & command : commands) {
    std::string err;
    (void)stop(command, err, policy);
  }
}

bool ProcessManager::is_running(const std::string & command) const
{
  const auto it = procs_.find(command);
  if (it == procs_.end()) {
    return false;
  }
  return pid_alive(it->second.pid);
}

std::vector<ProcessManager::Proc> ProcessManager::list() const
{
  std::vector<Proc> out;
  out.reserve(procs_.size());
  for (const auto & entry : procs_) {
    out.push_back(entry.second);
  }
  return out;
}

Layer1HardwareBringupNode::Layer1HardwareBringupNode()
: Node("layer_1_hardware_bringup_node")
{
  ::setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", kConsoleOutputFormat, 1);
  declare_parameters();
  build_stages();
  timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&Layer1HardwareBringupNode::on_timer, this));
}

Layer1HardwareBringupNode::~Layer1HardwareBringupNode()
{
  stop_all_processes();
}

void Layer1HardwareBringupNode::declare_parameters()
{
  auto declare_if_missing =
    [this](const std::string & name, const auto & default_value) {
      if (!has_parameter(name)) {
        declare_parameter(name, default_value);
      }
    };

  declare_if_missing("namespace", std::string("amr_sweeper"));
  declare_if_missing("log_level", std::string("info"));
  declare_if_missing("ublox_log_level", std::string("WARN"));
  declare_if_missing("use_sim_time", false);
  declare_if_missing("readiness_config_file", std::string(""));

  declare_if_missing("use_amr_sweeper_description", true);
  declare_if_missing("use_amr_sweeper_ros2_control", true);
  declare_if_missing("use_joint_broadcaster", true);
  declare_if_missing("use_amr_sweeper_battery", true);
  declare_if_missing("use_amr_sweeper_system_info", true);
  declare_if_missing("use_amr_sweeper_usb_cameras", true);
  declare_if_missing("use_amr_sweeper_depth_camera", true);
  declare_if_missing("use_amr_sweeper_imu", true);
  declare_if_missing("use_amr_sweeper_gnss", true);
  declare_if_missing("use_ntrip_client", true);

  declare_if_missing("battery_can_interface", std::string("can0"));
  declare_if_missing("battery_params_file", std::string(""));
  declare_if_missing("system_info_params_file", std::string(""));
  declare_if_missing("depth_camera_params_file", std::string(""));
  declare_if_missing("depth_camera_use_laserscan", true);
  declare_if_missing("depth_camera_laserscan_params_file", std::string(""));
  declare_if_missing("depth_camera_image_topic", std::string(""));
  declare_if_missing("depth_camera_info_topic", std::string(""));
  declare_if_missing("depth_camera_frame", std::string("depth_camera_link"));
  declare_if_missing("depth_camera_scan_topic", std::string("scan"));
  declare_if_missing("depth_camera_output_frame", std::string(""));
  declare_if_missing("depth_camera_range_min", std::string(""));
  declare_if_missing("depth_camera_range_max", std::string(""));
  declare_if_missing("depth_camera_scan_height", std::string(""));
  declare_if_missing("depth_camera_scan_tilt_angle_deg", std::string(""));
  declare_if_missing("depth_camera_scan_time", std::string(""));
  declare_if_missing("depth_camera_camera_domain_id", 5);
  declare_if_missing("imu_device_path", std::string(""));
  declare_if_missing("imu_port", std::string(""));
  declare_if_missing("imu_baud", std::string(""));
  declare_if_missing("imu_params_file", std::string(""));
  declare_if_missing("gnss_frame_id", std::string("gnss_link"));
  declare_if_missing("ntrip_params_file", std::string(""));
  declare_if_missing("front_left_camera_enabled", false);
  declare_if_missing("front_right_camera_enabled", false);
  declare_if_missing("rear_left_camera_enabled", true);
  declare_if_missing("rear_right_camera_enabled", true);
  declare_if_missing("tools_camera_enabled", true);
}

void Layer1HardwareBringupNode::build_stages()
{
  const std::string readiness_config_file = param_as_string("readiness_config_file");
  if (readiness_config_file.empty()) {
    throw std::runtime_error("readiness_config_file parameter must not be empty");
  }

  const YAML::Node root = YAML::LoadFile(readiness_config_file);
  const YAML::Node stages_node = root["stages"];
  if (!stages_node || !stages_node.IsMap()) {
    throw std::runtime_error("Missing or invalid 'stages' map in readiness config");
  }

  for (const auto & stage_name : kStageOrder) {
    if (stage_name == "robot_description" && !param_as_bool("use_amr_sweeper_description")) {
      continue;
    }
    if (stage_name == "system_info" && !param_as_bool("use_amr_sweeper_system_info")) {
      continue;
    }
    if (stage_name == "battery" && !param_as_bool("use_amr_sweeper_battery")) {
      continue;
    }
    if (stage_name == "gnss" && !param_as_bool("use_amr_sweeper_gnss")) {
      continue;
    }
    if (stage_name == "imu" && !param_as_bool("use_amr_sweeper_imu")) {
      continue;
    }
    if (stage_name == "usb_cameras" && !param_as_bool("use_amr_sweeper_usb_cameras")) {
      continue;
    }
    if (stage_name == "depth_camera" && !param_as_bool("use_amr_sweeper_depth_camera")) {
      continue;
    }
    if (stage_name == "ros2_control" && !param_as_bool("use_amr_sweeper_ros2_control")) {
      continue;
    }

    const YAML::Node stage_node = stages_node[stage_name];
    if (!stage_node) {
      continue;
    }

    StageSpec stage;
    stage.label = stage_name;
    stage.command = build_stage_command(stage_name);
    stage.timeout_sec = stage_node["timeout_sec"] ? stage_node["timeout_sec"].as<double>() : 30.0;
    stage.readiness_rules = load_stage_rules(stage_node);
    stages_.push_back(stage);
  }
}

void Layer1HardwareBringupNode::on_timer()
{
  if (bringup_complete_ || bringup_failed_) {
    return;
  }

  if (current_stage_index_ >= stages_.size()) {
    bringup_complete_ = true;
    RCLCPP_INFO(get_logger(), "%s", blue("Layer 1 bringup complete").c_str());
    return;
  }

  if (!stage_has_started()) {
    start_current_stage();
    return;
  }

  const auto & stage = stages_[current_stage_index_];
  std::vector<std::string> missing;
  if (!stage_process_running(stage, missing)) {
    fail_bringup("Stage '" + stage.label + "' exited before readiness checks passed");
    return;
  }

  if (stage_ready(stage, missing)) {
    finish_current_stage();
    return;
  }

  if (std::chrono::steady_clock::now() > stage_deadline_) {
    std::ostringstream oss;
    oss << "Layer 1 stage '" << stage.label << "' timed out after " << stage.timeout_sec
        << "s; missing: ";
    for (std::size_t index = 0; index < missing.size(); ++index) {
      if (index > 0) {
        oss << ", ";
      }
      oss << missing[index];
    }
    fail_bringup(oss.str());
  }
}

bool Layer1HardwareBringupNode::stage_ready(
  const StageSpec & stage,
  std::vector<std::string> & missing)
{
  missing.clear();
  for (const auto & rule : stage.readiness_rules) {
    if (!rule.required || !rule_is_enabled(rule)) {
      continue;
    }
    if (!rule_is_satisfied(rule, missing)) {
      return false;
    }
  }
  return true;
}

bool Layer1HardwareBringupNode::rule_is_enabled(const ReadinessRule & rule) const
{
  if (!rule.when_arg_true.empty() && !param_as_bool(rule.when_arg_true)) {
    return false;
  }
  if (!rule.when_arg_false.empty() && param_as_bool(rule.when_arg_false)) {
    return false;
  }
  return true;
}

bool Layer1HardwareBringupNode::rule_is_satisfied(
  const ReadinessRule & rule,
  std::vector<std::string> & missing)
{
  const std::string fq_target = qualify_to_ns(rule.target);
  if (rule.type == "topic") {
    const auto topics = topic_types();
    const auto it = topics.find(fq_target);
    if (it == topics.end() || it->second.empty()) {
      missing.push_back("topic " + fq_target);
      return false;
    }
    ensure_topic_subscription(fq_target);
    if (ready_topics_.count(fq_target) == 0U) {
      missing.push_back("topic data " + fq_target);
      return false;
    }
    return true;
  }

  if (rule.type == "service") {
    const auto services = service_types();
    if (services.find(fq_target) == services.end()) {
      missing.push_back("service " + fq_target);
      return false;
    }
    return true;
  }

  if (rule.type == "controller") {
    if (!controller_is_active(rule.target)) {
      missing.push_back("controller " + rule.target);
      return false;
    }
    return true;
  }

  if (rule.type == "hardware") {
    if (!hardware_component_active(rule.target, parse_lifecycle_level(rule.state))) {
      missing.push_back("hardware " + rule.target);
      return false;
    }
    return true;
  }

  if (rule.type == "node") {
    for (const auto & pair : get_node_graph_interface()->get_node_names_and_namespaces()) {
      const std::string fqn =
        pair.second == "/" ? normalize_fqn(pair.first) : normalize_fqn(pair.second + "/" + pair.first);
      if (fqn == fq_target) {
        return true;
      }
    }
    missing.push_back("node " + fq_target);
    return false;
  }

  return true;
}

bool Layer1HardwareBringupNode::stage_process_running(
  const StageSpec & stage,
  std::vector<std::string> &)
{
  return procman_.is_running(stage.command);
}

bool Layer1HardwareBringupNode::controller_is_active(const std::string & controller_name)
{
  auto probe = std::make_shared<rclcpp::Node>(
    "layer_1_controller_probe",
    rclcpp::NodeOptions()
    .context(get_node_base_interface()->get_context())
    .use_global_arguments(false)
    .enable_rosout(false));
  auto client = probe->create_client<controller_manager_msgs::srv::ListControllers>(
    qualify_to_ns("controller_manager/list_controllers"));
  if (!client->wait_for_service(std::chrono::milliseconds(0))) {
    return false;
  }

  auto future = client->async_send_request(
    std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(probe);
  const auto rc = exec.spin_until_future_complete(future, std::chrono::seconds(1));
  exec.remove_node(probe);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  const auto response = future.get();
  if (!response) {
    return false;
  }
  for (const auto & controller : response->controller) {
    if (controller.name == controller_name && controller.state == "active") {
      return true;
    }
  }
  return false;
}

bool Layer1HardwareBringupNode::hardware_component_active(
  const std::string & component_name,
  uint8_t expected_state)
{
  auto probe = std::make_shared<rclcpp::Node>(
    "layer_1_hardware_probe",
    rclcpp::NodeOptions()
    .context(get_node_base_interface()->get_context())
    .use_global_arguments(false)
    .enable_rosout(false));
  auto client = probe->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
    qualify_to_ns("controller_manager/list_hardware_components"));
  if (!client->wait_for_service(std::chrono::milliseconds(0))) {
    return false;
  }

  auto future = client->async_send_request(
    std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(probe);
  const auto rc = exec.spin_until_future_complete(future, std::chrono::seconds(1));
  exec.remove_node(probe);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  const auto response = future.get();
  if (!response) {
    return false;
  }
  for (const auto & component : response->component) {
    if (component.name == component_name && component.state.id == expected_state) {
      return true;
    }
  }
  return false;
}

bool Layer1HardwareBringupNode::stage_has_started() const
{
  if (current_stage_index_ >= stages_.size()) {
    return false;
  }
  return procman_.is_running(stages_[current_stage_index_].command);
}

void Layer1HardwareBringupNode::start_current_stage()
{
  const auto & stage = stages_[current_stage_index_];
  RCLCPP_INFO(get_logger(), "%s", blue("Layer 1 stage: " + stage.label).c_str());

  std::string err;
  if (!procman_.start(stage.command, err)) {
    fail_bringup("Failed to start stage '" + stage.label + "': " + err);
    return;
  }

  stage_started_at_ = std::chrono::steady_clock::now();
  stage_deadline_ =
    stage_started_at_ + std::chrono::milliseconds(static_cast<int64_t>(stage.timeout_sec * 1000.0));
}

void Layer1HardwareBringupNode::finish_current_stage()
{
  const auto & stage = stages_[current_stage_index_];
  RCLCPP_INFO(
    get_logger(),
    "%s",
    blue("Layer 1 ready: " + stage.label + " (checks=" +
      std::to_string(stage.readiness_rules.size()) + ")").c_str());
  ++current_stage_index_;
}

void Layer1HardwareBringupNode::fail_bringup(const std::string & reason)
{
  if (bringup_failed_) {
    return;
  }
  bringup_failed_ = true;
  RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  stop_all_processes();
  rclcpp::shutdown();
}

void Layer1HardwareBringupNode::stop_all_processes()
{
  procman_.stop_all();
}

void Layer1HardwareBringupNode::ensure_topic_subscription(const std::string & topic_name)
{
  if (topic_subscriptions_.find(topic_name) != topic_subscriptions_.end()) {
    return;
  }
  const auto topics = topic_types();
  const auto it = topics.find(topic_name);
  if (it == topics.end() || it->second.empty()) {
    return;
  }

  const auto callback =
    [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage>) {
      ready_topics_.insert(topic_name);
    };
  topic_subscriptions_[topic_name] = create_generic_subscription(
    topic_name,
    it->second.front(),
    rclcpp::SensorDataQoS(),
    callback);
}

std::string Layer1HardwareBringupNode::build_stage_command(const std::string & stage_name) const
{
  const auto ns = param_as_string("namespace");
  const auto log_level = param_as_string("log_level");
  const auto ublox_log_level = param_as_string("ublox_log_level");

  std::vector<std::string> args;
  auto add_arg = [&args](const std::string & name, const std::string & value) {
    if (value.empty()) {
      return;
    }
    args.push_back(name + ":=" + value);
  };
  auto build_ros2_launch = [&args](const std::string & package_name, const std::string & launch_file) {
    std::ostringstream oss;
    oss << "ros2 launch " << package_name << " " << launch_file;
    for (const auto & arg : args) {
      oss << " " << shell_quote(arg);
    }
    return oss.str();
  };

  if (stage_name == "robot_description") {
    add_arg("namespace", ns);
    add_arg("use_sim_time", param_as_bool("use_sim_time") ? "true" : "false");
    add_arg("use_ros2_control", param_as_bool("use_amr_sweeper_ros2_control") ? "true" : "false");
    add_arg("enable_usb_cameras", param_as_bool("use_amr_sweeper_usb_cameras") ? "true" : "false");
    add_arg("enable_gnss", param_as_bool("use_amr_sweeper_gnss") ? "true" : "false");
    add_arg("enable_imu", param_as_bool("use_amr_sweeper_imu") ? "true" : "false");
    add_arg("enable_depth_camera", param_as_bool("use_amr_sweeper_depth_camera") ? "true" : "false");
    return build_ros2_launch("amr_sweeper_description", "amr_sweeper_description.launch.py");
  }
  if (stage_name == "system_info") {
    add_arg("namespace", ns + "/system_info");
    add_arg("params_file", param_as_string("system_info_params_file"));
    return build_ros2_launch("amr_sweeper_system_info", "amr_sweeper_system_info.launch.py");
  }
  if (stage_name == "battery") {
    add_arg("namespace", ns + "/battery");
    add_arg("can_interface", param_as_string("battery_can_interface"));
    add_arg("params_file", param_as_string("battery_params_file"));
    return build_ros2_launch("amr_sweeper_battery", "amr_sweeper_battery.launch.py");
  }
  if (stage_name == "gnss") {
    add_arg("use_ublox_dgnss_node", param_as_bool("use_amr_sweeper_gnss") ? "true" : "false");
    add_arg("use_ublox_nav_sat_fix_hp", param_as_bool("use_amr_sweeper_gnss") ? "true" : "false");
    add_arg("use_ntrip_client", param_as_bool("use_ntrip_client") ? "true" : "false");
    add_arg("gnss_namespace", ns + "/gnss");
    add_arg("gnss_frame_id", param_as_string("gnss_frame_id"));
    add_arg("ntrip_params_file", param_as_string("ntrip_params_file"));
    add_arg("log_level", log_level);
    add_arg("ublox_log_level", ublox_log_level);
    return build_ros2_launch("amr_sweeper_gnss", "amr_sweeper_gnss.launch.py");
  }
  if (stage_name == "imu") {
    add_arg("namespace", ns + "/imu");
    add_arg("use_sim_time", param_as_bool("use_sim_time") ? "true" : "false");
    add_arg("params_file", param_as_string("imu_params_file"));
    add_arg("device_path", param_as_string("imu_device_path"));
    add_arg("port", param_as_string("imu_port"));
    add_arg("baud", param_as_string("imu_baud"));
    add_arg("use_imu_node", param_as_bool("use_amr_sweeper_imu") ? "true" : "false");
    return build_ros2_launch("amr_sweeper_imu", "amr_sweeper_imu.launch.py");
  }
  if (stage_name == "usb_cameras") {
    add_arg("namespace", ns + "/usb_cameras");
    add_arg("log_level", log_level);
    add_arg("front_left_camera_enabled", param_as_bool("front_left_camera_enabled") ? "true" : "false");
    add_arg("front_right_camera_enabled", param_as_bool("front_right_camera_enabled") ? "true" : "false");
    add_arg("rear_left_camera_enabled", param_as_bool("rear_left_camera_enabled") ? "true" : "false");
    add_arg("rear_right_camera_enabled", param_as_bool("rear_right_camera_enabled") ? "true" : "false");
    add_arg("tools_camera_enabled", param_as_bool("tools_camera_enabled") ? "true" : "false");
    return build_ros2_launch("amr_sweeper_usb_cameras", "amr_sweeper_usb_cameras.launch.py");
  }
  if (stage_name == "depth_camera") {
    add_arg("namespace", ns + "/depth_camera");
    add_arg("log_level", log_level);
    add_arg("use_sim_time", param_as_bool("use_sim_time") ? "true" : "false");
    add_arg("camera_domain_id", param_as_string("depth_camera_camera_domain_id"));
    add_arg("params_file", param_as_string("depth_camera_params_file"));
    add_arg("use_laserscan", param_as_bool("depth_camera_use_laserscan") ? "true" : "false");
    add_arg("laserscan_params_file", param_as_string("depth_camera_laserscan_params_file"));
    add_arg("depth_image_topic", param_as_string("depth_camera_image_topic"));
    add_arg("depth_camera_info_topic", param_as_string("depth_camera_info_topic"));
    add_arg("depth_camera_frame", param_as_string("depth_camera_frame"));
    add_arg("scan_topic", param_as_string("depth_camera_scan_topic"));
    add_arg("output_frame", param_as_string("depth_camera_output_frame"));
    add_arg("range_min", param_as_string("depth_camera_range_min"));
    add_arg("range_max", param_as_string("depth_camera_range_max"));
    add_arg("scan_height", param_as_string("depth_camera_scan_height"));
    add_arg("scan_tilt_angle_deg", param_as_string("depth_camera_scan_tilt_angle_deg"));
    add_arg("scan_time", param_as_string("depth_camera_scan_time"));
    return build_ros2_launch("amr_sweeper_depth_camera", "amr_sweeper_depth_camera.launch.py");
  }
  if (stage_name == "ros2_control") {
    add_arg("namespace", ns);
    add_arg("use_sim_time", param_as_bool("use_sim_time") ? "true" : "false");
    add_arg("use_ros2_control", param_as_bool("use_amr_sweeper_ros2_control") ? "true" : "false");
    add_arg("use_joint_broadcaster", param_as_bool("use_joint_broadcaster") ? "true" : "false");
    return build_ros2_launch("amr_sweeper_ros2_control", "amr_sweeper_ros2_control.launch.py");
  }

  throw std::runtime_error("Unsupported stage name: " + stage_name);
}

std::vector<ReadinessRule> Layer1HardwareBringupNode::load_stage_rules(const YAML::Node & stage_node) const
{
  std::vector<ReadinessRule> rules;
  const YAML::Node ready = stage_node["ready"];
  if (!ready || !ready.IsSequence()) {
    return rules;
  }
  for (const auto & rule_node : ready) {
    if (!rule_node || !rule_node.IsMap()) {
      continue;
    }
    ReadinessRule rule;
    rule.type = rule_node["type"] ? trim(rule_node["type"].as<std::string>()) : "";
    rule.target = rule_node["target"] ? trim(rule_node["target"].as<std::string>()) : "";
    rule.state = rule_node["state"] ? trim(rule_node["state"].as<std::string>()) : "active";
    rule.required = rule_node["required"] ? rule_node["required"].as<bool>() : true;
    rule.when_arg_true =
      rule_node["when_arg_true"] ? trim(rule_node["when_arg_true"].as<std::string>()) : "";
    rule.when_arg_false =
      rule_node["when_arg_false"] ? trim(rule_node["when_arg_false"].as<std::string>()) : "";
    rules.push_back(rule);
  }
  return rules;
}

std::string Layer1HardwareBringupNode::qualify_to_ns(const std::string & target) const
{
  if (target.empty()) {
    return target;
  }
  if (!target.empty() && target.front() == '/') {
    return normalize_fqn(target);
  }
  const auto ns = robot_namespace();
  if (ns == "/") {
    return normalize_fqn(target);
  }
  return normalize_fqn(ns + "/" + target);
}

std::string Layer1HardwareBringupNode::normalize_fqn(const std::string & name)
{
  std::string out = trim(name);
  if (out.empty()) {
    return out;
  }
  if (out.front() != '/') {
    out = "/" + out;
  }
  while (out.size() > 1 && out.back() == '/') {
    out.pop_back();
  }
  return out;
}

std::string Layer1HardwareBringupNode::robot_namespace() const
{
  const std::string raw = param_as_string("namespace");
  const std::string trimmed = trim(raw);
  if (trimmed.empty() || trimmed == "/") {
    return "/";
  }
  return normalize_fqn(trimmed);
}

std::string Layer1HardwareBringupNode::shell_quote(const std::string & value)
{
  if (value.empty()) {
    return "''";
  }
  std::string out = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out += "'";
  return out;
}

std::string Layer1HardwareBringupNode::blue(const std::string & text)
{
  return "\033[94m" + text + "\033[0m";
}

uint8_t Layer1HardwareBringupNode::parse_lifecycle_level(const std::string & raw)
{
  std::string normalized = raw;
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  if (normalized == "UNCONFIGURED" || normalized == "UNCONFIGURE" || normalized == "UNCONFIG") {
    return 1;
  }
  if (normalized == "INACTIVE" || normalized == "CONFIGURED") {
    return 2;
  }
  if (normalized == "FINALIZED" || normalized == "FINAL") {
    return 4;
  }
  return 3;
}

bool Layer1HardwareBringupNode::param_as_bool(const std::string & name) const
{
  return get_parameter(name).as_bool();
}

std::string Layer1HardwareBringupNode::param_as_string(const std::string & name) const
{
  const auto parameter = get_parameter(name);
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_STRING:
      return parameter.as_string();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return std::to_string(parameter.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return std::to_string(parameter.as_double());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return parameter.as_bool() ? "true" : "false";
    default:
      return parameter.value_to_string();
  }
}

std::map<std::string, std::vector<std::string>> Layer1HardwareBringupNode::topic_types() const
{
  std::map<std::string, std::vector<std::string>> out;
  for (const auto & item : get_topic_names_and_types()) {
    out.emplace(normalize_fqn(item.first), item.second);
  }
  return out;
}

std::map<std::string, std::vector<std::string>> Layer1HardwareBringupNode::service_types() const
{
  std::map<std::string, std::vector<std::string>> out;
  for (const auto & item : get_service_names_and_types()) {
    out.emplace(normalize_fqn(item.first), item.second);
  }
  return out;
}

}  // namespace amr_sweeper_layer_1_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<
      amr_sweeper_layer_1_hardware_bringup::Layer1HardwareBringupNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("layer_1_hardware_bringup_node"),
      "Unhandled exception: %s",
      exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
