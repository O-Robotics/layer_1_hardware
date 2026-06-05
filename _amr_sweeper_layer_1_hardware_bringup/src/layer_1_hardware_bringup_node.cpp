#include "layer_1_hardware_bringup_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cmath>
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
  "ros2_control",
  "system_info",
  "battery",
  "gnss",
  "imu",
  "usb_cameras",
  "depth_camera",
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

std::string logger_name_for_namespace_and_node(
  const std::string &,
  const std::string & node_name)
{
  return node_name;
}

std::string wrap_plain_stdout_lines_as_info_logs(
  const std::string & command,
  const std::string & logger_name)
{
  std::ostringstream wrapped;
  wrapped
    << "set -o pipefail; "
    << command
    << " 2>&1 | while IFS= read -r line; do "
    << "case \"$line\" in "
    << "translation:*|rotation:*|from\\ *) "
    << "printf '[INFO] [%s] [" << logger_name << "] : %s\\n' \"$(date +%s.%N)\" \"$line\" ;; "
    << "*) printf '%s\\n' \"$line\" ;; "
    << "esac; "
    << "done";
  return wrapped.str();
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
    ::execl("/bin/bash", "bash", "-lc", command.c_str(), (char *)nullptr);
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
  declare_if_missing("controller_manager_query_timeout_ms", 3000);

  declare_if_missing("use_amr_sweeper_description", true);
  declare_if_missing("use_amr_sweeper_ros2_control", true);
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
    stage.commands = build_stage_commands(stage_name);
    stage.timeout_sec = stage_node["timeout_sec"] ? stage_node["timeout_sec"].as<double>() : 30.0;
    stage.readiness_rules = load_stage_rules(stage_node);
    if (!stage.commands.empty()) {
      stages_.push_back(stage);
    }
  }
}

void Layer1HardwareBringupNode::on_timer()
{
  if (bringup_complete_ || bringup_failed_) {
    return;
  }

  if (current_stage_index_ >= stages_.size()) {
    bringup_complete_ = true;
    RCLCPP_INFO(get_logger(), "Layer 1 bringup complete");
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
  bool all_ready = true;
  for (const auto & rule : stage.readiness_rules) {
    if (!rule.required || !rule_is_enabled(rule)) {
      continue;
    }
    if (!rule_is_satisfied(rule, missing)) {
      all_ready = false;
    }
  }
  return all_ready;
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
    ensure_topic_subscription(fq_target, rule.durability == "transient_local");
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
      missing.push_back("controller active " + rule.target);
      return false;
    }
    return true;
  }

  if (rule.type == "hardware") {
    if (!hardware_component_active(rule.target, parse_lifecycle_level(rule.state))) {
      missing.push_back("hardware " + rule.target + " state " + rule.state);
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
  std::vector<std::string> & missing)
{
  bool all_running = true;
  for (const auto & command : stage.commands) {
    if (!procman_.is_running(command)) {
      missing.push_back("process exited: " + command);
      all_running = false;
    }
  }
  return all_running;
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
  const auto rc = exec.spin_until_future_complete(
    future,
    std::chrono::milliseconds(param_as_int("controller_manager_query_timeout_ms")));
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
  const auto rc = exec.spin_until_future_complete(
    future,
    std::chrono::milliseconds(param_as_int("controller_manager_query_timeout_ms")));
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
  return current_stage_started_;
}

void Layer1HardwareBringupNode::start_current_stage()
{
  const auto & stage = stages_[current_stage_index_];
  RCLCPP_INFO(get_logger(), "%s", blue("Layer 1 stage: " + stage.label).c_str());

  for (const auto & command : stage.commands) {
    std::string err;
    if (!procman_.start(command, err)) {
      fail_bringup("Failed to start stage '" + stage.label + "': " + err);
      return;
    }
  }

  stage_started_at_ = std::chrono::steady_clock::now();
  current_stage_started_ = true;
  stage_deadline_ =
    stage_started_at_ + std::chrono::milliseconds(static_cast<int64_t>(stage.timeout_sec * 1000.0));
}

void Layer1HardwareBringupNode::finish_current_stage()
{
  const auto & stage = stages_[current_stage_index_];
  RCLCPP_INFO(
    get_logger(),
    "Layer 1 ready: %s (checks=%zu)",
    stage.label.c_str(),
    stage.readiness_rules.size());
  current_stage_started_ = false;
  ++current_stage_index_;
}

void Layer1HardwareBringupNode::fail_bringup(const std::string & reason)
{
  if (bringup_failed_) {
    return;
  }
  bringup_failed_ = true;
  current_stage_started_ = false;
  RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  stop_all_processes();
  rclcpp::shutdown();
}

void Layer1HardwareBringupNode::stop_all_processes()
{
  procman_.stop_all();
}

void Layer1HardwareBringupNode::ensure_topic_subscription(
  const std::string & topic_name,
  bool transient_local)
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
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  if (transient_local) {
    qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  }
  topic_subscriptions_[topic_name] = create_generic_subscription(
    topic_name,
    it->second.front(),
    qos,
    callback);
}

std::vector<std::string> Layer1HardwareBringupNode::build_stage_commands(
  const std::string & stage_name) const
{
  const auto ns = param_as_string("namespace");
  const auto log_level = param_as_string("log_level");
  const auto ublox_log_level = param_as_string("ublox_log_level");

  auto build_ros2_run =
    [](const std::string & package_name,
      const std::string & executable_name,
      const std::vector<std::string> & tokens) {
      std::vector<std::string> command_tokens = {
        "exec", "ros2", "run", package_name, executable_name,
      };
      command_tokens.insert(command_tokens.end(), tokens.begin(), tokens.end());
      return shell_join(command_tokens);
    };
  auto build_ros2_run_with_env =
    [&build_ros2_run](const std::string & package_name,
      const std::string & executable_name,
      const std::vector<std::string> & tokens,
      const std::vector<std::pair<std::string, std::string>> & env_vars) {
      std::ostringstream command;
      command << "exec env";
      for (const auto & env_var : env_vars) {
        command << " " << shell_quote(env_var.first + "=" + env_var.second);
      }
      command << " " << build_ros2_run(package_name, executable_name, tokens).substr(5);
      return command.str();
    };
  auto add_ros_arg =
    [](std::vector<std::string> & tokens, const std::string & value) {
      if (!value.empty()) {
        tokens.push_back(value);
      }
    };
  auto add_param =
    [&add_ros_arg](std::vector<std::string> & tokens, const std::string & name, const std::string & value) {
      if (!value.empty()) {
        add_ros_arg(tokens, "-p");
        add_ros_arg(tokens, name + ":=" + value);
      }
    };
  auto add_remap =
    [&add_ros_arg](std::vector<std::string> & tokens, const std::string & from, const std::string & to) {
      if (!from.empty() && !to.empty()) {
        add_ros_arg(tokens, "-r");
        add_ros_arg(tokens, from + ":=" + to);
      }
    };
  auto add_params_file =
    [&add_ros_arg](std::vector<std::string> & tokens, const std::string & params_file) {
      if (!params_file.empty()) {
        add_ros_arg(tokens, "--params-file");
        add_ros_arg(tokens, params_file);
      }
    };
  auto bool_string = [](const bool value) {
      return value ? "true" : "false";
  };
  auto base_ros_args =
    [&](const std::string & node_namespace, const std::string & node_name, const std::string & level) {
      std::vector<std::string> tokens = {"--ros-args"};
      add_remap(tokens, "__ns", normalize_fqn(node_namespace));
      add_remap(tokens, "__node", node_name);
      if (!level.empty()) {
        add_ros_arg(tokens, "--log-level");
        add_ros_arg(tokens, level);
      }
      return tokens;
    };

  if (stage_name == "robot_description") {
    const auto description_share = package_share("amr_sweeper_description");
    const auto xacro_file = description_share / "urdf" / "robot" / "robot.urdf.xacro";
    std::ostringstream command;
    command
      << "exec ros2 run robot_state_publisher robot_state_publisher"
      << " --ros-args"
      << " -r " << shell_quote("__ns:=" + normalize_fqn(ns))
      << " -p " << shell_quote(
      std::string("use_sim_time:=") + bool_string(param_as_bool("use_sim_time")))
      << " -p robot_description:=\"$(xacro "
      << shell_quote(xacro_file.string())
      << " robot_namespace:=" << shell_quote(ns)
      << " use_ros2_control:=" << bool_string(param_as_bool("use_amr_sweeper_ros2_control"))
      << " enable_usb_cameras:=" << bool_string(param_as_bool("use_amr_sweeper_usb_cameras"))
      << " enable_gnss:=" << bool_string(param_as_bool("use_amr_sweeper_gnss"))
      << " enable_imu:=" << bool_string(param_as_bool("use_amr_sweeper_imu"))
      << " enable_depth_camera:=" << bool_string(param_as_bool("use_amr_sweeper_depth_camera"))
      << ")\"";
    return {command.str()};
  }
  if (stage_name == "system_info") {
    auto tokens = base_ros_args(ns + "/system_info", "system_info_node", "");
    add_params_file(tokens, param_as_string("system_info_params_file"));
    return {build_ros2_run("amr_sweeper_system_info", "system_info_node", tokens)};
  }
  if (stage_name == "battery") {
    auto tokens = base_ros_args(ns + "/battery", "battery_node", "");
    add_params_file(tokens, param_as_string("battery_params_file"));
    add_param(tokens, "can_interface", param_as_string("battery_can_interface"));
    return {build_ros2_run("amr_sweeper_battery", "battery_node", tokens)};
  }
  if (stage_name == "gnss") {
    std::vector<std::string> commands;
    auto gnss_tokens = base_ros_args(ns + "/gnss", "gnss_node", ublox_log_level);
    add_params_file(gnss_tokens, package_share("amr_sweeper_gnss").string() + "/config/amr_sweeper_gnss_ublox.yaml");
    add_param(gnss_tokens, "frame_id", param_as_string("gnss_frame_id"));
    commands.push_back(build_ros2_run("amr_sweeper_gnss", "gnss_node", gnss_tokens));

    if (param_as_bool("use_ntrip_client")) {
      auto ntrip_tokens = base_ros_args(ns + "/gnss", "ntrip_client", log_level);
      add_params_file(ntrip_tokens, param_as_string("ntrip_params_file"));
      add_param(ntrip_tokens, "send_nmea", bool_string(true));
      add_remap(ntrip_tokens, "fix", "navsat");
      commands.push_back(build_ros2_run("amr_sweeper_gnss", "ntrip_client", ntrip_tokens));
    }
    return commands;
  }
  if (stage_name == "imu") {
    auto tokens = base_ros_args(ns + "/imu", "imu_node", "");
    add_params_file(tokens, param_as_string("imu_params_file"));
    add_param(tokens, "use_sim_time", bool_string(param_as_bool("use_sim_time")));
    add_param(tokens, "device_path", param_as_string("imu_device_path"));
    add_param(tokens, "port", param_as_string("imu_port"));
    add_param(tokens, "baud", param_as_string("imu_baud"));
    return {build_ros2_run("amr_sweeper_imu", "imu_node", tokens)};
  }
  if (stage_name == "usb_cameras") {
    std::vector<std::string> commands;
    const auto usb_cameras_share = package_share("amr_sweeper_usb_cameras");
    const std::vector<std::pair<std::string, bool>> cameras = {
      {"front_left_camera", param_as_bool("front_left_camera_enabled")},
      {"front_right_camera", param_as_bool("front_right_camera_enabled")},
      {"rear_left_camera", param_as_bool("rear_left_camera_enabled")},
      {"rear_right_camera", param_as_bool("rear_right_camera_enabled")},
      {"tools_camera", param_as_bool("tools_camera_enabled")},
    };
    for (const auto & camera : cameras) {
      if (!camera.second) {
        continue;
      }
      auto tokens = base_ros_args(ns + "/usb_cameras", camera.first + "_node", log_level);
      add_params_file(tokens, (usb_cameras_share / "config" / (camera.first + "_params.yaml")).string());
      commands.push_back(build_ros2_run("amr_sweeper_usb_cameras", "usb_cameras_node", tokens));
    }
    return commands;
  }
  if (stage_name == "depth_camera") {
    std::vector<std::string> commands;
    const auto namespace_value = normalize_fqn(ns + "/depth_camera");
    const auto depth_camera_parent_namespace =
      namespace_value.substr(0, namespace_value.find_last_of('/'));
    const auto depth_camera_name = namespace_value.substr(namespace_value.find_last_of('/') + 1);
    const auto laserscan_output_frame = param_as_string("depth_camera_output_frame").empty() ?
      std::string("laserscan_link") : param_as_string("depth_camera_output_frame");
    const auto scan_tilt_angle_deg = param_as_string("depth_camera_scan_tilt_angle_deg").empty() ?
      std::string("4.5") : param_as_string("depth_camera_scan_tilt_angle_deg");
    auto realsense_tokens = base_ros_args(depth_camera_parent_namespace, depth_camera_name, log_level);
    add_params_file(realsense_tokens, param_as_string("depth_camera_params_file"));
    add_param(realsense_tokens, "camera_name", depth_camera_name);
    add_param(realsense_tokens, "use_sim_time", bool_string(param_as_bool("use_sim_time")));
    commands.push_back(
      build_ros2_run_with_env(
        "realsense2_camera",
        "realsense2_camera_node",
        realsense_tokens,
        {{"LRS_LOG_LEVEL", "FATAL"}}));

    if (param_as_bool("depth_camera_use_laserscan")) {
      auto laserscan_tokens = base_ros_args(ns + "/depth_camera", "laserscan", log_level);
      add_params_file(laserscan_tokens, param_as_string("depth_camera_laserscan_params_file"));
      add_param(laserscan_tokens, "use_sim_time", bool_string(param_as_bool("use_sim_time")));
      add_param(laserscan_tokens, "output_frame", laserscan_output_frame);
      add_param(laserscan_tokens, "range_min", param_as_string("depth_camera_range_min"));
      add_param(laserscan_tokens, "range_max", param_as_string("depth_camera_range_max"));
      add_param(laserscan_tokens, "scan_height", param_as_string("depth_camera_scan_height"));
      add_param(laserscan_tokens, "scan_tilt_angle_deg", scan_tilt_angle_deg);
      add_param(laserscan_tokens, "scan_time", param_as_string("depth_camera_scan_time"));
      add_remap(
        laserscan_tokens,
        "depth",
        param_as_string("depth_camera_image_topic").empty() ?
        normalize_fqn(ns + "/depth_camera/depth/image_rect_raw") : param_as_string("depth_camera_image_topic"));
      add_remap(
        laserscan_tokens,
        "depth_camera_info",
        param_as_string("depth_camera_info_topic").empty() ?
        normalize_fqn(ns + "/depth_camera/depth/camera_info") : param_as_string("depth_camera_info_topic"));
      add_remap(
        laserscan_tokens,
        "scan",
        param_as_string("depth_camera_scan_topic").empty() ? "scan" : param_as_string("depth_camera_scan_topic"));
      commands.push_back(build_ros2_run("amr_sweeper_depth_camera", "laserscan_node", laserscan_tokens));

      double scan_tilt_angle_rad = 0.0;
      try {
        scan_tilt_angle_rad = std::stod(scan_tilt_angle_deg) * M_PI / 180.0;
      } catch (const std::exception & exception) {
        throw std::runtime_error(
                "Invalid depth_camera_scan_tilt_angle_deg value '" + scan_tilt_angle_deg +
                "': " + exception.what());
      }
      std::vector<std::string> tf_tokens = {
        "--x", "0",
        "--y", "0",
        "--z", "0",
        "--roll", "0",
        "--pitch", std::to_string(scan_tilt_angle_rad),
        "--yaw", "0",
        "--frame-id", param_as_string("depth_camera_frame"),
        "--child-frame-id", laserscan_output_frame,
        "--ros-args",
        "-r", "__ns:=" + normalize_fqn(ns + "/depth_camera"),
        "-r", "__node:=laserscan_tf",
      };
      const std::string tf_command =
        build_ros2_run("tf2_ros", "static_transform_publisher", tf_tokens);
      commands.push_back(
        wrap_plain_stdout_lines_as_info_logs(
          tf_command,
          logger_name_for_namespace_and_node(normalize_fqn(ns + "/depth_camera"), "laserscan_tf")));
    }
    return commands;
  }
  if (stage_name == "ros2_control") {
    std::vector<std::string> commands;
    auto manager_tokens = base_ros_args(ns, "controller_manager", "");
    add_param(manager_tokens, "use_sim_time", bool_string(param_as_bool("use_sim_time")));
    add_params_file(
      manager_tokens,
      (package_share("amr_sweeper_description") / "urdf" / "control" / "ros2_control.yaml").string());
    add_remap(manager_tokens, "/robot_description", normalize_fqn(ns + "/robot_description"));
    commands.push_back(build_ros2_run("controller_manager", "ros2_control_node", manager_tokens));

    std::vector<std::string> spawner_tokens = {
      "joint_broad",
      "--controller-manager",
      normalize_fqn(ns + "/controller_manager"),
      "--controller-manager-timeout",
      "60",
      "--ros-args",
      "-r", "__ns:=" + normalize_fqn(ns),
    };
    const std::string controller_manager_service =
      normalize_fqn(ns + "/controller_manager/list_controllers");
    std::ostringstream spawner_command;
    spawner_command
      << "until ros2 service type "
      << shell_quote(controller_manager_service)
      << " >/dev/null 2>&1; do sleep 0.2; done; "
      << build_ros2_run("controller_manager", "spawner", spawner_tokens);
    commands.push_back(spawner_command.str());
    return commands;
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
    rule.durability =
      rule_node["durability"] ? trim(rule_node["durability"].as<std::string>()) : "";
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

std::string Layer1HardwareBringupNode::shell_join(const std::vector<std::string> & tokens)
{
  std::ostringstream oss;
  for (std::size_t index = 0; index < tokens.size(); ++index) {
    if (index > 0) {
      oss << " ";
    }
    if (index == 0 && tokens[index] == "exec") {
      oss << "exec";
      continue;
    }
    oss << shell_quote(tokens[index]);
  }
  return oss.str();
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

int Layer1HardwareBringupNode::param_as_int(const std::string & name) const
{
  return get_parameter(name).as_int();
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

std::filesystem::path Layer1HardwareBringupNode::package_share(const std::string & package_name) const
{
  return std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name));
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
