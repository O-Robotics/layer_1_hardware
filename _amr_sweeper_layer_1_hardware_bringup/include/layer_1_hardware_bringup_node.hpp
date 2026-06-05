#pragma once

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <sys/types.h>
#include <string>
#include <vector>

namespace amr_sweeper_layer_1_hardware_bringup
{

class ProcessManager
{
public:
  struct Proc
  {
    pid_t pid{-1};
    std::string command;
    std::chrono::steady_clock::time_point started_at;
  };

  struct StopPolicy
  {
    std::chrono::milliseconds sigint_timeout{std::chrono::milliseconds(2000)};
    std::chrono::milliseconds sigterm_timeout{std::chrono::milliseconds(2000)};
    std::chrono::milliseconds sigkill_timeout{std::chrono::milliseconds(500)};
  };

  ProcessManager() = default;
  ~ProcessManager();

  ProcessManager(const ProcessManager &) = delete;
  ProcessManager & operator=(const ProcessManager &) = delete;

  bool start(const std::string & command, std::string & err_out);
  bool stop(const std::string & command, std::string & err_out);
  bool stop(const std::string & command, std::string & err_out, const StopPolicy & policy);
  void stop_all();
  void stop_all(const StopPolicy & policy);

  bool is_running(const std::string & command) const;
  std::vector<Proc> list() const;

private:
  std::map<std::string, Proc> procs_;

  static bool pid_alive(pid_t pid);
  static bool wait_dead(pid_t pid, std::chrono::milliseconds timeout);
};

struct ReadinessRule
{
  std::string type;
  std::string target;
  std::string state;
  std::string durability;
  bool required{true};
  std::string when_arg_true;
  std::string when_arg_false;
};

struct StageSpec
{
  std::string label;
  std::vector<std::string> commands;
  std::vector<ReadinessRule> readiness_rules;
  double timeout_sec{30.0};
};

class Layer1HardwareBringupNode : public rclcpp::Node
{
public:
  Layer1HardwareBringupNode();
  ~Layer1HardwareBringupNode() override;

private:
  void declare_parameters();
  void build_stages();
  void on_timer();
  bool stage_ready(const StageSpec & stage, std::vector<std::string> & missing);
  bool rule_is_enabled(const ReadinessRule & rule) const;
  bool rule_is_satisfied(const ReadinessRule & rule, std::vector<std::string> & missing);
  bool stage_process_running(const StageSpec & stage, std::vector<std::string> & missing);
  bool controller_is_active(const std::string & controller_name);
  bool hardware_component_active(const std::string & component_name, uint8_t expected_state);
  bool stage_has_started() const;
  void start_current_stage();
  void finish_current_stage();
  void fail_bringup(const std::string & reason);
  void stop_all_processes();
  void ensure_topic_subscription(const std::string & topic_name, bool transient_local);
  std::vector<std::string> build_stage_commands(const std::string & stage_name) const;
  std::vector<ReadinessRule> load_stage_rules(const YAML::Node & stage_node) const;
  std::string qualify_to_ns(const std::string & target) const;
  static std::string normalize_fqn(const std::string & name);
  std::string robot_namespace() const;
  static std::string shell_quote(const std::string & value);
  static std::string shell_join(const std::vector<std::string> & tokens);
  static std::string blue(const std::string & text);
  static uint8_t parse_lifecycle_level(const std::string & raw);
  bool param_as_bool(const std::string & name) const;
  std::string param_as_string(const std::string & name) const;
  std::map<std::string, std::vector<std::string>> topic_types() const;
  std::map<std::string, std::vector<std::string>> service_types() const;
  std::filesystem::path package_share(const std::string & package_name) const;

  ProcessManager procman_;
  std::vector<StageSpec> stages_;
  std::size_t current_stage_index_{0};
  bool bringup_complete_{false};
  bool bringup_failed_{false};
  bool current_stage_started_{false};
  std::chrono::steady_clock::time_point stage_started_at_;
  std::chrono::steady_clock::time_point stage_deadline_;

  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> topic_subscriptions_;
  std::set<std::string> ready_topics_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_sweeper_layer_1_hardware_bringup
