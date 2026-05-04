#include "steadydrive_hw_interface/robot_hardware_interface_steadydrive.hpp"

// Replace defines with constexpr for motor indices
namespace {
    constexpr int LEFT_MOTOR_INDEX = 0;
    constexpr int RIGHT_MOTOR_INDEX = 1;
}

using amr_sweeper_steadydrive::SteadydriveHardwareInterface;



hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{  
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_name_ = info_.name;
  num_joints_ = info_.joints.size();

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Name: %s", hw_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Number of Joints %u", num_joints_);


  velocity_commands_.resize(num_joints_);
  prev_velocity_commands_.resize(num_joints_);
  velocity_states_.resize(num_joints_);
  position_states_.resize(num_joints_);

  if(num_joints_ != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Incorrect number of joints");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    std::string topic_speed_output_left = info_.hardware_parameters.at("topic_speed_output_left");
    std::string topic_speed_output_right = info_.hardware_parameters.at("topic_speed_output_right");
    std::string topic_motor_state_left = info_.hardware_parameters.at("topic_motor_state_left");
    std::string topic_motor_state_right = info_.hardware_parameters.at("topic_motor_state_right");

    RCLCPP_INFO(rclcpp::get_logger(hw_name_),
                "Loaded parameters: topic_speed_output_left=%s, topic_speed_output_right=%s, topic_motor_state_left=%s, topic_motor_state_right=%s",
                topic_speed_output_left.c_str(),
                topic_speed_output_right.c_str(),
                topic_motor_state_left.c_str(),
                topic_motor_state_right.c_str());

  node_ = std::make_shared<rclcpp::Node>(hw_name_);  
  publisher_left_ = node_->create_publisher<std_msgs::msg::Float32>(topic_speed_output_left, 10);
  publisher_right_ = node_->create_publisher<std_msgs::msg::Float32>(topic_speed_output_right, 10);
  subscriber_motor_state_left = node_->create_subscription<sensor_msgs::msg::JointState>(
      topic_motor_state_left, 10, std::bind(&SteadydriveHardwareInterface::callback_motor_state_left, this, std::placeholders::_1));
  subscriber_motor_state_right = node_->create_subscription<sensor_msgs::msg::JointState>(
      topic_motor_state_right, 10, std::bind(&SteadydriveHardwareInterface::callback_motor_state_right, this, std::placeholders::_1));

  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Parameter missing: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Error parsing parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }


  return hardware_interface::CallbackReturn::SUCCESS;
}


/**
 * @brief Write commanded velocities to the steadydrive
 */
void SteadydriveHardwareInterface::writeCommandsToHardware()
{
  std_msgs::msg::Float32 msg_left;
  std_msgs::msg::Float32 msg_right;
  msg_left.data = velocity_commands_[LEFT_MOTOR_INDEX];
  msg_right.data = velocity_commands_[RIGHT_MOTOR_INDEX];
  publisher_left_->publish(msg_left);
  publisher_right_->publish(msg_right);

  RCLCPP_DEBUG(
    rclcpp::get_logger(hw_name_),
    "Publishing command velocities (L: %f, R: %f)",
    msg_left.data, msg_right.data);


}
/**
 * @brief Pull latest speed and travel measurements from MCU, 
 * and store in joint structure for ros_control
 * 
 */
void SteadydriveHardwareInterface::updateJointsFromHardware()
{
  rclcpp::spin_some(node_);
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), 
    "Reading joint states (L: %f, R: %f)",
    position_states_[LEFT_MOTOR_INDEX], position_states_[RIGHT_MOTOR_INDEX]);
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_),
    "Reading joint velocities (L: %f, R: %f)",
    velocity_states_[LEFT_MOTOR_INDEX], velocity_states_[RIGHT_MOTOR_INDEX]);     
}


hardware_interface::CallbackReturn SteadydriveHardwareInterface::validateJoints()
{ 
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffDriveHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(hw_name_),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> SteadydriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < num_joints_; i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SteadydriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < num_joints_; i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Starting ...please wait...");
  // set some default values
  for (auto i = 0u; i < num_joints_; i++) {
    velocity_commands_[i] = 0.0;
    prev_velocity_commands_[i] = 0.0;
    position_states_[i] = 0.0;
    velocity_states_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteadydriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System successfully stopped!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SteadydriveHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Reading from hardware");
  updateJointsFromHardware();
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully read!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SteadydriveHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Writing to hardware");
  writeCommandsToHardware();
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully written!");
  return hardware_interface::return_type::OK;
}

void SteadydriveHardwareInterface::callback_motor_state_left(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->velocity.size() > 0 && msg->position.size() > 0)
    {
        velocity_states_[LEFT_MOTOR_INDEX] = msg->velocity[0];
        position_states_[LEFT_MOTOR_INDEX] = msg->position[0];
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger(hw_name_), "Received incomplete motor state for left motor");
    }
}

void SteadydriveHardwareInterface::callback_motor_state_right(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->velocity.size() > 0 && msg->position.size() > 0)
    {
        velocity_states_[RIGHT_MOTOR_INDEX] = msg->velocity[0];
        position_states_[RIGHT_MOTOR_INDEX] = msg->position[0];
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger(hw_name_), "Received incomplete motor state for right motor");
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(amr_sweeper_steadydrive::SteadydriveHardwareInterface, hardware_interface::SystemInterface)
