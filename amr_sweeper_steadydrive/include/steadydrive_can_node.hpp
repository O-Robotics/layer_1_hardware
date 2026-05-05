#ifndef AMR_SWEEPER_STEADYDRIVE__STEADYDRIVE_CAN_NODE_HPP_
#define AMR_SWEEPER_STEADYDRIVE__STEADYDRIVE_CAN_NODE_HPP_

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <linux/can.h>

class SteadyDriveCanNode : public rclcpp::Node
{
public:
  SteadyDriveCanNode();
  ~SteadyDriveCanNode() override;

private:
  bool initialize_can_socket();
  bool send_can_command(
    uint8_t command_byte, struct can_frame & response,
    uint8_t byte1 = 0x00, uint8_t byte2 = 0x00, uint8_t byte3 = 0x00,
    uint8_t byte4 = 0x00, uint8_t byte5 = 0x00, uint8_t byte6 = 0x00,
    uint8_t byte7 = 0x00);
  void joint_state_update_callback();

  void motor_off_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void motor_on_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void motor_stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void read_encoder_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void read_motor_state1_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void clear_motor_error_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void read_motor_state2_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void read_motor_state3_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void speed_control_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void torque_control_callback(const std_msgs::msg::Float32::SharedPtr msg);

  int can_socket_{-1};
  std::string can_interface_;
  uint32_t motor_can_id_{0x141};
  std::optional<float> last_logged_speed_dps_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motor_off_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motor_on_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motor_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr read_encoder_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr read_motor_state1_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_motor_error_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr read_motor_state2_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr read_motor_state3_service_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_control_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_control_subscriber_;

  rclcpp::TimerBase::SharedPtr joint_state_publisher_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
};

#endif  // AMR_SWEEPER_STEADYDRIVE__STEADYDRIVE_CAN_NODE_HPP_
