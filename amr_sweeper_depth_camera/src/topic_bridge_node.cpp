// Copyright (c) 2026 O-Robotics

#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace amr_sweeper_depth_camera
{

class TopicBridgeNode final : public rclcpp::Node
{
public:
  TopicBridgeNode()
  : rclcpp::Node("topic_bridge")
  {
    const auto qos = rclcpp::SensorDataQoS();

    color_image_pub_ = create_publisher<sensor_msgs::msg::Image>("color/image_raw", qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", qos);
    depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image", qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("depth/color/points", qos);
    motion_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("motion/imu", qos);

    color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "input/color/image_raw",
      qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        color_image_pub_->publish(std::move(*msg));
      });
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "input/color/camera_info",
      qos,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        color_info_pub_->publish(std::move(*msg));
      });
    depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "input/depth/image",
      qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        depth_image_pub_->publish(std::move(*msg));
      });
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "input/depth/camera_info",
      qos,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        depth_info_pub_->publish(std::move(*msg));
      });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/depth/color/points",
      qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pointcloud_pub_->publish(std::move(*msg));
      });
    motion_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "input/motion/imu",
      qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        motion_imu_pub_->publish(std::move(*msg));
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr motion_imu_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr motion_imu_sub_;
};

}  // namespace amr_sweeper_depth_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_depth_camera::TopicBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
