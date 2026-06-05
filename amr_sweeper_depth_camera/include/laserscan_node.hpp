// Copyright (c) 2026 O-Robotics

#ifndef LASERSCAN_NODE_HPP_
#define LASERSCAN_NODE_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#if __has_include("image_geometry/pinhole_camera_model.hpp")
#include "image_geometry/pinhole_camera_model.hpp"
#else
#include "image_geometry/pinhole_camera_model.h"
#endif
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace amr_sweeper_depth_camera
{

template<typename T>
struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) {return depth != 0;}
  static inline float toMeters(uint16_t depth) {return depth * 0.001f;}
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) {return std::isfinite(depth);}
  static inline float toMeters(float depth) {return depth;}
};

class LaserScanNode final : public rclcpp::Node
{
public:
  explicit LaserScanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  double magnitudeOfRay(const cv::Point3d & ray) const;
  double angleBetweenRays(const cv::Point3d & ray1, const cv::Point3d & ray2) const;
  bool usePoint(float new_value, float old_value, float range_min, float range_max) const;

  sensor_msgs::msg::LaserScan::UniquePtr convertMsg(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  template<typename T>
  void convert(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const image_geometry::PinholeCameraModel & cam_model,
    const sensor_msgs::msg::LaserScan::UniquePtr & scan_msg,
    int row_start,
    int row_count) const
  {
    const float center_x = cam_model.cx();
    const double unit_scaling = DepthTraits<T>::toMeters(T(1));
    const float constant_x = unit_scaling / cam_model.fx();

    const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
    const int row_step = static_cast<int>(depth_msg->step / sizeof(T));
    depth_row += row_start * row_step;

    for (int row = 0; row < row_count; ++row, depth_row += row_step) {
      for (uint32_t u = 0; u < depth_msg->width; ++u) {
        const T depth = depth_row[u];

        double range = depth;
        const double angle = -std::atan2(
          static_cast<double>(u - center_x) * constant_x,
          unit_scaling);
        const int index = static_cast<int>(
          (angle - scan_msg->angle_min) / scan_msg->angle_increment);
        if (index < 0 || index >= static_cast<int>(scan_msg->ranges.size())) {
          continue;
        }

        if (DepthTraits<T>::valid(depth)) {
          const double x = (u - center_x) * depth * constant_x;
          const double z = DepthTraits<T>::toMeters(depth);
          range = std::sqrt((x * x) + (z * z));
        }

        if (usePoint(
            range,
            scan_msg->ranges[static_cast<size_t>(index)],
            scan_msg->range_min,
            scan_msg->range_max))
        {
          scan_msg->ranges[static_cast<size_t>(index)] = static_cast<float>(range);
        }
      }
    }
  }

  void infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info);
  void depthCb(sensor_msgs::msg::Image::SharedPtr image);

  image_geometry::PinholeCameraModel cam_model_;
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;
  bool waiting_for_camera_info_logged_{false};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  float scan_time_;
  float range_min_;
  float range_max_;
  int scan_height_;
  double scan_tilt_angle_deg_;
  std::string output_frame_id_;
};

}  // namespace amr_sweeper_depth_camera

#endif  // LASERSCAN_NODE_HPP_
