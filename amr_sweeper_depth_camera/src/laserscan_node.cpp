// Copyright (c) 2026 O-Robotics

#include "laserscan_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace amr_sweeper_depth_camera
{

LaserScanNode::LaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("laserscan", options)
{
  // Match common camera publisher QoS so depth and camera info actually connect.
  const auto qos = rclcpp::SensorDataQoS();

  cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "depth_camera_info",
    qos,
    std::bind(&LaserScanNode::infoCb, this, std::placeholders::_1));
  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "depth",
    qos,
    std::bind(&LaserScanNode::depthCb, this, std::placeholders::_1));
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

  scan_time_ = declare_parameter("scan_time", 0.033);
  range_min_ = declare_parameter("range_min", 0.45);
  range_max_ = declare_parameter("range_max", 10.0);
  scan_height_ = declare_parameter("scan_height", 1);
  scan_tilt_angle_deg_ = declare_parameter("scan_tilt_angle_deg", 0.0);
  output_frame_id_ = declare_parameter("output_frame", std::string("camera_depth_frame"));
}

double LaserScanNode::magnitudeOfRay(const cv::Point3d & ray) const
{
  return std::sqrt((ray.x * ray.x) + (ray.y * ray.y) + (ray.z * ray.z));
}

double LaserScanNode::angleBetweenRays(const cv::Point3d & ray1, const cv::Point3d & ray2) const
{
  const double dot_product = (ray1.x * ray2.x) + (ray1.y * ray2.y) + (ray1.z * ray2.z);
  return std::acos(dot_product / (magnitudeOfRay(ray1) * magnitudeOfRay(ray2)));
}

bool LaserScanNode::usePoint(
  float new_value,
  float old_value,
  float range_min,
  float range_max) const
{
  const bool new_finite = std::isfinite(new_value);
  const bool old_finite = std::isfinite(old_value);

  if (!new_finite && !old_finite) {
    return !std::isnan(new_value);
  }

  if (!(range_min <= new_value && new_value <= range_max)) {
    return false;
  }

  if (!old_finite) {
    return true;
  }

  return new_value < old_value;
}

sensor_msgs::msg::LaserScan::UniquePtr LaserScanNode::convertMsg(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cam_model_.fromCameraInfo(info_msg);

  const cv::Point2d raw_pixel_left(0, cam_model_.cy());
  const cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
  const cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

  const cv::Point2d raw_pixel_right(depth_msg->width - 1, cam_model_.cy());
  const cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
  const cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

  const cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  const cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  const cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

  const double angle_max = angleBetweenRays(left_ray, center_ray);
  const double angle_min = -angleBetweenRays(center_ray, right_ray);

  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = depth_msg->header;
  if (!output_frame_id_.empty()) {
    scan_msg->header.frame_id = output_frame_id_;
  }
  scan_msg->angle_min = static_cast<float>(angle_min);
  scan_msg->angle_max = static_cast<float>(angle_max);
  scan_msg->angle_increment =
    (scan_msg->angle_max - scan_msg->angle_min) / static_cast<float>(depth_msg->width - 1);
  scan_msg->time_increment = 0.0f;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;
  // Use +inf for empty bins to represent "no return within range". Nav2 can
  // raytrace these correctly when obstacle_layer.<source>.inf_is_valid is true.
  scan_msg->ranges.assign(depth_msg->width, std::numeric_limits<float>::infinity());

  const double tilt_angle_rad = scan_tilt_angle_deg_ * M_PI / 180.0;
  const int row_offset = static_cast<int>(std::lround(-cam_model_.fy() * std::tan(tilt_angle_rad)));
  const int center_row = static_cast<int>(std::lround(cam_model_.cy())) + row_offset;
  const int row_start = center_row - (scan_height_ / 2);
  const int row_end = row_start + scan_height_;
  if (row_start < 0 || row_end > static_cast<int>(depth_msg->height)) {
    std::stringstream ss;
    ss << "Requested scan rows [" << row_start << ", " << row_end
       << ") fall outside image height " << depth_msg->height
       << " when scan_height=" << scan_height_
       << ", scan_tilt_angle_deg=" << scan_tilt_angle_deg_
       << ", and computed row_offset=" << row_offset << ".";
    throw std::runtime_error(ss.str());
  }

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convert<uint16_t>(depth_msg, cam_model_, scan_msg, row_start, scan_height_);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convert<float>(depth_msg, cam_model_, scan_msg, row_start, scan_height_);
  } else {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }

  return scan_msg;
}

void LaserScanNode::infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  cam_info_ = std::move(info);
}

void LaserScanNode::depthCb(sensor_msgs::msg::Image::SharedPtr image)
{
  if (cam_info_ == nullptr) {
    RCLCPP_INFO(get_logger(), "No camera info yet, skipping laserscan conversion.");
    return;
  }

  try {
    auto scan_msg = convertMsg(image, cam_info_);
    scan_pub_->publish(std::move(scan_msg));
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR(get_logger(), "Could not convert depth image to laserscan: %s", error.what());
  }
}

}  // namespace amr_sweeper_depth_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_depth_camera::LaserScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
