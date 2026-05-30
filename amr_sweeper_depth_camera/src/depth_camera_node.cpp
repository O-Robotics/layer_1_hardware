// Copyright (c) 2026 O-Robotics

#include "depth_camera_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace amr_sweeper_depth_camera
{

namespace
{

constexpr auto kCaptureRetryDelay = std::chrono::milliseconds(500);

std::array<double, 9> identityRotation()
{
  return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

std::array<double, 12> projectionFromIntrinsics(const rs2_intrinsics & intrinsics)
{
  return {
    intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
    0.0, intrinsics.fy, intrinsics.ppy, 0.0,
    0.0, 0.0, 1.0, 0.0};
}

float packRgb(uint8_t red, uint8_t green, uint8_t blue)
{
  union
  {
    std::uint32_t rgba;
    float packed;
  } value{};
  value.rgba = (static_cast<std::uint32_t>(red) << 16) |
    (static_cast<std::uint32_t>(green) << 8) |
    static_cast<std::uint32_t>(blue);
  return value.packed;
}

}  // namespace

DepthCameraNode::DepthCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_camera", options)
{
  configureParameters();
  configurePublishers();

  // TODO(alfol): resurrect the previous cross-domain custom wrapper work once the
  // direct librealsense path is stable enough for production.
  startPipeline();
}

DepthCameraNode::~DepthCameraNode()
{
  stopPipeline();
}

void DepthCameraNode::configureParameters()
{
  serial_no_ = declare_parameter<std::string>("serial_no", "");
  use_color_ = declare_parameter("use_color", true);
  use_depth_ = declare_parameter("use_depth", true);
  use_infra1_ = declare_parameter("use_infra1", false);
  use_infra2_ = declare_parameter("use_infra2", false);
  use_motion_ = declare_parameter("use_motion", true);
  publish_pointcloud_ = declare_parameter("publish_pointcloud", true);
  align_depth_to_color_ = declare_parameter("align_depth_to_color", true);
  wait_for_frames_timeout_ms_ = std::max(
    declare_parameter("wait_for_frames_timeout_ms", 2000),
    100);

  color_profile_.width = declare_parameter("color_width", 848);
  color_profile_.height = declare_parameter("color_height", 480);
  color_profile_.fps = declare_parameter("color_fps", 15);
  depth_profile_.width = declare_parameter("depth_width", 848);
  depth_profile_.height = declare_parameter("depth_height", 480);
  depth_profile_.fps = declare_parameter("depth_fps", 15);
  infra_profile_.width = declare_parameter("infra_width", 848);
  infra_profile_.height = declare_parameter("infra_height", 480);
  infra_profile_.fps = declare_parameter("infra_fps", 15);
  accel_fps_ = declare_parameter("accel_fps", 100);
  gyro_fps_ = declare_parameter("gyro_fps", 200);

  color_frame_id_ = declare_parameter("color_frame_id", color_frame_id_);
  depth_frame_id_ = declare_parameter("depth_frame_id", depth_frame_id_);
  infra1_frame_id_ = declare_parameter("infra1_frame_id", infra1_frame_id_);
  infra2_frame_id_ = declare_parameter("infra2_frame_id", infra2_frame_id_);
  imu_frame_id_ = declare_parameter("imu_frame_id", imu_frame_id_);
  pointcloud_frame_id_ = declare_parameter("pointcloud_frame_id", pointcloud_frame_id_);

  if (!use_color_ && publish_pointcloud_) {
    RCLCPP_WARN(
      get_logger(),
      "Disabling pointcloud output because /depth/color/points depends on the color stream.");
    publish_pointcloud_ = false;
  }
  if (!use_depth_ && publish_pointcloud_) {
    RCLCPP_WARN(
      get_logger(),
      "Disabling pointcloud output because /depth/color/points depends on the depth stream.");
    publish_pointcloud_ = false;
  }
}

void DepthCameraNode::configurePublishers()
{
  const auto qos = rclcpp::SensorDataQoS();

  if (use_color_) {
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("color/image_raw", qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", qos);
  }
  if (use_depth_) {
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image", qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", qos);
  }
  if (use_infra1_) {
    infra1_pub_ = create_publisher<sensor_msgs::msg::Image>("infra1/image", qos);
    infra1_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("infra1/camera_info", qos);
  }
  if (use_infra2_) {
    infra2_pub_ = create_publisher<sensor_msgs::msg::Image>("infra2/image", qos);
    infra2_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("infra2/camera_info", qos);
  }
  if (use_motion_) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("motion/imu", qos);
  }
  if (publish_pointcloud_) {
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("depth/color/points", qos);
  }
}

void DepthCameraNode::startPipeline()
{
  if (use_color_) {
    pipeline_config_.enable_stream(
      RS2_STREAM_COLOR,
      color_profile_.width,
      color_profile_.height,
      RS2_FORMAT_RGB8,
      color_profile_.fps);
  }
  if (use_depth_) {
    pipeline_config_.enable_stream(
      RS2_STREAM_DEPTH,
      depth_profile_.width,
      depth_profile_.height,
      RS2_FORMAT_Z16,
      depth_profile_.fps);
  }
  if (use_infra1_) {
    pipeline_config_.enable_stream(
      RS2_STREAM_INFRARED,
      1,
      infra_profile_.width,
      infra_profile_.height,
      RS2_FORMAT_Y8,
      infra_profile_.fps);
  }
  if (use_infra2_) {
    pipeline_config_.enable_stream(
      RS2_STREAM_INFRARED,
      2,
      infra_profile_.width,
      infra_profile_.height,
      RS2_FORMAT_Y8,
      infra_profile_.fps);
  }
  if (use_motion_) {
    pipeline_config_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, accel_fps_);
    pipeline_config_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, gyro_fps_);
  }
  if (!serial_no_.empty()) {
    pipeline_config_.enable_device(serial_no_);
  }

  running_.store(true);
  capture_thread_ = std::thread(&DepthCameraNode::captureLoop, this);
}

void DepthCameraNode::stopPipeline()
{
  running_.store(false);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (pipeline_profile_.has_value()) {
    try {
      pipeline_.stop();
    } catch (const rs2::error & error) {
      RCLCPP_WARN(
        get_logger(),
        "Error while stopping RealSense pipeline: %s",
        error.what());
    }
    pipeline_profile_.reset();
  }
}

sensor_msgs::msg::CameraInfo DepthCameraNode::buildCameraInfo(
  const rs2::video_stream_profile & profile,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  const rs2_intrinsics intrinsics = profile.get_intrinsics();

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = frame_id;
  info.width = static_cast<std::uint32_t>(intrinsics.width);
  info.height = static_cast<std::uint32_t>(intrinsics.height);
  info.k = {
    intrinsics.fx, 0.0, intrinsics.ppx,
    0.0, intrinsics.fy, intrinsics.ppy,
    0.0, 0.0, 1.0};
  info.p = projectionFromIntrinsics(intrinsics);
  info.r = identityRotation();
  info.distortion_model = "plumb_bob";
  info.d.assign(
    intrinsics.coeffs,
    intrinsics.coeffs + std::size(intrinsics.coeffs));
  return info;
}

sensor_msgs::msg::Image DepthCameraNode::buildImageMessage(
  const rs2::video_frame & frame,
  const std::string & encoding,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::Image message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.height = frame.get_height();
  message.width = frame.get_width();
  message.encoding = encoding;
  message.is_bigendian = false;
  message.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.get_stride_in_bytes());
  const auto * raw = static_cast<const std::uint8_t *>(frame.get_data());
  message.data.assign(raw, raw + (message.step * message.height));
  return message;
}

sensor_msgs::msg::PointCloud2 DepthCameraNode::buildPointCloudMessage(
  const rs2::depth_frame & depth_frame,
  const rs2::video_frame & color_frame,
  const rclcpp::Time & stamp)
{
  pointcloud_processor_.map_to(color_frame);
  const rs2::points points = pointcloud_processor_.calculate(depth_frame);
  const auto vertices = points.get_vertices();
  const auto texcoords = points.get_texture_coordinates();

  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = stamp;
  message.header.frame_id = pointcloud_frame_id_;
  message.height = 1;
  message.width = points.size();
  message.is_bigendian = false;
  message.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_rgb(message, "rgb");

  const auto * color_data = static_cast<const std::uint8_t *>(color_frame.get_data());
  const int color_width = color_frame.get_width();
  const int color_height = color_frame.get_height();
  const int color_pixel_stride = color_frame.get_bytes_per_pixel();
  const int color_row_stride = color_frame.get_stride_in_bytes();

  for (std::size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
    const auto & vertex = vertices[i];
    *iter_x = vertex.x;
    *iter_y = vertex.y;
    *iter_z = vertex.z;

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    if (std::isfinite(vertex.z)) {
      const int x = std::clamp(
        static_cast<int>(texcoords[i].u * static_cast<float>(color_width)),
        0,
        std::max(color_width - 1, 0));
      const int y = std::clamp(
        static_cast<int>(texcoords[i].v * static_cast<float>(color_height)),
        0,
        std::max(color_height - 1, 0));
      const std::size_t offset = static_cast<std::size_t>((y * color_row_stride) + (x * color_pixel_stride));
      red = color_data[offset + 0];
      green = color_data[offset + 1];
      blue = color_data[offset + 2];
    }
    *iter_rgb = packRgb(red, green, blue);
  }

  return message;
}

void DepthCameraNode::publishMotionFrame(const rs2::motion_frame & motion_frame, const rclcpp::Time & stamp)
{
  if (!imu_pub_) {
    return;
  }

  const auto motion = motion_frame.get_motion_data();
  std::scoped_lock lock(imu_mutex_);
  latest_imu_.header.stamp = stamp;
  latest_imu_.header.frame_id = imu_frame_id_;

  if (motion_frame.get_profile().stream_type() == RS2_STREAM_ACCEL) {
    latest_imu_.linear_acceleration.x = motion.x;
    latest_imu_.linear_acceleration.y = motion.y;
    latest_imu_.linear_acceleration.z = motion.z;
    have_accel_ = true;
  } else if (motion_frame.get_profile().stream_type() == RS2_STREAM_GYRO) {
    latest_imu_.angular_velocity.x = motion.x;
    latest_imu_.angular_velocity.y = motion.y;
    latest_imu_.angular_velocity.z = motion.z;
    have_gyro_ = true;
  }

  if (!have_accel_ || !have_gyro_) {
    return;
  }

  latest_imu_.orientation_covariance[0] = -1.0;
  latest_imu_.angular_velocity_covariance = {
    0.001, 0.0, 0.0,
    0.0, 0.001, 0.0,
    0.0, 0.0, 0.001};
  latest_imu_.linear_acceleration_covariance = {
    0.01, 0.0, 0.0,
    0.0, 0.01, 0.0,
    0.0, 0.0, 0.01};
  imu_pub_->publish(latest_imu_);
}

void DepthCameraNode::captureLoop()
{
  while (running_.load()) {
    try {
      if (!pipeline_profile_.has_value()) {
        pipeline_profile_.emplace(pipeline_.start(pipeline_config_));
        if (align_depth_to_color_ && use_color_ && use_depth_) {
          align_to_color_processor_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
        }
        RCLCPP_INFO(get_logger(), "RealSense pipeline started.");
      }

      const rs2::frameset raw_frames = pipeline_.wait_for_frames(wait_for_frames_timeout_ms_);
      const rclcpp::Time stamp = now();
      for (const rs2::frame & frame : raw_frames) {
        if (auto motion_frame = frame.as<rs2::motion_frame>()) {
          publishMotionFrame(motion_frame, stamp);
        }
      }

      if (use_color_) {
        const auto color_frame = raw_frames.get_color_frame();
        if (color_frame && color_pub_ && color_info_pub_) {
          color_pub_->publish(
            buildImageMessage(
              color_frame,
              sensor_msgs::image_encodings::RGB8,
              color_frame_id_,
              stamp));
          color_info_pub_->publish(
            buildCameraInfo(color_frame.get_profile().as<rs2::video_stream_profile>(), color_frame_id_, stamp));
        }
      }

      if (use_depth_) {
        const auto depth_frame = raw_frames.get_depth_frame();
        if (depth_frame && depth_pub_ && depth_info_pub_) {
          depth_pub_->publish(
            buildImageMessage(
              depth_frame,
              sensor_msgs::image_encodings::TYPE_16UC1,
              depth_frame_id_,
              stamp));
          depth_info_pub_->publish(
            buildCameraInfo(depth_frame.get_profile().as<rs2::video_stream_profile>(), depth_frame_id_, stamp));
        }
      }

      if (use_infra1_) {
        const auto infra1_frame = raw_frames.get_infrared_frame(1);
        if (infra1_frame && infra1_pub_ && infra1_info_pub_) {
          infra1_pub_->publish(
            buildImageMessage(
              infra1_frame,
              sensor_msgs::image_encodings::MONO8,
              infra1_frame_id_,
              stamp));
          infra1_info_pub_->publish(
            buildCameraInfo(infra1_frame.get_profile().as<rs2::video_stream_profile>(), infra1_frame_id_, stamp));
        }
      }

      if (use_infra2_) {
        const auto infra2_frame = raw_frames.get_infrared_frame(2);
        if (infra2_frame && infra2_pub_ && infra2_info_pub_) {
          infra2_pub_->publish(
            buildImageMessage(
              infra2_frame,
              sensor_msgs::image_encodings::MONO8,
              infra2_frame_id_,
              stamp));
          infra2_info_pub_->publish(
            buildCameraInfo(infra2_frame.get_profile().as<rs2::video_stream_profile>(), infra2_frame_id_, stamp));
        }
      }

      if (publish_pointcloud_ && pointcloud_pub_) {
        const rs2::frameset pointcloud_frames =
          (align_to_color_processor_ != nullptr) ? align_to_color_processor_->process(raw_frames) :
          raw_frames;
        const auto depth_frame = pointcloud_frames.get_depth_frame();
        const auto color_frame = pointcloud_frames.get_color_frame();
        if (depth_frame && color_frame) {
          pointcloud_pub_->publish(buildPointCloudMessage(depth_frame, color_frame, stamp));
        }
      }
    } catch (const rs2::error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "RealSense pipeline error: %s",
        error.what());
      if (pipeline_profile_.has_value()) {
        try {
          pipeline_.stop();
        } catch (const rs2::error &) {
          // Best effort while we retry.
        }
        pipeline_profile_.reset();
      }
      std::this_thread::sleep_for(kCaptureRetryDelay);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Depth camera wrapper error: %s",
        error.what());
      std::this_thread::sleep_for(kCaptureRetryDelay);
    }
  }
}

}  // namespace amr_sweeper_depth_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_depth_camera::DepthCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
