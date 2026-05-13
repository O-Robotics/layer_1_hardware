# amr_sweeper_depth_camera

```bash
ros2 launch amr_sweeper_depth_camera amr_sweeper_depth_camera.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package runs the depth-image-to-laserscan conversion node used by the AMR Sweeper depth camera setup.

## Main Launch File
`launch/amr_sweeper_depth_camera.launch.py`

## Available Launch Files
- `amr_sweeper_depth_camera.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper/depth_camera`
- `log_level`: default `info`
- `use_sim_time`: default `false`
- `use_depthimage_to_laserscan`: default `true`
- `depth_image_topic`: default `/camera/camera/depth/image_rect_raw`
- `depth_camera_info_topic`: default `/camera/camera/depth/camera_info`
- `scan_topic`: default `scan`
- `output_frame`: default `depth_camera_depth_optical_frame`
- `range_min`: default `0.25`
- `range_max`: default `8.0`
- `scan_height`: default `20`
- `scan_time`: default `0.0333`

## Overview
`amr_sweeper_depth_camera` contains the launch configuration needed to convert the RealSense D555 depth image stream into a `sensor_msgs/LaserScan` using the ROS `depthimage_to_laserscan` node. The package assumes the RealSense driver is already running on the robot and keeps the input topics configurable so robot-specific environment variables or driver launch setups can decide where the depth stream comes from.

## Notes
- Main node: `depthimage_to_laserscan_node`.
- The default input topics match the common RealSense ROS topic layout under `/camera/camera/...`.
- The default `output_frame` matches the depth camera frame names already present in the robot description.
- Override the topic launch arguments if the RealSense node on the robot publishes under a different namespace.
