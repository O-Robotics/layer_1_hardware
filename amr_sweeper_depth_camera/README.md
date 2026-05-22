# amr_sweeper_depth_camera

```bash
ros2 launch amr_sweeper_depth_camera amr_sweeper_depth_camera.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package runs the RealSense depth camera driver together with the local depth-image-to-laserscan conversion node used by the AMR Sweeper depth camera setup.

## Main Launch File
`launch/amr_sweeper_depth_camera.launch.py`

## Available Launch Files
- `amr_sweeper_depth_camera.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper/depth_camera`
- `log_level`: default `info`
- `use_sim_time`: default `false`
- `use_realsense_ros`: default `true`
- `realsense_params_file`: default `config/realsense-ros.yaml`
- `use_laserscan`: default `true`
- `laserscan_params_file`: default `config/laserscan.yaml`
- `depth_image_topic`: default derived from `namespace`, e.g. `/amr_sweeper/depth_camera/depth/image_rect_raw`
- `depth_camera_info_topic`: default derived from `namespace`, e.g. `/amr_sweeper/depth_camera/depth/camera_info`
- `depth_camera_frame`: default `depth_camera_link`
- `scan_topic`: default `scan`
- `output_frame`: default from `config/laserscan.yaml` (`laserscan_link`)
- `range_min`: default from `config/laserscan.yaml` (`0.20`)
- `range_max`: default from `config/laserscan.yaml` (`5.0`)
- `scan_height`: default from `config/laserscan.yaml` (`5`)
- `scan_tilt_angle_deg`: default from `config/laserscan.yaml` (`4.5`)
- `scan_time`: default from `config/laserscan.yaml` (`0.050`)

## Overview
`amr_sweeper_depth_camera` contains the launch configuration needed to start the RealSense D555 driver and convert its depth image stream into a `sensor_msgs/LaserScan` using the local `laserscan_node`. The launch file derives the RealSense node placement from `namespace` so the driver itself resolves to that exact path, for example `/amr_sweeper/depth_camera`. The RealSense YAML keeps the repo-wide ROS parameter file layout, and the launch file unpacks it before passing the parameters to `realsense2_camera_node`.

## Notes
- Main nodes: `realsense2_camera_node` and `laserscan_node`.
- A `tf2_ros` static transform publisher creates `laserscan_link` under `depth_camera_link`.
- The default input topics follow the flattened workspace namespace style under `/amr_sweeper/depth_camera/...`.
- The default output topic resolves to `/amr_sweeper/depth_camera/scan`.
- The default `output_frame` is `laserscan_link`, a frame pitched upward to match the selected scan row.
- `scan_tilt_angle_deg` is resolution-independent and shifts the sampled scan band by angle instead of raw pixels.
- The RealSense driver namespace is derived automatically from the parent of `namespace`, and the node name is the leaf of `namespace`.
- Override the topic launch arguments if the RealSense node should publish under a different input topic path.
