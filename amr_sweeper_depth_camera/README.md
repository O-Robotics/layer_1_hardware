# amr_sweeper_depth_camera

```bash
ros2 launch amr_sweeper_depth_camera amr_sweeper_depth_camera.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package bridges the native ROS 2 topics from the RealSense D555 camera into the AMR Sweeper namespace convention and runs the local depth-image-to-laserscan conversion node.

## Main Launch File
`launch/amr_sweeper_depth_camera.launch.py`

## Available Launch Files
- `amr_sweeper_depth_camera.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper/depth_camera`
- `log_level`: default `info`
- `use_sim_time`: default `false`
- `use_domain_bridge`: default `true`
- `source_domain_id`: default `5`
- `target_domain_id`: default `0`
- `source_root_namespace`: default `/realsense`
- `source_camera_model`: default `D555`
- `source_camera_id`: default empty, which auto-discovers the serial-specific camera ID on the source domain
- `domain_bridge_config_file`: default `config/domain_bridge.yaml`
- `use_laserscan`: default `true`
- `laserscan_params_file`: default `config/amr_sweeper_depth_camera_laserscan_node.yaml`
- `depth_image_topic`: default derived from `namespace`, e.g. `/amr_sweeper/depth_camera/depth/image_rect_raw`
- `depth_camera_info_topic`: default derived from `namespace`, e.g. `/amr_sweeper/depth_camera/depth/camera_info`
- `pointcloud_topic`: default derived from `namespace`, e.g. `/amr_sweeper/depth_camera/depth/color/points`
- `depth_camera_frame`: default `depth_camera_link`
- `scan_topic`: default `scan`
- `output_frame`: default from `config/amr_sweeper_depth_camera_laserscan_node.yaml` (`laserscan_link`)
- `range_min`: default from `config/amr_sweeper_depth_camera_laserscan_node.yaml` (`0.20`)
- `range_max`: default from `config/amr_sweeper_depth_camera_laserscan_node.yaml` (`5.0`)
- `scan_height`: default from `config/amr_sweeper_depth_camera_laserscan_node.yaml` (`5`)
- `scan_tilt_angle_deg`: default from `config/amr_sweeper_depth_camera_laserscan_node.yaml` (`4.5`)
- `scan_time`: default from `config/amr_sweeper_depth_camera_laserscan_node.yaml` (`0.050`)

## Overview
`amr_sweeper_depth_camera` contains the launch configuration needed to discover the serial-specific D555 topic prefix on ROS domain `5`, generate a resolved `domain_bridge` configuration from `config/domain_bridge.yaml`, and bridge those topics into the flattened AMR Sweeper namespace, for example `/amr_sweeper/depth_camera/depth/image_rect_raw`. The local `laserscan_node` and watchdog consume the bridged depth image, camera info, and point cloud topics, and the watchdog uses the same warn/error/fatal escalation pattern used elsewhere in the workspace.

## Notes
 - Main nodes: `domain_bridge`, `laserscan_node`, and `depth_camera_watchdog_node`.
 - A `tf2_ros` static transform publisher creates `laserscan_link` under `depth_camera_link`.
 - The bridge rewrites serial-specific native camera topics under `/realsense/D555_<serial>...` into `/amr_sweeper/depth_camera/...`.
 - The default output topic resolves to `/amr_sweeper/depth_camera/scan`.
 - The default `output_frame` is `laserscan_link`, a frame pitched upward to match the selected scan row.
 - `scan_tilt_angle_deg` is resolution-independent and shifts the sampled scan band by angle instead of raw pixels.
 - If topic auto-discovery finds more than one D555 on the source domain, set `source_camera_id` explicitly.
 - The current bridge template covers the native topics supplied for the D555, including metadata, compressed color, motion, and object detection.
 - The watchdog now treats a missing `/depth/color/points` topic as a health failure by default because the mapping stack depends on it.
