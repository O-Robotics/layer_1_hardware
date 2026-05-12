# amr_sweeper_depth_camera

```bash
ros2 launch amr_sweeper_depth_camera depth_camera.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_description`

## Purpose
This package launches the RealSense depth-camera stack used by the AMR Sweeper.

## Main Launch File
`launch/depth_camera.launch.py`

## Available Launch Files
- `depth_camera.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_depth_camera`: default `true`
- `use_depthimage_to_laserscan`: default `true`
- `camera_name`: default `depth_camera`
- `log_level`: default `info`
- `serial_no`: default ``
- `usb_port_id`: default ``
- `enable_pointcloud`: default `false`
- `align_depth`: default `false`
- `initial_reset`: default `false`

## Overview
`amr_sweeper_depth_camera` is a layer 1 integration package. It provides launch and configuration for `realsense2_camera` and `depthimage_to_laserscan`, both of which must already be installed in the robot build and runtime environment.

## Notes
- The launch starts `realsense2_camera` and `depthimage_to_laserscan` together.
- The default namespace layout is `/amr_sweeper/depth_camera/...` for the RealSense topics and `/amr_sweeper/depth_camera/scan` for the derived laser scan.
- The default `camera_name` is `depth_camera`, so the runtime frame IDs align with the URDF frames `depth_camera_link`, `depth_camera_depth_frame`, and `depth_camera_color_frame`.
- TF publication is disabled in the RealSense driver because the robot description already defines the camera frame tree.
- `realsense2_camera` and `depthimage_to_laserscan` must be installed and discoverable in the sourced ROS environment before launching this package.
