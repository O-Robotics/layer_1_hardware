# amr_sweeper_description

`ros2 launch amr_sweeper_description rsp.launch.py`

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package contains the robot description used by the AMR Sweeper runtime stack.

## Main Launch File
`launch/rsp.launch.py`

## Available Launch Files
- `rsp.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_ros2_control`: default `true`
- `enable_top_cameras`: default `true`
- `enable_gnss`: default `true`
- `enable_imu`: default `true`
- `enable_depth_camera`: default `true`

## Overview
`amr_sweeper_description` provides the URDF/Xacro model, meshes, and robot_state_publisher launch setup for the platform. It is used by the hardware layer to expose the robot model and by higher layers that rely on a consistent frame tree and robot description.

## Notes
- The package is commonly started as part of `amr_sweeper_layer_1_hardware_bringup`.
- It provides the robot model foundation used by ros2_control and localization.
