# amr_sweeper_description

```bash
ros2 launch amr_sweeper_description amr_sweeper_description.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package contains the robot description used by the AMR Sweeper runtime stack.

## Main Launch File
`launch/amr_sweeper_description.launch.py`

## Available Launch Files
- `amr_sweeper_description.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_ros2_control`: default `true`
- `ros2_control_config_file`: default `urdf/control/ros2_control.yaml`
- `enable_usb_cameras`: default `true`
- `enable_gnss`: default `true`
- `enable_imu`: default `true`
- `enable_depth_camera`: default `true`

## Overview
`amr_sweeper_description` provides the URDF/Xacro model, meshes, and robot_state_publisher launch setup for the platform. It is used by the hardware layer to expose the robot model and by higher layers that rely on a consistent frame tree and robot description.

## Notes
- The package is commonly started as part of `amr_sweeper_layer_1_hardware_bringup`.
- It provides the robot model foundation used by ros2_control and localization.
- In the default hardware bringup, `robot_state_publisher` from this package supplies the `robot_description` topic consumed by ros2_control.
- The default robot root namespace is `/amr_sweeper`.
- `amr_sweeper_description.launch.py` owns the default controller-config path used by the layer 1 ros2_control bringup.
- Each USB camera now exposes a single physical frame named `<camera_name>_link` plus the ROS optical frame `<camera_name>_optical_frame`, with no translation offset between them.
