# layer_1_hardware

```
ros2 launch amr_sweeper_layer_1_hardware_bringup amr_sweeper_layer_1_hardware_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_battery`
- `amr_sweeper_depth_camera`
- `amr_sweeper_description`
- `amr_sweeper_gnss`
- `amr_sweeper_imu`
- `amr_sweeper_odrive`
- `amr_sweeper_steadydrive`
- `amr_sweeper_system_info`
- `amr_sweeper_usb_cameras`

## Purpose
This repository is the real-robot hardware layer for the AMR Sweeper. It contains the packages that expose the physical robot model, wheel and tool motor interfaces, battery monitoring, GNSS, IMU, USB cameras, and system-health publishing.

## Launch Arguments
- `robot_namespace`: default `amr_sweeper`
- `log_level`: default `info`
- `use_sim_time`: default `false`
- `use_robot_description`: default `true`
- `use_ros2_control`: default `true`
- `use_battery_node`: default `true`
- `use_system_info_node`: default `true`
- `use_usb_cameras`: default `true`
- `use_depth_camera`: default `true`
- `use_imu_node`: default `true`
- `use_gnss_rover`: default `true`
- `use_ntrip_client`: default `false`
- `use_steadydrive_can_nodes`: default `true`
- `use_odrive_node`: default `false`
- `battery_can_interface`: default `can0`
- `steadydrive_can_interface`: default `can0`
- `imu_port`: default `/dev/imu_usb`
- `imu_baud`: default `9600`
- `odrive_interface`: default `can0`
- `odrive_node_id`: default `0`

## Overview
Layer 1 is the base runtime layer for the rest of the stack. It is responsible for making the robot's hardware available as ROS 2 topics, services, and ros2_control interfaces. Layer 2 controllers and layer 3 navigation depend on this layer to provide odometry, transforms, actuator interfaces, and sensor data. This includes the vendor-style RealSense depth camera package, which also publishes a derived scan from the depth stream.

## Notes
- The default command launches the full layer 1 hardware bringup package.
- This layer is intended for the physical AMR Sweeper robot, not simulation.
- Layer 2 and layer 3 should be started only after the required layer 1 hardware interfaces are available.
- The layer 1 ros2_control bringup relies on `robot_state_publisher` for the `robot_description` topic instead of passing the description directly into `ros2_control_node`.
- Controller spawners in the layer 1 ros2_control launch load controller settings from the shared ros2_control YAML file, which keeps the bringup aligned with ROS 2 Jazzy while remaining workable on Humble.
