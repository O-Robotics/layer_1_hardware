# amr_sweeper_layer_1_hardware_bringup

```bash
ros2 launch amr_sweeper_layer_1_hardware_bringup amr_sweeper_layer_1_hardware_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
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
This package is the main entrypoint for the AMR Sweeper hardware layer. It gathers the hardware-related packages in layer 1 and launches them in a coordinated way for the real robot.

## Main Launch File
`launch/amr_sweeper_layer_1_hardware_bringup.launch.py`

## Available Launch Files
- `amr_sweeper_layer_1_hardware_bringup.launch.py`
- `amr_sweeper_ros2_control.launch.py`

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
The main bringup launch starts the core hardware stack under the `amr_sweeper` namespace. Depending on launch arguments, it can enable the robot description, ros2_control, battery monitor, system information publisher, USB cameras, the RealSense depth camera, IMU, GNSS rover, optional NTRIP client, SteadyDrive nodes, and the standalone ODrive CAN node. The USB camera nodes are launched under `/amr_sweeper/usb_cameras/<camera_name>`, while the default RealSense topic root is `/amr_sweeper/depth_camera` and the derived depth scan is published on `/amr_sweeper/depth_camera/scan`.

## Notes
- Use this package when you want to start the whole layer 1 stack from a single command.
- `amr_sweeper_ros2_control.launch.py` is the lower-level ros2_control launch used by the main bringup.
- `amr_sweeper_ros2_control.launch.py` expects `robot_state_publisher` to publish the robot description and keeps the controller manager subscribed through topic remapping that remains compatible with both ROS 2 Humble and Jazzy.
- The ros2_control controller spawners load controller settings from `config/ros2_control.yaml` with `--param-file`.
- The `robot_namespace` argument becomes the root of the hardware stack, while USB camera nodes are placed in the nested `usb_cameras` namespace under that root.
- The depth camera launch keeps the upstream RealSense `camera_name` at `depth_camera` by default so its frame names stay aligned with the depth camera frames already defined in the robot description.
