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

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `log_level`: default `info`
- `ublox_log_level`: default `WARN`
- `use_sim_time`: default `false`
- `use_amr_sweeper_description`: default `true`
- `use_ros2_control`: default `true`
- `use_amr_sweeper_battery`: default `true`
- `use_amr_sweeper_system_info`: default `true`
- `use_amr_sweeper_usb_cameras`: default `true`
- `use_amr_sweeper_depth_camera`: default `true`
- `use_amr_sweeper_imu`: default `true`
- `use_amr_sweeper_gnss`: default `true`
- `use_ntrip_client`: default `true`
- `battery_can_interface`: default `can0`
- `steadydrive_can_interface`: default `can0`
- `imu_port`: default `/dev/imu_usb`
- `imu_baud`: default `9600`
- `imu_params_file`: default `amr_sweeper_imu/config/amr_sweeper_imu.yaml`
- `ros2_control_config_file`: default `amr_sweeper_description/urdf/control/ros2_control.yaml`

## Overview
The main bringup launch starts the core hardware stack under the default robot root `/amr_sweeper`. Depending on launch arguments, the bringup can then enable the robot description, ros2_control, battery monitor, system information publisher, USB cameras, the depth camera laser-scan conversion, IMU, GNSS rover, and optional NTRIP client. Package-owned sensor namespaces follow the package role below that root, including `/amr_sweeper/imu`, `/amr_sweeper/gnss`, `/amr_sweeper/usb_cameras/<camera_name>`, and `/amr_sweeper/depth_camera`.

## Notes
- Use this package when you want to start the whole layer 1 stack from a single command.
- `amr_sweeper_description.launch.py` is the robot description entrypoint used by the main bringup.
- The main bringup directly launches `ros2_control_node` plus the `joint_broad`, `diff_cont`, and `controller_steadydrive` spawners when `use_ros2_control:=true`.
- The inlined ros2_control sequence expects `robot_state_publisher` to publish the robot description and keeps the controller manager subscribed through topic remapping that remains compatible with both ROS 2 Humble and Jazzy.
- The ros2_control controller spawners load controller settings from `amr_sweeper_description/urdf/control/ros2_control.yaml` with `--param-file`.
- The `namespace` argument becomes the robot root, while package-owned sensor namespaces are nested below it, such as `imu`, `gnss`, `usb_cameras`, and `depth_camera`.
- The IMU bringup loads defaults from `amr_sweeper_imu/config/amr_sweeper_imu.yaml`, and `imu_params_file` can swap that file without changing the package launch file.
- When `use_amr_sweeper_gnss:=true` and `use_ntrip_client:=true`, the GNSS wrapper launches the package-local
  NTRIP node under `/amr_sweeper/gnss` and keeps RTCM on
  `/amr_sweeper/gnss/rtcm`.
