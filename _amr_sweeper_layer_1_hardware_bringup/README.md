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
This package is the main entrypoint for the AMR Sweeper hardware layer. It is a launch-only package that gathers the hardware-related packages in layer 1 and starts them through package launch includes.

## Main Launch File
`launch/amr_sweeper_layer_1_hardware_bringup.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `log_level`: default `info`
- `realsense_log_level`: default `error`
- `ublox_log_level`: default `WARN`
- `use_sim_time`: default `false`
- `use_amr_sweeper_description`: default `true`
- `use_amr_sweeper_ros2_control`: default `true`
- `use_amr_sweeper_battery`: default `true`
- `use_amr_sweeper_system_info`: default `true`
- `use_amr_sweeper_usb_cameras`: default `true`
- `use_amr_sweeper_depth_camera`: default `true`
- `use_amr_sweeper_imu`: default `true`
- `use_amr_sweeper_gnss`: default `true`
- `use_ntrip_client`: default `true`
- `battery_can_interface`: default `can0`
- `battery_params_file`: default `amr_sweeper_battery/config/amr_sweeper_battery.yaml`
- `system_info_params_file`: default `amr_sweeper_system_info/config/amr_sweeper_system_info.yaml`
- `depth_camera_params_file`: default `amr_sweeper_depth_camera/config/amr_sweeper_depth_camera.yaml`
- `depth_camera_use_laserscan`: default `true`
- `depth_camera_laserscan_params_file`: default `amr_sweeper_depth_camera/config/laserscan.yaml`
- `depth_camera_image_topic`: default `""`
- `depth_camera_info_topic`: default `""`
- `depth_camera_frame`: default `depth_camera_link`
- `depth_camera_scan_topic`: default `scan`
- `depth_camera_output_frame`: default `""`
- `depth_camera_range_min`: default `""`
- `depth_camera_range_max`: default `""`
- `depth_camera_scan_height`: default `""`
- `depth_camera_scan_tilt_angle_deg`: default `""`
- `depth_camera_scan_time`: default `""`
- `depth_camera_camera_domain_id`: default `5`
- `imu_device_path`: default `/dev/imu_usb`
- `imu_port`: default `/dev/imu_usb` (deprecated compatibility alias)
- `imu_baud`: default package-defined IMU config
- `imu_params_file`: default `amr_sweeper_imu/config/amr_sweeper_imu.yaml`
- `gnss_frame_id`: default `gnss_link`
- `ntrip_params_file`: default `amr_sweeper_gnss/config/amr_sweeper_gnss_ntrip_client.yaml`
- `front_left_camera_enabled`: default `false`
- `front_right_camera_enabled`: default `false`
- `rear_left_camera_enabled`: default `true`
- `rear_right_camera_enabled`: default `true`
- `tools_camera_enabled`: default `true`

## Overview
The main bringup launch starts the core hardware stack under the default robot root `/amr_sweeper`. Depending on launch arguments, the bringup includes the package launch files for robot description, `ros2_control`, battery monitor, system information publisher, USB cameras, the depth camera domain bridge and laser-scan conversion, IMU, GNSS rover, and optional NTRIP client. Package-owned sensor namespaces follow the package role below that root, including `/amr_sweeper/imu`, `/amr_sweeper/gnss`, `/amr_sweeper/usb_cameras/<camera_name>`, and the flattened depth-camera path `/amr_sweeper/depth_camera`.

## Notes
- Use this package when you want to start the whole layer 1 stack from a single command.
- This package no longer builds or runs a dedicated orchestration node; all behavior lives in the launch file and the included package launch files.
- `amr_sweeper_description.launch.py` is the robot description entrypoint used by the main bringup.
- `use_amr_sweeper_ros2_control` enables the layer 1 `ros2_control` runtime bringup alongside the robot description and hardware-related nodes.
- When `use_amr_sweeper_ros2_control:=true`, this bringup always owns `ros2_control_node` plus the `joint_broad` spawner that activates the ODrive and SteadyDrive hardware interfaces.
- The `namespace` argument becomes the robot root, while package-owned sensor namespaces are nested below it, such as `imu`, `gnss`, `usb_cameras`, and `depth_camera`.
- The depth camera bringup passes `namespace:=/amr_sweeper/depth_camera` into the package launch, and that launch bridges the camera's native ROS topics from domain `5` into the flattened `/amr_sweeper/depth_camera/...` topic layout.
- The battery and system-info nodes load defaults from `config/amr_sweeper_battery.yaml` and `config/amr_sweeper_system_info.yaml` before any bringup-level overrides are applied.
- The IMU bringup loads defaults from `amr_sweeper_imu/config/amr_sweeper_imu.yaml`, and `imu_params_file` can swap that file without changing the package launch file.
- When `use_amr_sweeper_gnss:=true` and `use_ntrip_client:=true`, the GNSS wrapper launches the package-local
  NTRIP node under `/amr_sweeper/gnss` and keeps RTCM on
  `/amr_sweeper/gnss/rtcm`.
- Wheel and tool motor hardware parameters are no longer launch arguments on this bringup path; they are loaded by the hardware-interface packages from `amr_sweeper_odrive/config/amr_sweeper_odrive.yaml` and `amr_sweeper_steadydrive/config/amr_sweeper_steadydrive.yaml`.
