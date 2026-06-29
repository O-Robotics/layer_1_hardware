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
- `../layer_0_supervisors/amr_sweeper_msgs/amr_sweeper_safety_msgs`
- `../layer_0_supervisors/amr_sweeper_msgs/amr_sweeper_system_info_msgs`
- `amr_sweeper_odrive`
- `amr_sweeper_ros2_control`
- `amr_sweeper_simulation`
- `amr_sweeper_steadydrive`
- `amr_sweeper_system_info`
- `amr_sweeper_usb_cameras`

## Purpose
This repository is the hardware-adjacent runtime layer for the AMR Sweeper. It contains the packages that expose the physical robot model, wheel and tool motor interfaces, the shared `ros2_control` runtime that activates them, battery monitoring, GNSS, IMU, USB cameras, depth-camera laser-scan conversion, system-health publishing, and the workspace-owned Gazebo simulation package.

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `log_level`: default `info`
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
Layer 1 is the base runtime layer for the rest of the stack. It is responsible for making the robot's hardware available as ROS 2 topics, services, and ros2_control interfaces. Layer 2 controllers and layer 3 navigation depend on this layer to provide odometry, transforms, actuator interfaces, and sensor data.

## Notes
- The default command launches the full layer 1 hardware bringup package.
- `amr_sweeper_layer_1_hardware_bringup` is now a launch-only package that includes the package launch files for sensors, `robot_state_publisher`, and `ros2_control`.
- `amr_sweeper_simulation` owns the AMR Sweeper-specific Gazebo world, model, and ROS bridge plumbing so upstream simulation dependencies can stay stock.
- Layer 2 and layer 3 should be started only after the required layer 1 hardware interfaces are available.
- The vendored RealSense ROS packages live under `amr_sweeper_depth_camera/src/realsense-ros`, but `colcon` will only discover them when they are also exposed at the workspace root `src/`. Run `src/layer_1_hardware/amr_sweeper_depth_camera/scripts/ensure_workspace_links.sh` before `colcon build`, or use `./update.sh`, which now does this automatically.
- Under the default robot root `/amr_sweeper`, package-owned sensor namespaces follow the package role, including `/amr_sweeper/imu`, `/amr_sweeper/gnss`, `/amr_sweeper/usb_cameras`, and `/amr_sweeper/depth_camera`.
- The layer 1 bringup now forwards the full standalone `amr_sweeper_depth_camera.launch.py` argument set into the depth-camera include path, so depth-camera behavior stays aligned unless layer-1-specific overrides are passed explicitly.
- When enabled, the GNSS `ntrip_client_node` runs inside `/amr_sweeper/gnss` and
  publishes RTCM on `/amr_sweeper/gnss/ntrip_client/rtcm`.
- The layer 1 ros2_control bringup publishes `/amr_sweeper/description/robot_description` via `robot_state_publisher` under the default namespace and lets `ros2_control_node` subscribe to that topic using the ROS 2 Jazzy controller-manager path.
- Controller spawners in the layer 1 ros2_control launch load controller settings from `amr_sweeper_description/urdf/control/ros2_control.yaml`, which keeps the bringup aligned with ROS 2 Jazzy while remaining workable on Humble.
- The robot description entrypoint is `amr_sweeper_description.launch.py`, which also owns the default controller-config path used by the bringup.
- Hardware-specific wheel and tool-motor parameters such as CAN interface, motor IDs, positive motor directions, and gear ratios now live in `amr_sweeper_odrive/config/amr_sweeper_odrive.yaml` and `amr_sweeper_steadydrive/config/amr_sweeper_steadydrive.yaml`.
