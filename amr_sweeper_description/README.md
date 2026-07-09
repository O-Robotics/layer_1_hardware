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
- `enable_seed_publishers`: default `true`
- `seed_publish_rate_hz`: default `2.0`
- `seed_base_link_roll_deg`: default `0.0`
- `seed_base_link_pitch_deg`: default `4.5`
- `seed_base_link_yaw_deg`: default `0.0`
- `seed_base_link_z_m`: default `0.13`

## Overview
`amr_sweeper_description` provides the URDF/Xacro model, meshes, and robot_state_publisher launch setup for the platform. It is used by the hardware layer to expose the robot model and by higher layers that rely on a consistent frame tree and robot description.

It also launches a lightweight description seed publisher by default. That helper keeps the robot visually connected before Layer 2 and Layer 3 are active by publishing seeded runtime transforms and zeroed joint states:
- `map -> odom` as identity
- `odom -> base_footprint` as identity
- `base_footprint -> base_link` from the configured seed roll/pitch/yaw/z values
- `joint_states` for the movable wheel and brush joints at zero position

The seed publisher watches the ROS graph and stops each seeded output as the real owner appears:
- `attitude_controller_node` takes over `base_footprint -> base_link`
- `odometry_projection` takes over `odom -> base_footprint`
- `map_pose_node` takes over `map -> odom`
- any other `joint_states` publisher takes over the seeded movable-joint positions

## Notes
- The package is commonly started as part of `amr_sweeper_layer_1_hardware_bringup`.
- It provides the robot model foundation used by ros2_control and localization.
- In the default hardware bringup, `robot_state_publisher` from this package supplies `/amr_sweeper/description/robot_description` under the default namespace, and ros2_control consumes that same topic.
- In the default hardware bringup, `robot_state_publisher` now publishes the `base_link`-rooted robot model directly, while `amr_sweeper_attitude_controller` owns the dynamic `base_footprint -> base_link` transform once it starts.
- The default robot root namespace is `/amr_sweeper`.
- `amr_sweeper_description.launch.py` owns the default controller-config path used by the layer 1 ros2_control bringup.
- Hardware-specific ODrive and Steadydrive runtime parameters are no longer declared in the description xacros; those values are loaded directly by `amr_sweeper_odrive` and `amr_sweeper_steadydrive` from their package-local config files.
- Each USB camera now exposes a single physical frame named `<camera_name>_link` plus the ROS optical frame `<camera_name>_optical_frame`, with no translation offset between them.
