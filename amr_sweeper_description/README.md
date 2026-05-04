# amr_sweeper_description

Slim ROS 2 robot description package for the AMR Sweeper platform.

This package is intentionally focused on the onboard robot model and ROS 2
control description. It keeps the working mesh, frame, and joint layout from
`ROS2_Control-main` while avoiding the larger simulation and demo surface from
other archives.

## Included

- robot meshes
- xacro/URDF description
- hardware-oriented ros2_control xacros
- `rsp.launch.py` for `robot_state_publisher`

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_sweeper_description --symlink-install
```

## Launch

```bash
source install/setup.bash
ros2 launch amr_sweeper_description rsp.launch.py
```
