# layer_1_hardware

ROS 2 hardware-layer workspace for the AMR Sweeper platform.

```bash
ros2 launch amr_sweeper_layer_1_hardware_bringup amr_sweeper_layer_1_hardware_bringup.launch.py
```

This repository currently contains:

- `_amr_sweeper_layer_1_hardware_bringup`: repo-level bringup package
- `amr_sweeper_system_info_msgs`: system state message package
- `amr_sweeper_battery`: Daly BMS over SocketCAN
- `amr_sweeper_usb_cameras`: MJPEG USB camera drivers
- `amr_sweeper_system_info`: system state publisher

## Package Overview

### `_amr_sweeper_layer_1_hardware_bringup`

Launches the packages that live in this repository under:

`/amr_sweeper`

With default arguments, the bringup starts:

- `amr_sweeper_battery_node`
- `amr_sweeper_system_info_node`
- all enabled camera nodes from `amr_sweeper_usb_cameras`

### `amr_sweeper_battery`

Publishes battery state and battery diagnostics from a Daly BMS over Linux SocketCAN.

Key topics:

- `battery_state`
- `battery_health`

### `amr_sweeper_usb_cameras`

Runs up to five MJPEG USB camera nodes using package-provided parameter files.

Configured cameras:

- `front_left_camera`
- `front_right_camera`
- `rear_left_camera`
- `rear_right_camera`
- `tools_camera`

### `amr_sweeper_system_info`

Publishes `amr_sweeper_system_info_msgs/msg/SystemState` based on files under:

`/opt/robot_config/monitoring/`

Key topic:

- `system_info`

## Prerequisites

- ROS 2 Humble installed at `/opt/ros/humble`
- `colcon`
- Linux SocketCAN available for the battery package
- V4L2 MJPEG cameras available for the camera package

For the battery package, the CAN interface typically needs to be brought up first:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To build only this repo's packages:

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select \
  amr_sweeper_layer_1_hardware_bringup \
  amr_sweeper_battery \
  amr_sweeper_usb_cameras \
  amr_sweeper_system_info \
  amr_sweeper_system_info_msgs \
  --symlink-install
source install/setup.bash
```

## Full Bringup

Launch the full hardware layer:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amr_sweeper_layer_1_hardware_bringup amr_sweeper_layer_1_hardware_bringup.launch.py
```

This starts the hardware layer in the default namespace:

`/amr_sweeper`

### Useful Bringup Arguments

Disable one or more subsystems:

```bash
ros2 launch amr_sweeper_layer_1_hardware_bringup amr_sweeper_layer_1_hardware_bringup.launch.py \
  use_battery_node:=false \
  use_system_info_node:=true \
  use_usb_cameras:=true
```

Use a different top-level namespace:

```bash
ros2 launch amr_sweeper_layer_1_hardware_bringup amr_sweeper_layer_1_hardware_bringup.launch.py \
  robot_namespace:=robot_17
```

Change log level:

```bash
ros2 launch amr_sweeper_layer_1_hardware_bringup amr_sweeper_layer_1_hardware_bringup.launch.py \
  log_level:=debug
```

## Run Individual Packages

### Battery

Run the battery node directly:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amr_sweeper_battery amr_sweeper_battery_node
```

Override the CAN interface or polling period:

```bash
ros2 run amr_sweeper_battery amr_sweeper_battery_node --ros-args \
  -p can_interface:=can0 \
  -p timer_period:=15.0
```

### USB Cameras

Launch all configured cameras:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amr_sweeper_usb_cameras amr_sweeper_usb_cameras.launch.py
```

Disable cameras that are not connected:

```bash
ros2 launch amr_sweeper_usb_cameras amr_sweeper_usb_cameras.launch.py \
  rear_left_enabled:=false \
  rear_right_enabled:=false
```

Run one camera node directly with a parameter file:

```bash
ros2 run amr_sweeper_usb_cameras amr_sweeper_usb_cameras_node \
  --ros-args \
  --params-file $(pwd)/amr_sweeper_usb_cameras/config/tools_camera_params.yaml
```

### System Info

Run the system info node directly:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amr_sweeper_system_info amr_sweeper_system_info_node
```

Note:

`amr_sweeper_system_info_node` expects its source files under `/opt/robot_config/` to exist. If they are missing, the node will log file-open errors and will not publish complete data.

## Quick Verification

After bringup or a direct package launch, these commands are useful:

List nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

Inspect battery output:

```bash
ros2 topic echo /amr_sweeper/battery_state
ros2 topic echo /amr_sweeper/battery_health
```

Inspect system info output:

```bash
ros2 topic echo /amr_sweeper/system_info
```

Inspect compressed camera output:

```bash
ros2 topic list | grep image_raw
ros2 topic hz /amr_sweeper/tools_camera/image_raw/compressed
```

## Package-Specific Docs

- [amr_sweeper_battery/README.md](./amr_sweeper_battery/README.md)
- [amr_sweeper_usb_cameras/README.md](./amr_sweeper_usb_cameras/README.md)
- [amr_sweeper_system_info/README.md](./amr_sweeper_system_info/README.md)

## Notes

- The bringup package name is `amr_sweeper_layer_1_hardware_bringup`, even though the directory is currently `_amr_sweeper_layer_1_hardware_bringup`.
- The camera launcher is included by the repo bringup and inherits the same namespace.
- `amr_sweeper_system_info` is built from `amr_sweeper_system_info/system_info`, and `amr_sweeper_system_info_msgs` is built from `amr_sweeper_system_info/system_info_msgs`.
