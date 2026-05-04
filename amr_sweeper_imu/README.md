# WIT ROS2 IMU Driver (C++)

ROS 2 C++ driver for WIT IMU devices. Publishes `sensor_msgs/msg/Imu` to:

- `/imu/data_raw`

## Build

```bash
colcon build --packages-select amr_sweeper_imu
source install/setup.bash
```

## Run

```bash
ros2 launch amr_sweeper_imu rviz_and_imu.launch.py
```

## Parameters

- `port` (string, default: `/dev/imu_usb`)
- `baud` (int, default: `115200`)
- `frame_id` (string, default: `imu_link`)
- `publish_hz` (double, default: `50.0`)

Example:

```bash
ros2 run amr_sweeper_imu amr_sweeper_imu_node --ros-args \
  -p port:=/dev/imu_usb \
  -p baud:=115200 \
  -p publish_hz:=50.0
```

## Notes

- Driver parses WIT 11-byte frames (`0x55` header).
- Acceleration output is in `m/s^2`, angular velocity in `rad/s`, orientation as quaternion.
- If serial is unavailable at startup, the node retries periodically.
