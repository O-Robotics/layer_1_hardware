# JY901 ROS 2 IMU Driver

Lean ROS 2 C++ driver for the JY901 IMU.

```bash
ros2 launch amr_sweeper_imu imu.launch.py
```

## What It Does

- Opens the IMU on `/dev/imu_usb` by default
- Parses JY901 11-byte serial frames with checksum validation
- Publishes `sensor_msgs/msg/Imu` on `imu/data_raw`
- Retries opening the serial device if it is not available at startup

## Defaults

- `port`: `/dev/imu_usb`
- `baud`: `9600`
- `frame_id`: `imu_link`
- `publish_hz`: `10.0`

## Build

```bash
colcon build --packages-select amr_sweeper_imu --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch amr_sweeper_imu imu.launch.py
```

Example with explicit parameters:

```bash
ros2 launch amr_sweeper_imu imu.launch.py \
  port:=/dev/imu_usb \
  baud:=9600 \
  frame_id:=imu_link \
  publish_hz:=10.0
```

## Run Directly

```bash
ros2 run amr_sweeper_imu imu_node --ros-args \
  -p port:=/dev/imu_usb \
  -p baud:=9600 \
  -p frame_id:=imu_link \
  -p publish_hz:=10.0
```

## Parameters

- `port`: serial device path
- `baud`: serial baud rate
- `frame_id`: frame ID written into the IMU message header
- `publish_hz`: maximum publish rate; values below `1.0` are clamped to `1.0`

## Topics

- Publishes `imu/data_raw` as `sensor_msgs/msg/Imu`

## Notes

- Orientation is derived from the JY901 Euler-angle frame and published as a quaternion.
- Linear acceleration is published in `m/s^2`.
- Angular velocity is published in `rad/s`.
- This package is IMU-only and contains no RViz or visualization components.

## Current Limitations

- The node publishes when it receives an angle frame, so one outgoing IMU message may combine the latest accel, gyro, and angle samples rather than one fully grouped measurement burst.
- IMU covariance matrices are left at the ROS message defaults.
