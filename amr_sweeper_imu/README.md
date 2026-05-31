# amr_sweeper_imu

```bash
ros2 launch amr_sweeper_imu amr_sweeper_imu.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package runs the JY901 IMU driver used by the AMR Sweeper.

## Main Launch File
`launch/amr_sweeper_imu.launch.py`

## Available Launch Files
- `amr_sweeper_imu.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper/imu`
- `use_sim_time`: default `false`
- `params_file`: default `config/amr_sweeper_imu.yaml`
- `device_path`: default `/dev/imu_usb`
- `port`: default `/dev/imu_usb` (deprecated compatibility alias)
- `baud`: default `9600`
- `imu_frame_id`: default `imu_link`
- `publish_hz`: default `10.0`
- `yaw_offset_deg`: default `0.0`
- `use_imu_node`: default `true`

## Overview
`amr_sweeper_imu` provides the ROS 2 node that reads the physical IMU and publishes the orientation-related data used by the rest of the robot stack. It is a foundational sensor package for localization and is typically started as part of layer 1 bringup.

## Notes
- Main node: `imu_node`.
- Layer 3 localization relies on this package for IMU data.
- Default tunable parameters live in `config/amr_sweeper_imu.yaml`.
- The node publishes three IMU topics under the selected namespace:
- `data_raw`: full corrected IMU message with orientation, gyro, and acceleration. With the default namespace this resolves to `/amr_sweeper/imu/data_raw`.
- `data_acc_gyro`: acceleration and gyro only, with orientation marked unavailable. With the default namespace this resolves to `/amr_sweeper/imu/data_acc_gyro`.
- `data_heading`: yaw-only orientation derived from the same IMU, with roll and pitch removed. With the default namespace this resolves to `/amr_sweeper/imu/data_heading`.
- The launch file passes that wildcard YAML directly into the node, then launch arguments such as `device_path`, `baud`, `imu_frame_id`, and `publish_hz` can override individual values.
- For robot deployments, prefer a stable `/dev/serial/by-id/...` path in `device_path` if the IMU exposes one. The older `/dev/imu_usb` symlink still works as a compatibility path.
- On startup the node can also program JY901 registers for return content, return rate, installation direction, algorithm mode, gyro auto-calibration, LED state, and baud according to the YAML.
- The driver can parse the IMU's native `0x59` quaternion packet and use it as the primary ROS orientation source when `output_quaternion` is enabled.
- `yaw_offset_deg` applies a software yaw correction before publishing orientation, angular velocity, and linear acceleration. For a 180 degree yaw mounting mismatch, set `yaw_offset_deg: 180.0`.
- If `baud` differs from the sensor's current baud, use `fallback_baud` to tell the node how to reach the sensor before reprogramming it.
- Reconnect failures now follow a warn/error/fatal escalation pattern similar to the GNSS NTRIP client, using `retry_attempts_before_error`, `fatal_after_consecutive_errors`, and `max_reconnect_attempts`.
