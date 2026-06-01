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
- `device_path`: default empty, so the YAML value is used unless explicitly overridden
- `port`: default empty (deprecated compatibility alias)
- `baud`: default empty, so the YAML value is used unless explicitly overridden
- `imu_frame_id`: default empty, so the YAML value is used unless explicitly overridden
- `publish_hz`: default empty, so the YAML value is used unless explicitly overridden
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
- On startup the node reads back the JY901 configuration registers, prints the current device configuration, compares them against the YAML-backed desired configuration, and only updates the registers that differ before printing `IMU configuration complete`.
- The package uses the bundled JY901 datasheet for the write-side register map and the newer official WIT standard protocol documentation for register readback (`0x27` read command and `0x5F` reply).
- `gyroscope_auto_calibration` is treated as a command sent with `0x63`, not as a readable persistent configuration register, because that is how it is described in the bundled JY901 datasheet.
- The driver can parse the IMU's native `0x59` quaternion packet and use it as the primary ROS orientation source when `output_quaternion` is enabled.
- `yaw_offset_deg` applies a software yaw correction before publishing orientation, angular velocity, and linear acceleration. For a 180 degree yaw mounting mismatch, set `yaw_offset_deg: 180.0`.
- If `baud` differs from the sensor's current baud, use `fallback_baud` to tell the node how to reach the sensor before reprogramming it. The datasheet notes that baud/rate related changes may require a module restart or re-power to fully take effect.
- Reading back `algorithm=nine_axis` only confirms the configuration register value. It does not by itself prove that the runtime yaw is magnetically referenced and calibrated.
- Reconnect failures now follow a warn/error/fatal escalation pattern similar to the GNSS NTRIP client, using `retry_attempts_before_error`, `fatal_after_consecutive_errors`, and `max_reconnect_attempts`.

## TODO
- changing the baud rate does work, but it takes a few launches and are genrally unreliable. The process should be looked into and improved. 
