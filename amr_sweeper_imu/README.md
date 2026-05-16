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
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `params_file`: default `config/amr_sweeper_imu.yaml`
- `port`: default `/dev/imu_usb`
- `baud`: default `9600`
- `imu_frame_id`: default `imu_link`
- `publish_hz`: default `10.0`
- `use_imu_node`: default `true`

## Overview
`amr_sweeper_imu` provides the ROS 2 node that reads the physical IMU and publishes the orientation-related data used by the rest of the robot stack. It is a foundational sensor package for localization and is typically started as part of layer 1 bringup.

## Notes
- Main node: `imu_node`.
- Layer 3 localization relies on this package for IMU data.
- Default tunable parameters live in `config/amr_sweeper_imu.yaml`.
- The launch file passes that wildcard YAML directly into the node, then launch arguments such as `port`, `baud`, `imu_frame_id`, and `publish_hz` can override individual values.
- On startup the node can also program JY901 registers for return content, return rate, installation direction, algorithm mode, gyro auto-calibration, LED state, and baud according to the YAML.
- If `baud` differs from the sensor's current baud, use `device_bootstrap_baud` to tell the node how to reach the sensor before reprogramming it.
- Reconnect failures now follow a warn/error/fatal escalation pattern similar to the GNSS NTRIP client, using `retry_attempts_before_error`, `fatal_after_consecutive_errors`, and `max_reconnect_attempts`.
