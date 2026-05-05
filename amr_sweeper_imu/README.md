# amr_sweeper_imu

```bash
ros2 launch amr_sweeper_imu imu.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package runs the JY901 IMU driver used by the AMR Sweeper.

## Main Launch File
`launch/imu.launch.py`

## Available Launch Files
- `imu.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `port`: default `/dev/imu_usb`
- `baud`: default `9600`
- `frame_id`: default `imu_link`
- `publish_hz`: default `10.0`
- `use_imu_node`: default `true`

## Overview
`amr_sweeper_imu` provides the ROS 2 node that reads the physical IMU and publishes the orientation-related data used by the rest of the robot stack. It is a foundational sensor package for localization and is typically started as part of layer 1 bringup.

## Notes
- Main node: `imu_node`.
- Layer 3 localization relies on this package for IMU data.
