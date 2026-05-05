# amr_sweeper_odrive

```bash
ros2 launch amr_sweeper_odrive odrive.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides the ODrive integration for the AMR Sweeper wheel drive system.

## Main Launch File
`launch/odrive.launch.py`

## Available Launch Files
- `odrive.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `interface`: default `can0`
- `node_id`: default `0`

## Overview
`amr_sweeper_odrive` contains the standalone ODrive CAN node as well as the ros2_control integration used by the robot's wheel drive. It is the package that bridges the wheel hardware into ROS 2 and is the downstream execution target for the layer 2 wheel-controller package.

## Notes
- Main node: `odrive_node`.
- The ros2_control plugin in this package is used by the layer 1 ros2_control launch.
- Layer 2 wheel control ultimately feeds commands into this package's `diff_cont` controller path.
