# amr_sweeper_steadydrive

```bash
ros2 launch amr_sweeper_steadydrive steadydrive.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides the SteadyDrive motor interface used for the tool-side drive system on the AMR Sweeper.

## Main Launch File
`launch/steadydrive.launch.py`

## Available Launch Files
- `steadydrive.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `can_interface`: default `can0`
- `left_motor_can_id`: default `0x141`
- `right_motor_can_id`: default `0x142`

## Overview
`amr_sweeper_steadydrive` contains the runtime nodes and hardware-interface integration for the two SteadyDrive motors. In the current stack, this package is started by the layer 1 hardware bringup and receives command data from the layer 2 tool-controller package.

## Notes
- Starts the left and right SteadyDrive CAN nodes.
- Layer 2 tool control publishes into the controller command path exposed by this package.
