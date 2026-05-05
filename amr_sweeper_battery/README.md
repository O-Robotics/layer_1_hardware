# amr_sweeper_battery

`ros2 launch amr_sweeper_battery battery.launch.py`

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package publishes battery-state and battery-health information for the AMR Sweeper from the battery CAN interface.

## Main Launch File
`launch/battery.launch.py`

## Available Launch Files
- `battery.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `can_interface`: default `can0`

## Overview
`amr_sweeper_battery` wraps the battery node that talks to the physical battery subsystem over CAN and exposes the information into ROS 2. It is typically launched as part of the layer 1 hardware bringup, but it can also be launched on its own for battery diagnostics or integration testing.

## Notes
- Main node: `amr_sweeper_battery_node`.
- This package is a sensor and monitoring package and does not control motion.
