# amr_sweeper_battery

```bash
ros2 launch amr_sweeper_battery amr_sweeper_battery.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package publishes battery-state and battery-health information for the AMR Sweeper from the battery CAN interface.

## Main Launch File
`launch/amr_sweeper_battery.launch.py`

## Available Launch Files
- `amr_sweeper_battery.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `params_file`: default `<package_share>/config/amr_sweeper_battery.yaml`
- `can_interface`: default `can0`

## Overview
`amr_sweeper_battery` wraps the battery node that talks to the physical battery subsystem over CAN and exposes the information into ROS 2. It is typically launched as part of the layer 1 hardware bringup, but it can also be launched on its own for battery diagnostics or integration testing.

## Notes
- Main node: `amr_sweeper_battery_node`.
- Default package parameters live in `config/amr_sweeper_battery.yaml`.
- This package is a sensor and monitoring package and does not control motion.
