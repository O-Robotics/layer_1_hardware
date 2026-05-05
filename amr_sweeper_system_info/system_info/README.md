# amr_sweeper_system_info

```bash
ros2 launch amr_sweeper_system_info system_info.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_system_info_msgs`

## Purpose
This package publishes system-health and system-state information for the AMR Sweeper.

## Main Launch File
`launch/system_info.launch.py`

## Available Launch Files
- `system_info.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`

## Overview
`amr_sweeper_system_info` runs the node that collects and publishes robot system state using the message definitions from `amr_sweeper_system_info_msgs`. It is meant to give the wider stack visibility into important runtime status information and is commonly launched through the layer 1 hardware bringup.

## Notes
- Main node: `amr_sweeper_system_info_node`.
- The package depends on `amr_sweeper_system_info_msgs` for its custom message types.
