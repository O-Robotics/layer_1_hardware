# amr_sweeper_system_info_repository

```bash
ros2 launch amr_sweeper_system_info system_info.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_system_info`
- `amr_sweeper_system_info_msgs`

## Purpose
This directory groups the AMR Sweeper system-information runtime package together with its nested interface package under `amr_sweeper_msgs/`.

## Launch Arguments
- `namespace`: default `amr_sweeper`

## Overview
The runtime package `amr_sweeper_system_info` publishes the robot system state, while `amr_sweeper_system_info_msgs` provides the message definitions used by that node. Together they make up the system-information portion of the layer 1 hardware stack.

## Notes
- The actual runtime launch belongs to the package `amr_sweeper_system_info` under `system_info/`.
- The interface package now lives at `src/layer_1_hardware/amr_sweeper_msgs/amr_sweeper_system_info_msgs/`.
