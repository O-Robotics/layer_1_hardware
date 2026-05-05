# amr_sweeper_system_info_msgs

`ros2 launch amr_sweeper_system_info_msgs system_info_msgs.launch.py`

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides the custom message definitions used by the AMR Sweeper system information publisher.

## Main Launch File
`launch/system_info_msgs.launch.py`

## Available Launch Files
- `system_info_msgs.launch.py`

## Launch Arguments
- None

## Overview
`amr_sweeper_system_info_msgs` is an interface package. It contains the message definitions needed by `amr_sweeper_system_info`, but it does not host a real robot runtime node of its own. The included launch file is only an informational placeholder so the package follows the same launch and README structure as the rest of the stack.

## Notes
- Used by `amr_sweeper_system_info`.
- The launch file does not start any runtime node.
