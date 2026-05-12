# amr_sweeper_microros

```bash
ros2 launch amr_sweeper_microros microros_agent.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package vendors and launches the micro-ROS agent used by the AMR Sweeper.

## Main Launch File
`launch/microros_agent.launch.py`

## Available Launch Files
- `microros_agent.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_microros`: default `true`
- `can_interface`: default `can0`
- `request_id_min`: default `0x500`
- `request_id_max`: default `0x57F`
- `reply_id_offset`: default `0x80`
- `same_id_reply`: default `false`
- `verbosity`: default `4`

## Overview
`amr_sweeper_microros` is a vendor-style layer 1 package. During build it fetches and builds the upstream Micro XRCE-DDS Agent library, then builds an AMR Sweeper specific classic-CAN custom transport executable on top of the official `CustomAgent` API.

## Notes
- The launch starts `amr_sweeper_microros_classic_can_agent`, not the stock `micro_ros_agent` executable.
- The custom transport uses Linux SocketCAN with classic CAN frames and a simple XRCE packet fragmentation scheme over 11-bit CAN identifiers.
- By default, client request frames are accepted in the CAN-ID range `0x500` to `0x57F`, and agent replies are sent back on request ID plus `0x80`.
- Set `same_id_reply:=true` if the MCU transport uses the same CAN identifier in both directions.
- The first build is heavy because the vendor step fetches and builds Micro XRCE-DDS Agent together with its upstream middleware dependencies before installing the custom AMR Sweeper agent executable into the workspace.
- The vendor build defaults to `MICROROS_VENDOR_JOBS=2` to keep the upstream Fast DDS compile from exhausting memory on smaller build machines.
