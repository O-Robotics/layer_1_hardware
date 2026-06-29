# amr_sweeper_simulation

```bash
ros2 launch amr_sweeper_simulation amr_sweeper_simulation.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_description`

## Purpose
`amr_sweeper_simulation` owns the AMR Sweeper-specific Gazebo simulation assets and launch plumbing. It keeps workspace customizations out of vendored third-party simulation packages so the simulation stack remains stable across upstream resets.

## Main Launch Files
- `launch/amr_sweeper_gazebo.launch.py`
- `launch/amr_sweeper_simulation.launch.py`

## Overview
This package provides the AMR Sweeper simulation world, a Gazebo-specific URDF wrapper built on `amr_sweeper_description`, the Gazebo-to-ROS topic bridge, and the simulated GNSS publisher. Layer 1 bringup uses the Gazebo launch when `use_simulation:=true`.

## Notes
- The robot is spawned from `amr_sweeper_description`, not from a duplicated simulation-only model.
- The bridge contract lives in `config/amr_sweeper_simulation.yaml`.
- The Gazebo IMU, wheel odometry, and command topics use the workspace topic names under `/amr_sweeper/...`.
