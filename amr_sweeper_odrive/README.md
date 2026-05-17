# amr_sweeper_odrive

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides the ODrive integration for the AMR Sweeper wheel drive system.

## Overview
`amr_sweeper_odrive` contains the ros2_control integration used by the robot's wheel drive. It is the package that bridges the wheel hardware into ROS 2 and is the downstream execution target for the layer 2 wheel-controller package.

The package is intentionally structured to match `amr_sweeper_steadydrive` closely: a flat `include/` plus `src/` layout, a dedicated hardware-interface header and source pair, and a velocity-focused ros2_control surface for the two wheel joints.

## Notes
- The ros2_control plugin in this package is the only supported runtime owner of the ODrive CAN hardware.
- Layer 2 wheel control ultimately feeds commands into this package's `diff_cont` controller path.
- Hardware-specific runtime configuration is loaded from `config/amr_sweeper_odrive.yaml`.
- The config file owns the SocketCAN interface, left/right motor IDs, positive motor directions, and the shared gear ratio used by the ODrive hardware interface.
