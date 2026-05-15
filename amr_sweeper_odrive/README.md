# amr_sweeper_odrive

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides the ODrive integration for the AMR Sweeper wheel drive system.

## Overview
`amr_sweeper_odrive` contains the ros2_control integration used by the robot's wheel drive. It is the package that bridges the wheel hardware into ROS 2 and is the downstream execution target for the layer 2 wheel-controller package.

## Notes
- The ros2_control plugin in this package is the only supported runtime owner of the ODrive CAN hardware.
- Layer 2 wheel control ultimately feeds commands into this package's `diff_cont` controller path.
