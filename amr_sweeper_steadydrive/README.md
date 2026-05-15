# amr_sweeper_steadydrive

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides the SteadyDrive motor interface used for the tool-side drive system on the AMR Sweeper.

## Overview
`amr_sweeper_steadydrive` contains the ros2_control hardware-interface integration for the two SteadyDrive motors. In the current stack, this package is started by the layer 1 hardware bringup and receives command data from the layer 2 tool-controller package.

## Notes
- The ros2_control plugin in this package is the only supported runtime owner of the SteadyDrive CAN hardware.
- Layer 2 tool control publishes into the controller command path exposed by this package.
