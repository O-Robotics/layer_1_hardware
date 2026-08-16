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
- Hardware-specific runtime configuration is loaded from `config/amr_sweeper_steadydrive.yaml`.
- The config file owns the SocketCAN interface, left/right motor IDs, positive motor directions, and the shared gear ratio used by the SteadyDrive hardware interface.
- The hardware interface now exports `effort` plus additional protection/telemetry state interfaces for torque/current proxy, current, temperature, voltage, and latched-fault reporting.
- SteadyDrive protection uses the `0x9C` status frame for temperature, speed, encoder position, and current-based torque proxy telemetry, plus the `0x9A` state frame for bus voltage and motor error-state readiness checks.
- `0x88` (motor run), `0x81` (motor stop), and `0x80` (motor off) are explicit state-set commands, not toggles. Per the LK-TECH CAN protocol (`resources/Motor control protocol(CAN) V2.35.pdf`), there is no readable status bit for on/off/run state anywhere in the protocol: the only status byte (`errorState`, returned by the `0x9A` state-1 query) reports solely under-voltage and over-temperature faults, so the interface cannot query whether a motor is currently on or off.
- `0x88` is only sent when the hardware is actually brought up or explicitly recovered: once per joint in `on_activate`, again if the SocketCAN connection is re-established while active (`ensureCanSockets`), and when `clear_safety_stop`/`reset_latched_stop` explicitly re-enables the motors after a latched safety stop. It is deliberately not resent on every `write()` cycle, to avoid doubling CAN bus traffic for these motors on every control tick. This means a motor disabled by an out-of-band `0x80`/`0x81` from another CAN node (e.g. the physical stop-button module) only comes back on through one of those explicit recovery paths, not automatically on the next mission — see `amr_sweeper_safety_controller`'s `external_can_stop_monitor_enabled` for the mechanism that latches a safety stop (and thus drives that recovery) when such a command is observed.
