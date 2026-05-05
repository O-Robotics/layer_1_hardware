# amr_sweeper_gnss

```bash
ros2 launch amr_sweeper_gnss ublox_rover_hpposllh_navsatfix.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides the GNSS stack for the AMR Sweeper, including rover launches and optional NTRIP client support.

## Main Launch File
`launch/ublox_rover_hpposllh_navsatfix.launch.py`

## Available Launch Files
- `ntrip_client.launch.py`
- `ublox_mb+r_base.launch.py`
- `ublox_mb+r_rover.launch.py`
- `ublox_rover_hpposecef.launch.py`
- `ublox_rover_hpposllh.launch.py`
- `ublox_rover_hpposllh_navsatfix.launch.py`
- `ublox_rover_hpposllh_satellite.launch.py`

## Launch Arguments
- `use_ublox_dgnss_node`: default `true`
- `use_ublox_nav_sat_fix_hp`: default `true`
- `namespace`: default ``""` (empty string)`

## Overview
`amr_sweeper_gnss` consolidates the GNSS-related functionality for the robot into one package. It includes launch files for rover and base modes as well as the NTRIP client path needed for correction data. In the current stack, the main rover launch is the typical entrypoint used by the layer 1 bringup.

## Notes
- This package is normally launched through layer 1 bringup rather than by itself.
- Layer 3 localization depends on the GNSS topics produced by this package.
