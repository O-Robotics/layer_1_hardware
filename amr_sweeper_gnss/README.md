# amr_sweeper_gnss

```bash
ros2 launch amr_sweeper_gnss amr_sweeper_gnss.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Overview
`amr_sweeper_gnss` consolidates the GNSS-related functionality for the robot into one package. It includes launch files for rover and base modes as well as the NTRIP client path needed for correction data. The package-level `amr_sweeper_gnss.launch.py` is now the standard entrypoint for both standalone use and layer 1 bringup integration.

## Main Launch File
`launch/amr_sweeper_gnss.launch.py`

### Launch Arguments
- `use_ublox_dgnss_node`: default `true`
- `use_ublox_nav_sat_fix_hp`: default `true`
- `use_ntrip_client`: default `true`
- `ntrip_params_file`: default `config/ntrip_client.yaml`
- `namespace`: default `amr_sweeper`
- `gnss_frame_id`: default `gnss_link`

This launch starts the standard AMR Sweeper GNSS stack:
- `UbloxDGNSSNode` for the u-blox receiver connection and UBX topic publishing
- `UbloxNavSatHpFixNode` for converting high-precision u-blox outputs into the `navsat` topic
- Optional `NTRIPClientNode` when `use_ntrip_client:=true` for RTCM correction streaming from an NTRIP caster


### Notes
- This package is normally launched through layer 1 bringup rather than by itself.
- Layer 3 localization depends on the GNSS topics produced by this package.


## Package Launch Options
- `ntrip_client.launch.py`: starts only the NTRIP client component for RTCM correction streaming
- `ublox_mb+r_base.launch.py`: starts a moving-base configuration for a u-blox base receiver plus NavSat conversion
- `ublox_mb+r_rover.launch.py`: starts a moving-base rover configuration plus NavSat conversion
- `ublox_rover_hpposecef.launch.py`: starts only the u-blox driver publishing high-precision ECEF position data
- `ublox_rover_hpposllh.launch.py`: starts only the u-blox driver publishing high-precision latitude/longitude/height data
- `ublox_rover_hpposllh_navsatfix.launch.py`: starts the rover driver plus NavSat conversion without the package-level GNSS wrapper
- `ublox_rover_hpposllh_satellite.launch.py`: starts a rover-oriented diagnostic configuration with extra satellite, signal, RAWX, MEASX, RTCM, and interference-monitor outputs


## NTRIP Caster Configuration
The NTRIP client launch supports a YAML parameter file for caster details.

Installed default config:
- `share/amr_sweeper_gnss/config/ntrip_client.yaml`

Source config in the repo:
- `config/ntrip_client.yaml`

Example standalone launch:

```bash
export NTRIP_USERNAME=your_user
export NTRIP_PASSWORD=your_password

ros2 launch amr_sweeper_gnss amr_sweeper_gnss.launch.py \
  namespace:=amr_sweeper \
  ntrip_params_file:=/absolute/path/to/ntrip_client.yaml
```

The package-level launch still accepts direct NTRIP overrides such as
`ntrip_host:=...` and `ntrip_mountpoint:=...`. Those launch arguments override values from the
YAML file, which is useful for temporary mountpoint changes or per-machine secrets handling.
