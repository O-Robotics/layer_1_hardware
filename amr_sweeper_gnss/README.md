# amr_sweeper_gnss

```bash
ros2 launch amr_sweeper_gnss amr_sweeper_gnss.launch.py
```

## Overview
`amr_sweeper_gnss` is the AMR Sweeper wrapper package for the upstream
`ublox_dgnss` stack. It keeps the workspace-specific launch entrypoint,
namespace defaults, and NTRIP YAML config, while the actual GNSS
implementation comes from the dependency packages:

- `ublox_dgnss`
- `ublox_dgnss_node`
- `ublox_nav_sat_fix_hp_node`
- `ntrip_client_node`

## External Dependencies
- `ublox_dgnss`: vendored in this workspace under `src/dependencies/ublox_dgnss`
  GitHub: `https://github.com/aussierobots/ublox_dgnss`
- `rtcm_msgs`: vendored in this workspace under `src/dependencies/rtcm_msgs` and required by `ntrip_client_node`
  GitHub: `https://github.com/tilk/rtcm_msgs`
  Example install:

```bash
git clone https://github.com/tilk/rtcm_msgs.git 
git clone https://github.com/aussierobots/ublox_dgnss
```

## Main Launch File
`launch/amr_sweeper_gnss.launch.py`

### Launch Arguments
- `use_ublox_dgnss_node`: default `true`
- `use_ublox_nav_sat_fix_hp`: default `true`
- `use_ntrip_client`: default `true`
- `ntrip_params_file`: default `config/ntrip_client.yaml`
- `namespace`: default `amr_sweeper/gnss`
- `gnss_frame_id`: default `gnss_link`
- `device_family`: default `F9P`
- `device_serial_string`: default `""`
- `log_level`: default `INFO`

This launch starts the standard AMR Sweeper GNSS stack:
- Upstream `ublox_dgnss_node` for the u-blox receiver connection and UBX topic publishing
- Upstream `ublox_nav_sat_fix_hp_node` for converting high-precision u-blox outputs into the `navsat` topic
- Optional upstream `ntrip_client_node` when `use_ntrip_client:=true` for RTCM correction streaming from an NTRIP caster

### Notes
- This package is normally launched through layer 1 bringup rather than by itself.
- Layer 3 localization depends on the GNSS topics produced by this package.
- No GNSS driver source code or custom UBX messages are implemented in this package anymore.
- By default the GNSS stack is namespaced under `amr_sweeper/gnss`, so topics such as `navsat` become `/amr_sweeper/gnss/navsat`.


## Package Launch Options
- `ntrip_client.launch.py`: starts only the NTRIP client component for RTCM correction streaming
- `ublox_rover_hpposllh_navsatfix.launch.py`: starts the rover driver plus NavSat conversion without the package-level GNSS wrapper

The package keeps only the AMR-specific launch entrypoints used by this workspace.
For moving-base, ECEF, or satellite-diagnostic variants, launch the upstream
`ublox_dgnss` package directly.


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
  namespace:=amr_sweeper/gnss \
  ntrip_params_file:=/absolute/path/to/ntrip_client.yaml
```
