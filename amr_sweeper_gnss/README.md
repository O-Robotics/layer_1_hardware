# amr_sweeper_gnss

```bash
ros2 launch amr_sweeper_gnss amr_sweeper_gnss.launch.py
```

## Overview
`amr_sweeper_gnss` is the AMR Sweeper wrapper package for the upstream
`ublox_dgnss` rover stack plus an AMR-local NTRIP ROS 2 node in C++. It keeps
the workspace-specific launch entrypoint, namespace defaults, and NTRIP
YAML config, while the GNSS receiver implementation comes from the
dependency packages:

- `ublox_dgnss`
- `ublox_dgnss_node`
- `ublox_nav_sat_fix_hp_node`
- `amr_sweeper_gnss` local `ntrip_client`

## External Dependencies
- `ublox_dgnss`: vendored in this workspace under `src/dependencies/ublox_dgnss`
  GitHub: `https://github.com/aussierobots/ublox_dgnss`
- `rtcm_msgs`: installed from the ROS Jazzy packages and used by both the
  NTRIP client and the u-blox driver

Example install:

```bash
git clone https://github.com/aussierobots/ublox_dgnss
sudo apt install ros-jazzy-rtcm-msgs
```

## Main Launch File
`launch/amr_sweeper_gnss.launch.py`

### Launch Arguments
- `use_ublox_dgnss_node`: default `true`
- `use_ublox_nav_sat_fix_hp`: default `true`
- `use_ntrip_client`: default `true`
- `ublox_params_file`: default `config/ublox_dgnss.yaml`
- `ntrip_params_file`: default `config/ntrip_client.yaml`
- `gnss_namespace`: default `amr_sweeper/gnss`
- `gnss_frame_id`: default `gnss_link`
- `device_family`: default `F9P`
- `device_serial_string`: default `""`
- `log_level`: default `INFO`

This launch starts the standard AMR Sweeper GNSS stack:
- Upstream `ublox_dgnss_node` for the u-blox receiver connection and UBX topic publishing
- Upstream `ublox_nav_sat_fix_hp_node` for converting high-precision u-blox outputs into the `navsat` topic
- Optional AMR-local `ntrip_client` node when `use_ntrip_client:=true`
  for RTCM correction streaming from an NTRIP caster

### Notes
- This package is normally launched through layer 1 bringup rather than by itself.
- Layer 3 localization depends on the GNSS topics produced by this package.
- No GNSS driver source code or custom UBX messages are implemented in this package anymore.
- By default the GNSS stack is namespaced under `/amr_sweeper/gnss`, so topics
  such as `navsat` become `/amr_sweeper/gnss/navsat`.
- The NTRIP client runs in that same GNSS namespace and publishes RTCM on
  `/amr_sweeper/gnss/rtcm`.
- The upstream `ublox_dgnss_node` subscription to `/ntrip_client/rtcm` is
  remapped in launch to the GNSS-local `rtcm` topic so corrections still reach
  the receiver under the AMR namespace layout.
- When `use_nmea_to_caster:=true` in `ntrip_client.launch.py`, the local NTRIP
  node subscribes to `fix` with best-effort QoS and sends GGA messages to the
  caster. The default is `false`.

## Package Launch Options
- `ntrip_client.launch.py`: starts only the NTRIP client node for RTCM
  correction streaming
- `ublox_dgnss.launch.py`: starts the rover driver plus NavSat conversion without the package-level GNSS wrapper

The package keeps only the AMR-specific launch entrypoints used by this workspace.
For moving-base, ECEF, or satellite-diagnostic variants, launch the upstream
`ublox_dgnss` package directly.


## NTRIP Caster Configuration
The NTRIP client launch supports a YAML parameter file for caster details.

Installed default config:
- `share/amr_sweeper_gnss/config/ublox_dgnss.yaml`
- `share/amr_sweeper_gnss/config/ntrip_client.yaml`

Source config in the repo:
- `config/ublox_dgnss.yaml`
- `config/ntrip_client.yaml`

Example standalone launch:

```bash
ros2 launch amr_sweeper_gnss amr_sweeper_gnss.launch.py \
  gnss_namespace:=amr_sweeper/gnss \
  ublox_params_file:=/absolute/path/to/ublox_dgnss.yaml \
  ntrip_params_file:=/absolute/path/to/ntrip_client.yaml
```

Useful u-blox parameters in `config/ublox_dgnss.yaml`:
- receiver protocol enablement for USB RTCM/NMEA
- measurement and navigation rates
- enabled GNSS constellations and signals
- published UBX message rates
- NavSatFix quality thresholds and QoS overrides for `ublox_nav_sat_fix_hp`

Example standalone NTRIP launch with optional caster GGA uplink:

```bash
ros2 launch amr_sweeper_gnss ntrip_client.launch.py \
  gnss_namespace:=amr_sweeper/gnss \
  params_file:=/absolute/path/to/ntrip_client.yaml \
  use_nmea_to_caster:=true
```

Useful NTRIP parameters in `config/ntrip_client.yaml`:
- `alternate_mountpoint`: optional backup mountpoint the node will try after a failed connection or dropped RTCM stream
- `mountpoint_failover_threshold`: number of consecutive failures on the active mountpoint before the node switches to the backup mountpoint
- `startup_retry_seconds`: wait time before retrying node startup after a startup/config failure
- `failed_connection_retry_seconds`: wait time before reconnecting after a failed connection attempt or dropped stream
- `reconnect_attempt_wait_seconds`: legacy compatibility fallback used when `failed_connection_retry_seconds` is not positive
- `socket_timeout_seconds`: socket read/connect timeout for the TCP session
- `rtcm_timeout_seconds`: reconnect when a connected session stops delivering valid RTCM for this long
- `retry_attempts_before_error`: number of consecutive connection-loss or bad-RTCM warnings before the node escalates to error logs
- `fatal_after_consecutive_errors`: number of consecutive startup or runtime failures before the node exits fatally
