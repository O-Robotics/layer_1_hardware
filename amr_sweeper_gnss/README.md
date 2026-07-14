# amr_sweeper_gnss

```bash
ros2 launch amr_sweeper_gnss amr_sweeper_gnss.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package contains the AMR Sweeper GNSS stack: a local C++ GNSS receiver node plus the local C++ NTRIP client node. In simulation, the same `gnss_node` executable consumes the simulated robot pose and publishes receiver-style GNSS topics.

## Main Launch File
`launch/amr_sweeper_gnss.launch.py`

## Available Launch Files
- `amr_sweeper_gnss.launch.py`
- `ntrip_client.launch.py`
- `ublox_dgnss.launch.py`

## Launch Arguments
- `use_ublox_dgnss_node`: default `true`
- `use_ublox_nav_sat_fix_hp`: default `true` (deprecated compatibility argument; the local node publishes `navsat` directly)
- `use_ntrip_client`: default `true`
- `use_nmea_to_caster`: default `true`
- `use_simulation`: default `false`
- `robot_pose_topic`: default `""` (`gnss_node` falls back to `/amr_sweeper/simulation/robot_pose` in simulation mode)
- `ublox_params_file`: default `config/amr_sweeper_gnss.yaml`
- `ntrip_params_file`: default `config/amr_sweeper_gnss_ntrip_client.yaml`
- `fix_topic`: default `navsat`
- `gnss_namespace`: default `amr_sweeper/gnss`
- `gnss_frame_id`: default `gnss_link`
- `device_family`: default `F9P` (deprecated compatibility argument)
- `device_serial_string`: default `""` (deprecated compatibility argument)
- `log_level`: default `INFO`
- `ublox_log_level`: default `WARN`

## Overview
`amr_sweeper_gnss` keeps the workspace-specific launch entrypoint, namespace defaults, and package-owned YAML configuration for the robot GNSS stack. The launch starts the local `gnss_node` receiver node and the optional local `ntrip_client_node` for RTCM correction streaming. In hardware mode, `gnss_node` configures the u-blox receiver on connect, subscribes to `ntrip_client/rtcm`, and publishes `navsat` directly. In simulation mode, `gnss_node` subscribes to the configured robot pose topic, converts the local simulation pose to WGS84 latitude/longitude/altitude, and uses simulated RTCM to report GPS/DGPS/RTK states.

## External Dependencies
- `rtcm_msgs`: installed from the ROS Jazzy packages and used by both the
  NTRIP client and the local u-blox driver

Example install:

```bash
sudo apt install ros-jazzy-rtcm-msgs
```

## Notes
- This package is normally launched through layer 1 bringup rather than by itself.
- Layer 3 localization depends on the GNSS topics produced by this package.
- By default the GNSS stack is namespaced under `/amr_sweeper/gnss`, so topics
  such as `navsat` become `/amr_sweeper/gnss/navsat`.
- The `ntrip_client_node` runs in that same GNSS namespace and publishes RTCM on
  `/amr_sweeper/gnss/ntrip_client/rtcm`.
- The local u-blox node subscribes to the GNSS-local `ntrip_client/rtcm` topic so
  corrections still reach the receiver under the AMR namespace layout.
- When `use_nmea_to_caster:=true` in `ntrip_client.launch.py`, the local NTRIP
  node subscribes to the configured `fix_topic` with best-effort QoS and sends
  GGA messages to the caster. The default `fix_topic` is `navsat`, and the
  default `use_nmea_to_caster` value is `true`.
- `ntrip_client.launch.py` starts only the `ntrip_client_node` for RTCM correction streaming.
- `ublox_dgnss.launch.py` launches the same local `gnss_node` in hardware and simulation modes.

## Configuration Files
The GNSS stack uses one YAML file for the local u-blox node and one YAML file for the local NTRIP client.

Installed default config:
- `share/amr_sweeper_gnss/config/amr_sweeper_gnss.yaml`
- `share/amr_sweeper_gnss/config/amr_sweeper_gnss_ntrip_client.yaml`

Source config in the repo:
- `config/amr_sweeper_gnss.yaml`
- `config/amr_sweeper_gnss_ntrip_client.yaml`

Example standalone launch:

```bash
ros2 launch amr_sweeper_gnss amr_sweeper_gnss.launch.py \
  gnss_namespace:=amr_sweeper/gnss \
  use_nmea_to_caster:=true \
  fix_topic:=navsat \
  ublox_params_file:=$(realpath config/amr_sweeper_gnss.yaml) \
  ntrip_params_file:=$(realpath config/amr_sweeper_gnss_ntrip_client.yaml)
```

Useful u-blox parameters in `config/amr_sweeper_gnss.yaml`:
- `device_path` and `baud_rate` for the local receiver connection
- receiver protocol enablement for UBX and RTCM on the USB CDC link
- measurement and navigation rates
- the three UBX messages the local node consumes: HPPOSLLH, STATUS, and COV
- NavSat filtering thresholds and covariance scaling used before publishing `navsat`

Example standalone NTRIP launch:

```bash
ros2 launch amr_sweeper_gnss ntrip_client.launch.py \
  gnss_namespace:=amr_sweeper/gnss \
  params_file:=$(realpath config/amr_sweeper_gnss_ntrip_client.yaml) \
  fix_topic:=navsat \
  use_nmea_to_caster:=true
```

Useful NTRIP parameters in `config/amr_sweeper_gnss_ntrip_client.yaml`:
- `alternate_mountpoint`: optional backup mountpoint the node will try after a failed connection or dropped RTCM stream
- `mountpoint_failover_threshold`: number of consecutive failures on the active mountpoint before the node switches to the backup mountpoint
- `startup_retry_seconds`: wait time before retrying node startup after a startup/config failure
- `failed_connection_retry_seconds`: wait time before reconnecting after a failed connection attempt or dropped stream
- `reconnect_attempt_wait_seconds`: legacy compatibility fallback used when `failed_connection_retry_seconds` is not positive
- `socket_timeout_seconds`: socket read/connect timeout for the TCP session
- `initial_rtcm_grace_seconds`: extra grace window after a successful stream header before the first valid RTCM is required
- `rtcm_timeout_seconds`: reconnect when a connected session stops delivering valid RTCM for this long
- `retry_attempts_before_error`: number of consecutive connection-loss or bad-RTCM warnings before the node escalates to error logs
- `fatal_after_consecutive_errors`: number of consecutive startup or runtime failures before the node exits fatally
