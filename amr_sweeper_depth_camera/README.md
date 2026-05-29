# amr_sweeper_depth_camera

```bash
ros2 launch amr_sweeper_depth_camera amr_sweeper_depth_camera.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package is a standalone ROS 2 wrapper around the D555's native ROS interfaces. It bridges the camera's native topics and control services from camera domain `5` into the AMR Sweeper workspace domain `0`, monitors bridge health, and runs the local depth-image-to-laserscan conversion node.

## Main Launch File
`launch/amr_sweeper_depth_camera.launch.py`

## Available Launch Files
- `amr_sweeper_depth_camera.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper/depth_camera`
- `log_level`: default `info`
- `use_sim_time`: default `false`
- `use_domain_bridge`: default `true`
- `camera_domain_id`: default `5`
- `workspace_domain_id`: default `0`
- `source_root_namespace`: default `/realsense`
- `source_camera_model`: default `D555`
- `source_camera_id`: default empty, which auto-discovers the serial-specific camera ID on the source domain
- `params_file`: default `config/amr_sweeper_depth_camera.yaml`
- `use_laserscan`: default `true`
- `laserscan_params_file`: default `config/laserscan.yaml`
- `depth_image_topic`: default derived from `namespace`, e.g. `/amr_sweeper/depth_camera/depth`
- `depth_camera_info_topic`: default derived from `namespace`, e.g. `/amr_sweeper/depth_camera/depth/camera_info`
- `depth_camera_frame`: default `depth_camera_link`
- `scan_topic`: default `scan`
- `output_frame`: default from `config/laserscan.yaml` (`laserscan_link`)
- `range_min`: default from `config/laserscan.yaml` (`0.20`)
- `range_max`: default from `config/laserscan.yaml` (`5.0`)
- `scan_height`: default from `config/laserscan.yaml` (`5`)
- `scan_tilt_angle_deg`: default from `config/laserscan.yaml` (`4.5`)
- `scan_time`: default from `config/laserscan.yaml` (`0.050`)

## Overview
`amr_sweeper_depth_camera` contains the launch configuration needed to discover the serial-specific D555 topic prefix on camera domain `5` and start a single custom `depth_camera_node`. The wrapper is independent of `librealsense` itself: it does not link against or embed the SDK, and instead consumes only the native ROS interfaces already exposed by the D555 side. The node uses the `domain_bridge` C++ API directly to bridge the selected camera topics and services into the flattened AMR Sweeper namespace in the workspace domain, for example `/amr_sweeper/depth_camera/depth`, `/amr_sweeper/depth_camera/depth/camera_info`, and `/amr_sweeper/depth_camera/set_parameters`. The same node also monitors the bridged camera topic set across depth, color, infrared, motion, and compressed color streams.

## Notes
- Main nodes: `depth_camera_node` and `laserscan_node`.
- A `tf2_ros` static transform publisher creates `laserscan_link` under `depth_camera_link`.
- `config/amr_sweeper_depth_camera.yaml` controls topic bridging, source stream activation, optional source profile selection, service bridging, and watchdog monitoring.
- The top-level domain keys are `camera_domain_id` and `workspace_domain_id`, matching the native D555 side and the AMR Sweeper side respectively.
- `config/laserscan.yaml` remains the dedicated config for `laserscan_node`.
- Stream controls in `config/amr_sweeper_depth_camera.yaml` are grouped by sensor family: `use_color`, `use_compressed_color`, `use_depth`, `use_infra1`, `use_infra2`, `use_motion`, and `use_tf_static`.
- Each `use_*` flag now controls both sides together: it enables the corresponding source D555 stream at startup and bridges that stream into `/amr_sweeper/depth_camera`.
- Per-stream profile overrides now use the naming pattern `*_profile_name` for the upstream parameter key and `*_profile_parameter` for the requested profile value, for example `color_profile_name: rgb_camera.profile` and `color_profile_parameter: "1280x800x30:RGB8"`. This now includes `compressed_color_profile_name` and `compressed_color_profile_parameter`.
- The old separate `*_profile_enable` layer is gone. If `use_color` is true, the wrapper enables the source color stream, applies the configured color profile, and bridges the resulting topics together.
- The default profiles now bias toward the lowest listed resolution and FPS to reduce DDS / ROS bandwidth by default.
- The watchdog is demand-aware: a bridged stream is only treated as required when there is at least one downstream subscriber beyond the wrapper's own internal monitor subscription.
- `watchdog_shutdown_on_fatal` can be left `false` while debugging so the node keeps running and logging instead of exiting on watchdog fatal conditions.
- When `apply_source_stream_control_on_startup` is enabled, the wrapper sends the configured `use_*` and `*_profile_*` values to `/amr_sweeper/depth_camera/set_parameters_atomically`, which the domain bridge forwards to the native D555 ROS node in camera domain `5`.
- The bridge rewrites serial-specific native camera topics under `/realsense/D555_<serial>...` into `/amr_sweeper/depth_camera/...`.
- The native D555 services are bridged from `/<camera_id>/...`, matching the live graph exposed by the DDS-backed RealSense wrapper.
- The bridge also exposes the D555's control surfaces under `/amr_sweeper/depth_camera/...`, including ROS parameter services for stream toggles and profiles plus native RealSense ROS services such as calibration, application config, safety config, hardware monitor, reset, and device info.
- The wrapper now bridges only the configured core stream topics plus their matching `camera_info` topics, along with `tf_static`. It does not bridge the native `metadata`, `metadata_legacy`, point-cloud, or object-detection topics.
- The default output topic resolves to `/amr_sweeper/depth_camera/scan`.
- The default `output_frame` is `laserscan_link`, a frame pitched upward to match the selected scan row.
- `scan_tilt_angle_deg` is resolution-independent and shifts the sampled scan band by angle instead of raw pixels.
- If topic auto-discovery finds more than one D555 on the camera domain, set `source_camera_id` explicitly.
- Launch-time camera discovery now queries camera domain `5` with `ros2 topic list --no-daemon`, so it can refresh the D555 graph directly without restarting the global ROS 2 daemon.
- The watchdog monitors the bridged depth, color, infrared, and motion topics by default.
- If some monitored topics are stale while at least one monitored topic is still healthy, the watchdog stays at `WARN`.
- The watchdog escalates to `ERROR` and `FATAL` only when all enabled monitored topics are missing or stale together.
