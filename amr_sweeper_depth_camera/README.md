# amr_sweeper_depth_camera

```bash
ros2 launch amr_sweeper_depth_camera amr_sweeper_depth_camera.launch.py
```

## Purpose
This package is now a thin wrapper around `realsense2_camera` plus the local `laserscan_node`. The RealSense driver publishes directly under `/amr_sweeper/depth_camera/...` while relying on the bundled `realsense-ros` subtree at `src/realsense-ros/` for the driver itself.

## Main Outputs
- `/amr_sweeper/depth_camera/color/image_raw`
- `/amr_sweeper/depth_camera/color/camera_info`
- `/amr_sweeper/depth_camera/depth/image_rect_raw`
- `/amr_sweeper/depth_camera/depth/camera_info`
- `/amr_sweeper/depth_camera/depth/color/points`
- `/amr_sweeper/depth_camera/motion/sample`
- `/amr_sweeper/depth_camera/scan`

## Notes
- The `realsense2_camera_node` now runs directly in the configured camera namespace. The default namespace still resolves to `/amr_sweeper/depth_camera`.
- There is no topic bridge in the launch flow anymore, so consumers should use the native RealSense topic names such as `depth/image_rect_raw` and `motion/sample`.
- The derived `/amr_sweeper/depth_camera/scan` topic stays on sensor-data QoS with best-effort reliability so navigation and mapping consume the live scan stream with the same contract as the publisher.
- `config/amr_sweeper_depth_camera.yaml` now uses `realsense2_camera` parameter names.
- `camera_domain_id` is kept only as an informational compatibility argument because the D555 side is still expected to run with `ROS_DOMAIN_ID=5`.
- The default config now prefers the D555 DDS motion stream over the separate accel/gyro path and disables aligned-depth publishing to reduce runtime noise.
- The bundled RealSense ROS packages live under `src/realsense-ros/` inside this package directory and should be built in the same workspace as `amr_sweeper_depth_camera`.
- TODO: the older custom wrapper development is intentionally deferred for now.

## Build
Build the wrapper together with the bundled RealSense packages:

```bash
colcon build --packages-select realsense2_camera_msgs realsense2_camera amr_sweeper_depth_camera
```

If `realsense2_camera` gets killed during compilation on a low-memory machine, rebuild it single-threaded:

```bash
colcon build --packages-select realsense2_camera amr_sweeper_depth_camera --executor sequential --parallel-workers 1
```
