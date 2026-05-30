# amr_sweeper_depth_camera

```bash
ros2 launch amr_sweeper_depth_camera amr_sweeper_depth_camera.launch.py
```

## Purpose
This package is now a thin wrapper around `realsense2_camera` plus the local `laserscan_node`. It keeps the existing AMR Sweeper-facing topic contract under `/amr_sweeper/depth_camera/...` while relying on the vendored `realsense-ros` package for the RealSense driver itself.

## Main Outputs
- `/amr_sweeper/depth_camera/color/image_raw`
- `/amr_sweeper/depth_camera/color/camera_info`
- `/amr_sweeper/depth_camera/depth/image`
- `/amr_sweeper/depth_camera/depth/camera_info`
- `/amr_sweeper/depth_camera/depth/color/points`
- `/amr_sweeper/depth_camera/motion/imu`
- `/amr_sweeper/depth_camera/scan`

## Notes
- The wrapper launch includes `realsense2_camera/launch/rs_launch.py` and remaps the few native topics that differ from the AMR Sweeper contract, mainly `depth/image_rect_raw -> depth/image` and `imu -> motion/imu`.
- `config/amr_sweeper_depth_camera.yaml` now uses `realsense2_camera` parameter names.
- `camera_domain_id` is kept only as an informational compatibility argument because the D555 side is still expected to run with `ROS_DOMAIN_ID=5`.
- TODO: the older custom wrapper development is intentionally deferred for now.
