# amr_sweeper_usb_cameras

```bash
ros2 launch amr_sweeper_usb_cameras amr_sweeper_usb_cameras.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package runs the USB camera nodes used by the AMR Sweeper camera set.

## Main Launch File
`launch/amr_sweeper_usb_cameras.launch.py`

## Available Launch Files
- `amr_sweeper_usb_cameras.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper/usb_cameras`
- `log_level`: default `info`
- `front_left_camera_enabled`: default `true`
- `front_right_camera_enabled`: default `true`
- `rear_left_camera_enabled`: default `true`
- `rear_right_camera_enabled`: default `true`
- `tools_camera_enabled`: default `true`

## Overview
`amr_sweeper_usb_cameras` contains the camera driver node and the launch configuration needed to start the onboard MJPEG USB cameras. It is part of the layer 1 sensor stack and is usually brought up through the hardware layer bringup package. By default it launches the camera nodes under `/amr_sweeper/usb_cameras/<camera_name>`.

## Notes
- Main node: `amr_sweeper_usb_cameras_node`.
- Used to expose the camera topics required by higher-level perception or operator tools.
- Each enabled camera node gets its own namespace below the configured camera root, for example `/amr_sweeper/usb_cameras/front_left_camera`.
- The robot description publishes one physical frame per USB camera as `<camera_name>_link` and the corresponding ROS optical frame as `<camera_name>_optical_frame`.
- The default USB camera parameter files stamp images with the physical camera frame ids `<camera_name>_link`.
