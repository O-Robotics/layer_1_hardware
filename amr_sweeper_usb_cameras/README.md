# amr_sweeper_usb_cameras

`amr_sweeper_usb_cameras` is a ROS 2 package for running up to five MJPEG USB cameras on the AMR sweeper platform.

The package is intentionally lean:
- it captures MJPEG from V4L2 with `mmap`
- it publishes the compressed image stream by default
- it decodes to raw RGB only when a raw subscriber is present
- it loads camera calibration from YAML files for wide-angle lenses

## Cameras

The package is set up for these camera names:
- `front_left_camera`
- `front_right_camera`
- `rear_left_camera`
- `rear_right_camera`
- `tools_camera`

Each camera has:
- a parameter file in `config/*_camera_params.yaml`
- a calibration file in `config/*_camera_info.yaml`

## Published Topics

Each camera publishes into its own namespace based on `camera_name`.

For `tools_camera`, the topics are:
- `tools_camera/image_raw`
- `tools_camera/image_raw/compressed`
- `tools_camera/tools_camera_info`

The same pattern applies to the other cameras.

### Publish behavior

- `<camera_name>/image_raw/compressed` is always published
- `<camera_name>/image_raw` is only decoded and published when something subscribes to it
- `<camera_name>/<camera_name>_info` is published alongside the image stream

This keeps CPU use lower when only the compressed stream is needed.

## Parameters

The node uses a small fixed parameter set:

- `camera_name`: logical camera name and topic namespace
- `camera_info_url`: `package://...` URL for the calibration YAML
- `frame_id`: frame ID written into published message headers
- `framerate`: requested camera frame rate
- `image_width`: requested image width
- `image_height`: requested image height
- `video_device`: V4L2 device path such as `/dev/video0`

Current default deployment uses:
- `320x240`
- `5 fps`
- MJPEG input

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_sweeper_usb_cameras --symlink-install
```

## Run

Run one camera instance with its parameter file:

```bash
ros2 run amr_sweeper_usb_cameras amr_sweeper_usb_cameras_node \
  --ros-args \
  --params-file ~/rob_ws/src/layer_1_hardware/amr_sweeper_usb_cameras/config/tools_camera_params.yaml
```

Examples for other cameras:

```bash
ros2 run amr_sweeper_usb_cameras amr_sweeper_usb_cameras_node \
  --ros-args \
  --params-file ~/rob_ws/src/layer_1_hardware/amr_sweeper_usb_cameras/config/front_left_camera_params.yaml
```

```bash
ros2 run amr_sweeper_usb_cameras amr_sweeper_usb_cameras_node \
  --ros-args \
  --params-file ~/rob_ws/src/layer_1_hardware/amr_sweeper_usb_cameras/config/rear_right_camera_params.yaml
```

Run multiple cameras by starting multiple node instances, each with a different parameter file.

## Config Files

### Parameter files

Each `*_params.yaml` file selects:
- the device path
- the camera name
- the frame ID
- the resolution and frame rate
- the matching calibration file

### Calibration files

Each `*_info.yaml` file stores the camera calibration used by `camera_info_manager`:
- image size
- camera matrix
- distortion coefficients
- rectification matrix
- projection matrix

These files are important for wide-angle lens calibration and downstream image processing.

## Notes

- The package expects the input stream to be MJPEG.
- The driver uses V4L2 `mmap` streaming only.
- Invalid camera device paths are checked before streaming starts.
- Camera timestamps are translated into ROS-friendly wall-clock timestamps before publication.

## Typical Workflow

1. Build the package.
2. Start one node instance per camera with the matching parameter file.
3. Consume the compressed topic by default.
4. Subscribe to the raw topic only when a decoded RGB image is needed.

