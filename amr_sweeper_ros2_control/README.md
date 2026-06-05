# amr_sweeper_ros2_control

```bash
ros2 launch amr_sweeper_ros2_control amr_sweeper_ros2_control.launch.py
```

## Purpose
This package provides the shared layer 1 `ros2_control` runtime for the AMR Sweeper. It starts `ros2_control_node` plus the `joint_broad`, `drive_controller`, and `tool_controller` spawners while continuing to use the layer 1 ODrive and SteadyDrive hardware-interface plugins through the shared robot description.

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_ros2_control`: default `true`
- `ros2_control_config_file`: default `amr_sweeper_description/urdf/control/ros2_control.yaml`

## Notes
- The hardware-interface implementations remain in layer 1 packages.
- This package now lives in `src/layer_1_hardware/` and layer 1 hardware bringup is the normal owner of this runtime.
- This launch only owns `ros2_control_node` and the `joint_broad` spawner.
