# ROS2 Package for ODrive

This repository contains ROS2 packages for the [ODrive motor controller](https://odriverobotics.com):

- **`odrive_node`**: Standalone ROS2 node for communication with ODrives via CAN bus. → [More info](odrive_node/README.md)
This package consolidates:
- `odrive_can_node` (CAN-based ODrive node)
- `odrive_ros2_control_plugin` (ros2_control integration for ODrive CAN)

`odrive_can_node` and the ros2_control plugin are two alternative approaches and cannot be used at the same time.

For information about installation, prerequisites and getting started, check out the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).

## Compatible Devices

- [ODrive Pro](https://shop.odriverobotics.com/products/odrive-pro)
- [ODrive S1](https://shop.odriverobotics.com/products/odrive-s1)
- [ODrive Micro](https://shop.odriverobotics.com/products/odrive-micro)

(not compatible with ODrive 3.x)

## System Requirements

- Ubuntu >= 20.04
- ROS2 >= Humble

## Developer Notes

(For user instructions, see [this guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html) instead.)

You can build this node on a non-ROS developer PC by using the DevContainer configurations in this repository. For example with VSCode:

1. Clone repository
2. Open the repository folder in VSCode. It should automatically present an option "Reopen in Dev Container". Click on that and select the desired ROS version.
3. Once it's re-opened, in the VSCode terminal, run:

   ```bash
   colcon build --packages-select amr_sweeper_odrive
   source ./install/setup.bash
   ```

4. Running the node requires hardware access and only works if the container host is Linux.
