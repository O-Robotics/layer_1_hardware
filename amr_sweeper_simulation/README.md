# amr_sweeper_simulation

```bash
ros2 launch amr_sweeper_simulation amr_sweeper_simulation.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_description`

## Purpose
`amr_sweeper_simulation` owns the AMR Sweeper-specific Gazebo simulation assets and launch plumbing. It keeps workspace customizations out of vendored third-party simulation packages so the simulation stack remains stable across upstream resets.

## Main Launch Files
- `launch/amr_sweeper_gazebo.launch.py`
- `launch/amr_sweeper_simulation.launch.py`
- `launch/simulate_programmed_missions.launch.py`

## Package Scripts
- `scripts/simulate_programmed_missions.sh`

## Overview
This package provides the AMR Sweeper simulation world, a Gazebo-specific URDF wrapper built on `amr_sweeper_description`, the Gazebo-to-ROS topic bridge, and the simulated GNSS publisher. Layer 1 bringup uses the Gazebo launch when `use_simulation:=true`.

## How To Run

### 1. Launch only the simulation package
Use this when you want the Gazebo world, robot spawn, ROS-Gazebo bridges, and simulated GNSS without the rest of the AMR Sweeper stack.

```bash
ros2 launch amr_sweeper_simulation amr_sweeper_simulation.launch.py
```

This entrypoint includes `launch/amr_sweeper_gazebo.launch.py`.

### 2. Launch the simulation package directly with arguments
Use the Gazebo launch directly when you want to change the namespace or selectively disable simulated sensors.

```bash
ros2 launch amr_sweeper_simulation amr_sweeper_gazebo.launch.py
```

Available launch arguments:
- `namespace`: default `amr_sweeper`
- `enable_gnss`: default `true`
- `enable_imu`: default `true`
- `enable_depth_camera`: default `true`

### 3. Launch the full layer 1 stack in simulation
Use this when you want the simulation world plus the AMR Sweeper hardware-layer nodes.

```bash
ros2 launch amr_sweeper_layer_1_hardware_bringup   amr_sweeper_layer_1_hardware_bringup.launch.py   use_simulation:=true
```

### 4. Launch navigation on top of the simulated robot
If you also want localization, mapping, and Nav2, start layer 1 first, then layer 2 and layer 3 in separate terminals.

### 5. Simulate programmed missions through the full FSM cycle
Use this entrypoint when you want a schedule-driven simulation that runs the normal layer 0 stack and lets the FSM progress through `INITIALIZING`, `IDLING`, and `RUNNING`.

Direct bringup launch:

```bash
ros2 launch amr_sweeper_bringup amr_sweeper_bringup.launch.py   use_simulation:=true   use_profile:=050   schedule_ics_path:=missions/database/schedule_20260000T000000Z.ics
```

Wrapper script from the source tree:

```bash
src/layer_1_hardware/amr_sweeper_simulation/scripts/simulate_programmed_missions.sh   schedule_ics_path:=missions/database/schedule_20260000T000000Z.ics   simulation_speed:=10.0
```

Wrapper script after install:

```bash
$(ros2 pkg prefix amr_sweeper_simulation)/lib/amr_sweeper_simulation/simulate_programmed_missions.sh   schedule_ics_path:=missions/database/schedule_20260000T000000Z.ics   simulation_speed:=10.0
```

The wrapper script auto-sources ROS 2 and the workspace overlay before delegating to `amr_sweeper_bringup.launch.py` in simulation mode. It also applies the Gazebo speed override via `simulation_speed:=...`.

Main launch arguments:
- `schedule_ics_path`: explicit `.ics` schedule file. Leave empty to let the scheduler auto-discover the newest `schedule_*.ics` in `missions/database`.
- `simulation_speed`: wrapper-only Gazebo real-time factor. Default `10.0`.
- `simulation_world_name`: wrapper-only Gazebo world name. Default `amr_sweeper_test`.
- `missions_from_db_directory`: mission database root. Default `missions/database`.
- `missions_log_directory`: simulation log root. Default `missions/simulations`.
- `use_profile`: startup FSM profile. Default `050`.

Simulation outputs are created under the configured log root, for example:

```text
missions/simulations/Ecopark1/20260629T154500Z
```
