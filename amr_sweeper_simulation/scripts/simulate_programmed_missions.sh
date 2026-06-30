#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${WORKSPACE_ROOT:-}" ]]; then
  workspace_root="${WORKSPACE_ROOT}"
elif [[ "${SCRIPT_DIR}" == *"/install/"* ]]; then
  workspace_root="${SCRIPT_DIR%%/install/*}"
else
  workspace_root="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
fi

cd "${workspace_root}"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

simulation_speed="10.0"
simulation_world_name="amr_sweeper_test"
launch_args=()
use_profile_seen=false
use_simulation_seen=false

for arg in "$@"; do
  case "$arg" in
    simulation_speed:=*)
      simulation_speed="${arg#simulation_speed:=}"
      ;;
    simulation_world_name:=*)
      simulation_world_name="${arg#simulation_world_name:=}"
      ;;
    use_profile:=*)
      use_profile_seen=true
      launch_args+=("$arg")
      ;;
    use_simulation:=*)
      use_simulation_seen=true
      launch_args+=("$arg")
      ;;
    *)
      launch_args+=("$arg")
      ;;
  esac
done

if [[ "$use_simulation_seen" == false ]]; then
  launch_args=("use_simulation:=true" "${launch_args[@]}")
fi

if [[ "$use_profile_seen" == false ]]; then
  launch_args=("use_profile:=050" "${launch_args[@]}")
fi

(
  sleep 8
  ros2 run amr_sweeper_simulation set_gazebo_simulation_speed.py     --speed "$simulation_speed"     --world "$simulation_world_name"
) &
speed_pid=$!
trap 'kill "$speed_pid" 2>/dev/null || true' EXIT

ros2 launch amr_sweeper_bringup amr_sweeper_bringup.launch.py "${launch_args[@]}"
