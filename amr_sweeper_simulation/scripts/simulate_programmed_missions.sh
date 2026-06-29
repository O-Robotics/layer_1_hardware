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

ros2 launch amr_sweeper_simulation simulate_programmed_missions.launch.py "$@"
