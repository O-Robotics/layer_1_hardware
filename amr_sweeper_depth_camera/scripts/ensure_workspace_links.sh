#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${script_dir}/../../../.." && pwd)"
workspace_src="${workspace_root}/src"
vendor_root="${workspace_src}/layer_1_hardware/amr_sweeper_depth_camera/src/realsense-ros"

ensure_link() {
  local link_name="$1"
  local target_path="${vendor_root}/${link_name}"
  local link_path="${workspace_src}/${link_name}"

  if [[ ! -d "${target_path}" ]]; then
    echo "Skipping ${link_name}: target package not found at ${target_path}" >&2
    return
  fi

  if [[ -L "${link_path}" ]]; then
    local current_target
    current_target="$(readlink "${link_path}")"
    if [[ "${current_target}" == "layer_1_hardware/amr_sweeper_depth_camera/src/realsense-ros/${link_name}" ]]; then
      return
    fi
    rm -f "${link_path}"
  elif [[ -e "${link_path}" ]]; then
    echo "Refusing to replace existing non-symlink path: ${link_path}" >&2
    return 1
  fi

  ln -s "layer_1_hardware/amr_sweeper_depth_camera/src/realsense-ros/${link_name}" "${link_path}"
  echo "Linked ${link_name} into workspace src/"
}

ensure_link realsense2_camera
ensure_link realsense2_camera_msgs
ensure_link realsense2_description
