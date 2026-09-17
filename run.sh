#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  bash "${BASH_SOURCE[0]}" "$@"
  return $?
fi

set -Eeo pipefail

readonly STIER_WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[bringup] ROS Noetic is not installed at /opt/ros/noetic" >&2
  exit 1
fi
if [[ ! -f "${STIER_WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  echo "[bringup] workspace is not built; run catkin_make first" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source "${STIER_WORKSPACE_ROOT}/devel/setup.bash"

exec roslaunch stier_bringup full_vehicle.launch "$@"
