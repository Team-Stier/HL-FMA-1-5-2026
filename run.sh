#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  bash "${BASH_SOURCE[0]}" "$@"
  return $?
fi

set -Eeo pipefail

readonly STIER_WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
declare -a STIER_NODE_PIDS=()
STIER_MASTER_PID=""

cleanup_bringup() {
  local bringup_status=$?

  trap - EXIT INT TERM

  if ((${#STIER_NODE_PIDS[@]})); then
    kill "${STIER_NODE_PIDS[@]}" 2>/dev/null || true
    wait "${STIER_NODE_PIDS[@]}" 2>/dev/null || true
  fi

  if [[ -n "${STIER_MASTER_PID}" ]]; then
    kill "${STIER_MASTER_PID}" 2>/dev/null || true
    wait "${STIER_MASTER_PID}" 2>/dev/null || true
  fi

  exit "${bringup_status}"
}

start_ros_node() {
  local package_name=$1
  local node_name=$2

  if ! rosrun --prefix /usr/bin/true "${package_name}" "${node_name}" >/dev/null 2>&1; then
    echo "[bringup] ${package_name}/${node_name} not started: executable not found"
    return 0
  fi

  echo "[bringup] starting ${package_name}/${node_name}"
  rosrun "${package_name}" "${node_name}" &
  STIER_NODE_PIDS+=("$!")
}

start_package_launcher() {
  local package_name=$1
  local launcher_path=$2

  if [[ ! -x "${launcher_path}" ]]; then
    echo "[bringup] ${package_name} not started: launcher not executable"
    return 0
  fi

  echo "[bringup] starting ${package_name} launcher"
  "${launcher_path}" &
  STIER_NODE_PIDS+=("$!")
}

trap cleanup_bringup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[bringup] ROS Noetic is not installed at /opt/ros/noetic" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
cd "${STIER_WORKSPACE_ROOT}"
catkin_make
source "${STIER_WORKSPACE_ROOT}/devel/setup.bash"

if ! rosnode list >/dev/null 2>&1; then
  echo "[bringup] starting roscore"
  roscore &
  STIER_MASTER_PID=$!

  for _ in {1..50}; do
    if rosnode list >/dev/null 2>&1; then
      break
    fi
    sleep 0.1
  done

  if ! rosnode list >/dev/null 2>&1; then
    echo "[bringup] ROS master did not become ready" >&2
    exit 1
  fi
fi

start_package_launcher localization "${STIER_WORKSPACE_ROOT}/src/localization/launch.sh"
start_package_launcher state_manager "${STIER_WORKSPACE_ROOT}/src/state_manager/launch.sh"

if ((${#STIER_NODE_PIDS[@]} == 0)); then
  echo "[bringup] no runnable nodes found"
  exit 0
fi

wait "${STIER_NODE_PIDS[@]}" || true
