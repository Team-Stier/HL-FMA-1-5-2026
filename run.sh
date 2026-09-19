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

# 현 위치 GPS에 정렬한 용인 T자 주차 테스트 RDDF와 미션을 기본으로 실행한다. 뒤에 전달한 인자로 기본값을 변경할 수 있다.
exec roslaunch stier_bringup full_vehicle.launch \
  mission_config:="${STIER_WORKSPACE_ROOT}/src/localization/test_data/everland_rddf/missions.json" \
  rddf_directory:="${STIER_WORKSPACE_ROOT}/src/localization/test_data/everland_rddf" \
  rddf_initialization_config:="${STIER_WORKSPACE_ROOT}/src/localization/test_data/everland_rddf/initialization.yaml" \
  localization_viewer_config:="${STIER_WORKSPACE_ROOT}/src/localization/test_data/everland_rddf/viewer.yaml" \
  hongik_test:=false \
  hongik_s_test:=false \
  hongik_parallel_test:=false \
  start_rviz:=true \
  "$@"
