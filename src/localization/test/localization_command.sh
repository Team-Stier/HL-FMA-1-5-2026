#!/usr/bin/env bash
# 실제 GPS, IMU, Encoder 입력과 Localization RViz를 한 번에 실행한다.
# 연결되지 않은 센서 드라이버는 시작하지 않고 RViz에 입력 없음으로 남긴다.

set -Eeo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly PACKAGE_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")/.." && pwd)"
readonly WORKSPACE_ROOT="$(cd -- "${PACKAGE_DIR}/../.." && pwd)"

declare -a CHILD_PIDS=()
MASTER_PID=""

cleanup() {
  local exit_status=$?
  trap - EXIT INT TERM

  if ((${#CHILD_PIDS[@]})); then
    kill -INT "${CHILD_PIDS[@]}" 2>/dev/null || true
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
  fi
  if [[ -n "${MASTER_PID}" ]]; then
    kill -INT "${MASTER_PID}" 2>/dev/null || true
    wait "${MASTER_PID}" 2>/dev/null || true
  fi
  exit "${exit_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[localization] /opt/ros/noetic/setup.bash가 없습니다." >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  echo "[localization] 먼저 ~/HL-FMA2026-suhyeon에서 catkin_make를 실행하세요." >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

if rosnode list >/dev/null 2>&1 &&
   rosnode list 2>/dev/null | grep -qx '/localization_live_sensor_rviz'; then
  echo "[localization] 실센서 RViz가 이미 실행 중입니다."
  exit 0
fi

if ! rosnode list >/dev/null 2>&1; then
  echo "[localization] ROS master 시작"
  roscore &
  MASTER_PID=$!
  for _ in {1..50}; do
    if rosnode list >/dev/null 2>&1; then
      break
    fi
    sleep 0.1
  done
  if ! rosnode list >/dev/null 2>&1; then
    echo "[localization] ROS master 시작 실패" >&2
    exit 1
  fi
fi

if rosnode list 2>/dev/null | grep -qx '/gps_bag_play'; then
  echo "[localization] GPS bag 재생이 실행 중입니다. 먼저 bag을 종료하세요." >&2
  exit 1
fi

start_driver() {
  local label=$1
  shift
  echo "[localization] ${label} 드라이버 시작"
  "$@" &
  CHILD_PIDS+=("$!")
}

device_is_usable() {
  [[ -e "$1" && -r "$1" && -w "$1" ]]
}

GPS_TARGET=""
IMU_TARGET=""
if [[ -e /dev/gps ]]; then
  GPS_TARGET="$(readlink -f /dev/gps)"
fi
if [[ -e /dev/imu ]]; then
  IMU_TARGET="$(readlink -f /dev/imu)"
fi

if device_is_usable /dev/gps; then
  start_driver "GPS" roslaunch gps_bringup gps.launch
else
  echo "[localization] GPS: 입력 없음 (/dev/gps)"
fi

if device_is_usable /dev/imu; then
  start_driver "IMU" roslaunch imu_bringup xsens_mti.launch
else
  echo "[localization] IMU: 입력 없음 (/dev/imu)"
fi

ENCODER_PORT="${STIER_ENCODER_PORT:-}"
if [[ -z "${ENCODER_PORT}" && -e /dev/arduino ]]; then
  ENCODER_PORT=/dev/arduino
fi

# udev 별칭이 아직 없어도 USB 포트 번호가 아닌 Arduino 고유 by-id를 우선한다.
# GPS/IMU가 여러 시리얼 포트를 사용해도 Arduino가 한 대면 자동 선택된다.
if [[ -z "${ENCODER_PORT}" ]]; then
  shopt -s nullglob
  declare -a arduino_by_id_candidates=(/dev/serial/by-id/*Arduino*)
  shopt -u nullglob
  if ((${#arduino_by_id_candidates[@]} == 1)); then
    ENCODER_PORT="${arduino_by_id_candidates[0]}"
  elif ((${#arduino_by_id_candidates[@]} > 1)); then
    echo "[localization] Arduino가 여러 대입니다. STIER_ENCODER_PORT로 지정하세요."
  fi
fi

if [[ -z "${ENCODER_PORT}" ]]; then
  shopt -s nullglob
  declare -a encoder_candidates=()
  for candidate in /dev/ttyACM* /dev/ttyUSB*; do
    candidate_target="$(readlink -f -- "${candidate}")"
    if [[ "${candidate_target}" == "${GPS_TARGET}" ||
          "${candidate_target}" == "${IMU_TARGET}" ]]; then
      continue
    fi
    encoder_candidates+=("${candidate}")
  done
  shopt -u nullglob
  if ((${#encoder_candidates[@]} == 1)); then
    ENCODER_PORT="${encoder_candidates[0]}"
  elif ((${#encoder_candidates[@]} > 1)); then
    echo "[localization] Encoder 후보가 여러 개입니다. 다음처럼 지정하세요:"
    echo "  STIER_ENCODER_PORT=/dev/serial/by-id/<Arduino-ID> localization"
  fi
fi

if [[ -n "${ENCODER_PORT}" ]] && device_is_usable "${ENCODER_PORT}"; then
  start_driver "Encoder (${ENCODER_PORT})" \
    roslaunch vehicle_interface_bringup arduino.launch \
    "port:=${ENCODER_PORT}" baud:=57600
else
  echo "[localization] Encoder: 입력 없음"
fi

echo "[localization] 실데이터 Localization + RViz 시작"
roslaunch localization localization_live_sensor_debug.launch &
MONITOR_PID=$!
CHILD_PIDS+=("${MONITOR_PID}")

echo "[localization] 종료하려면 Ctrl+C"
wait "${MONITOR_PID}"
