#!/usr/bin/env bash
# 2026-09-18 GPS straight-run placement; source course remains unchanged.
set -Eeo pipefail
readonly STIER_S_COURSE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly STIER_S_COURSE_MAP="${STIER_S_COURSE_ROOT}/src/localization/test_data/hongik_s_live_20260918"
exec bash "${STIER_S_COURSE_ROOT}/run.sh" \
  hongik_test:=false hongik_s_test:=true \
  rddf_directory:="${STIER_S_COURSE_MAP}" \
  rddf_initialization_config:="${STIER_S_COURSE_MAP}/initialization.yaml" \
  localization_viewer_config:="${STIER_S_COURSE_MAP}/viewer.yaml" \
  arduino_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  enable_camera:=false start_traffic_light:=false target_speed_kph:=5 "$@"
