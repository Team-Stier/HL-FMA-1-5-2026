#!/usr/bin/env bash
# School full-course preset. Starts hardware/control only when invoked.
# Does not alter an already running S-course session or rewrite shared config.
set -Eeo pipefail

readonly STIER_FULL_COURSE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Arguments after these defaults can override speed, parking side and devices.
# The camera-free preset exercises the existing UNKNOWN-signal timeout policy;
# it is not a traffic-light recognition test.
exec bash "${STIER_FULL_COURSE_ROOT}/run.sh" \
  hongik_test:=true hongik_s_test:=false \
  arduino_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  enable_camera:=false start_traffic_light:=false \
  target_speed_kph:=5 \
  "$@"
