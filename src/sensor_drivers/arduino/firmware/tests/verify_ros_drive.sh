#!/usr/bin/env bash
# No Arduino connection. First argument: ros_lib generated from integration ERP42 messages.
set -euo pipefail
readonly STIER_ROS_LIB="${1:?provide absolute generated ros_lib path}"
readonly STIER_ARDUINO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly STIER_FIRMWARE_TEST_DIR="$(mktemp -d /tmp/stier-firmware-test.XXXXXX)"
echo "Host test artifacts: ${STIER_FIRMWARE_TEST_DIR}"
for stier_car in White Black; do
  stier_sketch_dir="${STIER_ARDUINO_ROOT}/BROON_T870_${stier_car}_Car"
  g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-function \
    -DBROON_ENABLE_ROS=1 -DBROON_ENABLE_HUMAN_SERIAL=0 \
    -DBROON_ENABLE_ACTUATOR_OUTPUTS=1 \
    "-DSTIER_SKETCH=\"${stier_sketch_dir}/BROON_T870_${stier_car}_Car.ino\"" \
    -I"${STIER_ARDUINO_ROOT}/tests/stubs" -I"${STIER_ROS_LIB}" -I"${stier_sketch_dir}" \
    "${STIER_ARDUINO_ROOT}/tests/ros_drive_integration.cpp" \
    "${stier_sketch_dir}/BroonT870Core.cpp" \
    -o "${STIER_FIRMWARE_TEST_DIR}/${stier_car}"
  "${STIER_FIRMWARE_TEST_DIR}/${stier_car}"
done
