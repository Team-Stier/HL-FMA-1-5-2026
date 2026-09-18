#!/usr/bin/env bash
# No ROS generation, no devices, no firmware uploads.
set -euo pipefail
readonly STIER_FIRMWARE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly STIER_SPEED_TEST_DIR="$(mktemp -d /tmp/stier-speed-core.XXXXXX)"
echo "Speed core test artifacts: ${STIER_SPEED_TEST_DIR}"
for stier_car in White Black; do
  stier_core="${STIER_FIRMWARE_ROOT}/BROON_T870_${stier_car}_Car"
  g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=undefined \
    -fno-sanitize-recover=all -I"${STIER_FIRMWARE_ROOT}/tests/stubs" -I"${stier_core}" \
    "${STIER_FIRMWARE_ROOT}/tests/speed_profile_core.cpp" \
    "${stier_core}/BroonT870Core.cpp" -o "${STIER_SPEED_TEST_DIR}/${stier_car}"
  "${STIER_SPEED_TEST_DIR}/${stier_car}"
done
