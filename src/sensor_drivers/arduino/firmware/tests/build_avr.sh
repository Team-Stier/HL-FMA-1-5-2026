#!/usr/bin/env bash
# Compile/link only. Never uploads. Uses installed AVR tools and Arduino core.
set -euo pipefail
readonly STIER_ROS_LIB="${1:?provide absolute generated ros_lib path}"
readonly STIER_ARDUINO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly STIER_AVR_BUILD="$(mktemp -d /tmp/stier-avr-build.XXXXXX)"
readonly STIER_AVR_CORE="${STIER_ARDUINO_CORE:-/usr/share/arduino/hardware/arduino/cores/arduino}"
readonly STIER_AVR_VARIANT="${STIER_ARDUINO_VARIANT:-/usr/share/arduino/hardware/arduino/variants/standard}"
readonly STIER_OUTPUT_MODE="${STIER_ACTUATOR_OUTPUTS:-0}"
readonly STIER_ROS_MODE="${STIER_ROS_ENABLED:-1}"
case "$STIER_OUTPUT_MODE" in 0|1) ;; *) exit 2;; esac
case "$STIER_ROS_MODE" in 0|1) ;; *) exit 2;; esac
echo "AVR artifacts: ${STIER_AVR_BUILD}; actuator=${STIER_OUTPUT_MODE}, ROS=${STIER_ROS_MODE} (compile only)"
stier_flags=(-mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10819 -DARDUINO_AVR_UNO
  -DARDUINO_ARCH_AVR -Os -flto -ffunction-sections -fdata-sections
  -I"$STIER_AVR_CORE" -I"$STIER_AVR_VARIANT")
mkdir -p "${STIER_AVR_BUILD}/core"
for stier_source in "$STIER_AVR_CORE"/*.cpp "$STIER_ROS_LIB"/*.cpp; do
  avr-g++ "${stier_flags[@]}" -I"$STIER_ROS_LIB" -std=gnu++11 -fno-exceptions -fno-threadsafe-statics \
    -c "$stier_source" -o "${STIER_AVR_BUILD}/core/$(basename "$stier_source").o"
done
for stier_source in "$STIER_AVR_CORE"/*.c; do
  avr-gcc "${stier_flags[@]}" -std=gnu11 -c "$stier_source" \
    -o "${STIER_AVR_BUILD}/core/$(basename "$stier_source").o"
done
for stier_source in "$STIER_AVR_CORE"/*.S; do
  [[ -f "$stier_source" ]] || continue
  avr-gcc "${stier_flags[@]}" -x assembler-with-cpp -c "$stier_source" \
    -o "${STIER_AVR_BUILD}/core/$(basename "$stier_source").o"
done
for stier_car in White Black; do
  stier_sketch_dir="${STIER_ARDUINO_ROOT}/BROON_T870_${stier_car}_Car"
  mkdir -p "${STIER_AVR_BUILD}/${stier_car}"
  for stier_source in "${stier_sketch_dir}/BROON_T870_${stier_car}_Car.ino" "${stier_sketch_dir}/BroonT870Core.cpp"; do
    avr-g++ "${stier_flags[@]}" -I"$STIER_ROS_LIB" -I"$stier_sketch_dir" \
      "-DBROON_ENABLE_ROS=${STIER_ROS_MODE}" "-DBROON_ENABLE_HUMAN_SERIAL=$((1-STIER_ROS_MODE))" \
      "-DBROON_ENABLE_ACTUATOR_OUTPUTS=${STIER_OUTPUT_MODE}" \
      -std=gnu++11 -fno-exceptions -fno-threadsafe-statics -x c++ -c "$stier_source" \
      -o "${STIER_AVR_BUILD}/${stier_car}/$(basename "$stier_source").o"
  done
  avr-gcc -mmcu=atmega328p -Os -flto -Wl,--gc-sections \
    "${STIER_AVR_BUILD}/${stier_car}"/*.o "${STIER_AVR_BUILD}/core"/*.o -lm \
    -o "${STIER_AVR_BUILD}/${stier_car}/firmware.elf"
  avr-size -C --mcu=atmega328p "${STIER_AVR_BUILD}/${stier_car}/firmware.elf"
  read -r stier_text stier_data stier_bss _ < <(avr-size "${STIER_AVR_BUILD}/${stier_car}/firmware.elf" | tail -n 1)
  (( stier_text + stier_data <= 32256 && stier_data + stier_bss < 2048 ))
  avr-objcopy -O ihex -R .eeprom "${STIER_AVR_BUILD}/${stier_car}/firmware.elf" \
    "${STIER_AVR_BUILD}/${stier_car}/firmware.hex"
done
