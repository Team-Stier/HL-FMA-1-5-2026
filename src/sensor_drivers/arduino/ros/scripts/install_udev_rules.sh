#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_PATH="${SCRIPT_DIR}/../udev/99-stier-arduino.rules"

sudo install -m 0644 "${RULES_PATH}" /etc/udev/rules.d/99-stier-arduino.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --property-match=ID_VENDOR_ID=1a86 --property-match=ID_MODEL_ID=7523
sudo udevadm settle --timeout=30

echo "Installed Arduino udev rule. Use the registered USB port and /dev/arduino."
