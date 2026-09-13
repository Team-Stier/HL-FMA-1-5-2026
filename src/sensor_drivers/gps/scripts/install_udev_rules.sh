#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_PATH="${SCRIPT_DIR}/../udev/99-stier-gps.rules"

sudo install -m 0644 "${RULES_PATH}" /etc/udev/rules.d/99-stier-gps.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed GPS udev rules. Reconnect the receiver and use /dev/gps."
