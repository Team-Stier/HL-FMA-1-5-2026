#!/usr/bin/env bash

DEVICE="${1:-/dev/cam}"

# Wait until usb_cam has opened the device.
sleep 3

echo "Starting Logitech C922 hardware configuration on ${DEVICE}..."

v4l2-ctl -d "${DEVICE}" --set-ctrl=power_line_frequency=2
v4l2-ctl -d "${DEVICE}" --set-ctrl=white_balance_automatic=1
v4l2-ctl -d "${DEVICE}" --set-ctrl=exposure_dynamic_framerate=0
v4l2-ctl -d "${DEVICE}" --set-ctrl=sharpness=128
v4l2-ctl -d "${DEVICE}" --set-ctrl=focus_automatic_continuous=1
v4l2-ctl -d "${DEVICE}" --set-ctrl=backlight_compensation=1

echo "Logitech C922 hardware settings applied successfully."
