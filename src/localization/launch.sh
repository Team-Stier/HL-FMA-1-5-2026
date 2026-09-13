#!/usr/bin/env bash

set -eo pipefail

exec roslaunch localization localization.launch start_rviz:=false "$@"
