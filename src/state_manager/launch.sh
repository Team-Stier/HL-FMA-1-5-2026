#!/usr/bin/env bash
set -eo pipefail
exec roslaunch state_manager mission.launch "$@"
