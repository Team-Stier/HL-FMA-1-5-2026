#!/usr/bin/env bash
# Isolated middleware verification. Never starts a hardware driver or command publisher.
set -euo pipefail

readonly STIER_REPOSITORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
  echo 'ROS Noetic is required; use the branch GitHub Actions job or a Noetic host.' >&2
  exit 1
fi
readonly STIER_TEST_WORKSPACE="$(mktemp -d "${TMPDIR:-/tmp}/stier-noetic.XXXXXX")"
trap 'rm -rf -- "${STIER_TEST_WORKSPACE}"' EXIT
mkdir -p "${STIER_TEST_WORKSPACE}/src/localization/scripts"
for stier_package in state_manager selector control; do
  cp -R "${STIER_REPOSITORY}/src/${stier_package}" "${STIER_TEST_WORKSPACE}/src/${stier_package}"
done
cp -R "${STIER_REPOSITORY}/src/interfaces/planning_interfaces" "${STIER_TEST_WORKSPACE}/src/planning_interfaces"
cp -R "${STIER_REPOSITORY}/src/interfaces/vehicle_interface/erp42_msgs" "${STIER_TEST_WORKSPACE}/src/erp42_msgs"
# Preserve the real RDDF regression fixtures without building vendor drivers.
cp -R "${STIER_REPOSITORY}/src/localization/rddf" "${STIER_TEST_WORKSPACE}/src/localization/rddf"
cp "${STIER_REPOSITORY}/src/localization/scripts/rddf_route_provider.py" "${STIER_TEST_WORKSPACE}/src/localization/scripts/"
# A message-only fixture builds the real localization wire types for inspection.
# This is not a build of the localization C++ stack or its hardware drivers.
mkdir -p "${STIER_TEST_WORKSPACE}/src/localization/msg"
cp "${STIER_REPOSITORY}/src/localization/msg/RddfCandidate.msg" "${STIER_TEST_WORKSPACE}/src/localization/msg/"
cp "${STIER_REPOSITORY}/src/localization/msg/RddfMatch.msg" "${STIER_TEST_WORKSPACE}/src/localization/msg/"
cat > "${STIER_TEST_WORKSPACE}/src/localization/package.xml" <<'XML'
<package format="2">
  <name>mando_localization</name><version>0.0.0</version>
  <description>Message-only fixture for inspection tests</description>
  <maintainer email="team-stier@example.com">Stier</maintainer><license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend><build_depend>message_generation</build_depend>
  <depend>std_msgs</depend><depend>geometry_msgs</depend><exec_depend>message_runtime</exec_depend>
</package>
XML
cat > "${STIER_TEST_WORKSPACE}/src/localization/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.0.2)
project(mando_localization)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)
add_message_files(FILES RddfCandidate.msg RddfMatch.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
CMAKE

set +u
source /opt/ros/noetic/setup.bash
set -u
cd "${STIER_TEST_WORKSPACE}"
catkin_make -DCATKIN_ENABLE_TESTING=ON
set +u
source devel/setup.bash
set -u
export ROS_TEST_RESULTS_DIR="${STIER_TEST_WORKSPACE}/build/test_results"
python3 -m unittest discover -s src/state_manager/test -v
python3 -m unittest discover -s src/selector/test -v
rostest state_manager mission_pipeline.test
catkin_test_results build/test_results
