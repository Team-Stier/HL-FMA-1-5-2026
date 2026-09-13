cmake_minimum_required(VERSION 3.0.2)
project(cam_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/kiyo_setup.sh
  scripts/logi_setup.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
