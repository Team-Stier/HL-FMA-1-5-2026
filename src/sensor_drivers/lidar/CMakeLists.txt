cmake_minimum_required(VERSION 3.0.2)
project(lidar_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  launch
  scripts
  udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
