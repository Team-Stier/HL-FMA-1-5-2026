cmake_minimum_required(VERSION 3.0.2)
project(selector)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS nav_msgs rospy
)

catkin_install_python(PROGRAMS
  scripts/selector_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
