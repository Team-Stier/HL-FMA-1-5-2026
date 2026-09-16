cmake_minimum_required(VERSION 3.0.2)
project(selector)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  planning_interfaces
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nav_msgs planning_interfaces rospy
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/selector_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
