cmake_minimum_required(VERSION 3.0.2)
project(gps_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_interfaces
  ublox_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_interfaces ublox_msgs
)

catkin_install_python(PROGRAMS
  scripts/gps_status_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    test/test_gps_status_node.py
    DEPENDENCIES ${catkin_EXPORTED_TARGETS}
  )
endif()
