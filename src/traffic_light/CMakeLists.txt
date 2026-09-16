cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  planning_interfaces
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  cv_bridge planning_interfaces rospy sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/traffic_light_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_detector_core.py)
endif()
