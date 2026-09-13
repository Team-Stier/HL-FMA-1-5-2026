cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs)

catkin_install_python(PROGRAMS scripts/lidar_preprocessor_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_lidar_preprocessor.py)
endif()
