cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs tf tf2_ros)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs tf tf2_ros)

catkin_install_python(PROGRAMS
  scripts/dbscan_visualizer_node
  scripts/lidar_preprocessor_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_dbscan_core.py)
  catkin_add_nosetests(test/test_lidar_preprocessor.py)
endif()
