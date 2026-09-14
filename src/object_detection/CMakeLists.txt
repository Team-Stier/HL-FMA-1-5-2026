cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mando_localization
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  visualization_msgs
)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  geometry_msgs mando_localization rospy sensor_msgs std_msgs tf tf2_ros visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/object_detection_viewer.py
  scripts/rddf_roi_detector_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_dbscan_core.py)
  catkin_add_nosetests(test/test_rddf_roi_core.py)
endif()
