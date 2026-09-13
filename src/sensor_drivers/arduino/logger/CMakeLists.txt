cmake_minimum_required(VERSION 3.0.2)
project(can_logger)
find_package(catkin REQUIRED)
catkin_package()
install(PROGRAMS scripts/can_csv_logger scripts/capture_csv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_capture.py)
endif()
