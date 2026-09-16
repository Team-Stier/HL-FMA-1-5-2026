cmake_minimum_required(VERSION 3.0.2)
project(sensor_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_launch_contract.py)
endif()
