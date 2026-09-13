cmake_minimum_required(VERSION 3.0.2)
project(localization)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  erp42_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  rosgraph_msgs
  std_msgs
  ublox_msgs
  rviz
  sensor_interfaces
  sensor_msgs
  tf2_ros
  visualization_msgs
)
catkin_python_setup()

find_package(PkgConfig REQUIRED)
pkg_check_modules(QT5_WIDGETS REQUIRED Qt5Widgets)
find_program(QT_MOC_EXECUTABLE moc REQUIRED)

catkin_package(
  LIBRARIES localization_rviz_plugins
  CATKIN_DEPENDS
    diagnostic_msgs
    erp42_msgs
    geometry_msgs
    nav_msgs
    pluginlib
    roscpp
    rospy
    rosgraph_msgs
    std_msgs
    ublox_msgs
    rviz
    sensor_interfaces
    sensor_msgs
    tf2_ros
    visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/localization_node
  scripts/bag_route_compare_node
  scripts/localization_sensor_status_node
  scripts/wheel_imu_gps_odometry_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS launch.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  src/rviz_plugins
  ${catkin_INCLUDE_DIRS}
  ${QT5_WIDGETS_INCLUDE_DIRS}
)
link_directories(${QT5_WIDGETS_LIBRARY_DIRS})

set(RVIZ_PANEL_HEADER
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rviz_plugins/localization_sensor_status_panel.hpp
)
set(RVIZ_PANEL_MOC
  ${CMAKE_CURRENT_BINARY_DIR}/moc_localization_sensor_status_panel.cpp
)
set(MOC_INCLUDE_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/src/rviz_plugins")
foreach(MOC_INCLUDE_DIR ${QT5_WIDGETS_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  list(APPEND MOC_INCLUDE_FLAGS "-I${MOC_INCLUDE_DIR}")
endforeach()
add_custom_command(
  OUTPUT ${RVIZ_PANEL_MOC}
  COMMAND ${QT_MOC_EXECUTABLE} ${MOC_INCLUDE_FLAGS} ${RVIZ_PANEL_HEADER}
          -o ${RVIZ_PANEL_MOC}
  DEPENDS ${RVIZ_PANEL_HEADER}
  COMMENT "Generating Qt meta-object code for LocalizationSensorStatusPanel"
  VERBATIM
)
add_library(localization_rviz_plugins
  src/rviz_plugins/localization_sensor_status_panel.cpp
  ${RVIZ_PANEL_MOC}
)
target_link_libraries(localization_rviz_plugins
  ${catkin_LIBRARIES}
  ${QT5_WIDGETS_LIBRARIES}
)
target_compile_options(localization_rviz_plugins PRIVATE ${QT5_WIDGETS_CFLAGS_OTHER})

install(TARGETS localization_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/localization_pipeline.test)
  add_rostest(test/localization_debug_pipeline.test)
  catkin_add_nosetests(test/test_bag_route_compare_core.py)
  catkin_add_nosetests(test/test_localization_core.py)
  catkin_add_nosetests(test/test_localization_configuration.py)
endif()
