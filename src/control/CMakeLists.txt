cmake_minimum_required(VERSION 3.0.2)
project(control)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  erp42_msgs
  geometry_msgs
  nav_msgs
  planning_interfaces
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    control_lateral
    control_longitudinal
    control_vehicle
  CATKIN_DEPENDS
    erp42_msgs
    geometry_msgs
    nav_msgs
    planning_interfaces
    roscpp
    std_msgs
    tf2
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(control_lateral
  src/lateral/pure_pursuit.cpp
  src/lateral/stanley_controller.cpp
)
target_compile_features(control_lateral PUBLIC cxx_std_14)

add_library(control_longitudinal
  src/longitudinal/fixed_speed_controller.cpp
  src/longitudinal/path_speed_profile.cpp
)
target_compile_features(control_longitudinal PUBLIC cxx_std_14)

add_library(control_vehicle
  src/vehicle/t870_command_mapper.cpp
)
target_compile_features(control_vehicle PUBLIC cxx_std_14)

add_executable(control_node src/node/control_node.cpp)
target_compile_features(control_node PUBLIC cxx_std_14)
target_link_libraries(control_node
  control_lateral
  control_longitudinal
  control_vehicle
  ${catkin_LIBRARIES}
)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/control_contract_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/control_contract.test DEPENDENCIES control_node)
  catkin_add_gtest(test_pure_pursuit
    test/unit/test_main.cpp
    test/unit/test_pure_pursuit.cpp
  )
  if(TARGET test_pure_pursuit)
    target_link_libraries(test_pure_pursuit control_lateral)
  endif()

  catkin_add_gtest(test_stanley_controller
    test/unit/test_main.cpp
    test/unit/test_stanley_controller.cpp
  )
  if(TARGET test_stanley_controller)
    target_link_libraries(test_stanley_controller control_lateral)
  endif()

  catkin_add_gtest(test_fixed_speed_controller
    test/unit/test_main.cpp
    test/unit/test_fixed_speed_controller.cpp
  )
  if(TARGET test_fixed_speed_controller)
    target_link_libraries(test_fixed_speed_controller control_longitudinal)
  endif()

  catkin_add_gtest(test_path_speed_profile
    test/unit/test_main.cpp test/unit/test_path_speed_profile.cpp)
  if(TARGET test_path_speed_profile)
    target_link_libraries(test_path_speed_profile control_longitudinal)
  endif()

  catkin_add_gtest(test_t870_command_mapper
    test/unit/test_main.cpp
    test/unit/test_t870_command_mapper.cpp
  )
  if(TARGET test_t870_command_mapper)
    target_link_libraries(test_t870_command_mapper
      control_vehicle
    )
  endif()
endif()

install(TARGETS
  control_lateral
  control_longitudinal
  control_vehicle
  control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
