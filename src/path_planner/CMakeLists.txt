cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  planning_interfaces
  rospy
  roscpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planner_core
  CATKIN_DEPENDS geometry_msgs nav_msgs planning_interfaces rospy roscpp
                 std_msgs std_srvs tf2 tf2_geometry_msgs visualization_msgs
)

catkin_install_python(PROGRAMS test/frenet_field_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(path_planner_core
  src/reference_path.cpp
  src/frenet_planner.cpp
  src/collision_checker.cpp
  src/boundary_checker.cpp
  src/obstacle_projection.cpp
  src/pose_jump_guard.cpp
)
target_include_directories(path_planner_core PUBLIC include ${catkin_INCLUDE_DIRS})
target_compile_features(path_planner_core PUBLIC cxx_std_14)
target_compile_options(path_planner_core PRIVATE -Wall -Wextra -Wpedantic)
add_dependencies(path_planner_core ${catkin_EXPORTED_TARGETS})

add_executable(path_planner_node src/path_planner_node.cpp)
target_compile_features(path_planner_node PRIVATE cxx_std_14)
target_compile_options(path_planner_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(path_planner_node path_planner_core ${catkin_LIBRARIES})
add_dependencies(path_planner_node ${catkin_EXPORTED_TARGETS})

install(TARGETS path_planner_core path_planner_node
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
install(FILES
  test/frenet_field_control.yaml
  test/frenet_field_planner.yaml
  test/frenet_field_test.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(path_planner_core_test test/test_frenet_core.cpp)
  target_compile_features(path_planner_core_test PRIVATE cxx_std_14)
  target_link_libraries(path_planner_core_test PRIVATE path_planner_core)
  add_test(NAME path_planner_core COMMAND path_planner_core_test)
endif()
