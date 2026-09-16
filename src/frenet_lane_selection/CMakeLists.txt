cmake_minimum_required(VERSION 3.10)
project(frenet_lane_selection LANGUAGES CXX)

# Deliberately NOT a catkin package. No ROS, node, launch or actuator interface.
add_library(frenet_lane_selection STATIC src/lane_selector.cpp)
target_include_directories(frenet_lane_selection PUBLIC include)
target_compile_features(frenet_lane_selection PUBLIC cxx_std_14)
target_compile_options(frenet_lane_selection PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(lane_selection_test test/test_lane_selector.cpp)
  target_link_libraries(lane_selection_test PRIVATE frenet_lane_selection)
  add_test(NAME lane_selection_core COMMAND lane_selection_test)
endif()
