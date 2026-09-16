cmake_minimum_required(VERSION 3.10)
project(path_planner LANGUAGES CXX)

set(PATH_PLANNER_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Path planner source root")
add_library(path_planner STATIC
  ${PATH_PLANNER_SOURCE_ROOT}/src/reference_path.cpp
  ${PATH_PLANNER_SOURCE_ROOT}/src/frenet_planner.cpp
  ${PATH_PLANNER_SOURCE_ROOT}/src/collision_checker.cpp
  ${PATH_PLANNER_SOURCE_ROOT}/src/boundary_checker.cpp
  ${PATH_PLANNER_SOURCE_ROOT}/src/obstacle_projection.cpp
  ${PATH_PLANNER_SOURCE_ROOT}/src/pose_jump_guard.cpp
)
target_include_directories(path_planner PUBLIC ${PATH_PLANNER_SOURCE_ROOT}/include)
target_compile_features(path_planner PUBLIC cxx_std_14)
target_compile_options(path_planner PRIVATE -Wall -Wextra -Wpedantic)
include(CTest)
if(BUILD_TESTING)
  add_executable(path_planner_core_test test/test_frenet_core.cpp)
  target_link_libraries(path_planner_core_test PRIVATE path_planner)
  add_test(NAME path_planner_core COMMAND path_planner_core_test)
endif()
