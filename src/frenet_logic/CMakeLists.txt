cmake_minimum_required(VERSION 3.10)
project(frenet_logic LANGUAGES CXX)

# In this workspace point this at the existing lidar_path_planning package.
# In the generated handoff all files are local, so no override is necessary.
set(FRENET_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Frenet core source root")
add_library(frenet_logic STATIC
  ${FRENET_SOURCE_ROOT}/src/reference_path.cpp
  ${FRENET_SOURCE_ROOT}/src/frenet_planner.cpp
  ${FRENET_SOURCE_ROOT}/src/collision_checker.cpp
  ${FRENET_SOURCE_ROOT}/src/boundary_checker.cpp
  ${FRENET_SOURCE_ROOT}/src/obstacle_projection.cpp
  ${FRENET_SOURCE_ROOT}/src/pose_jump_guard.cpp
)
target_include_directories(frenet_logic PUBLIC ${FRENET_SOURCE_ROOT}/include)
target_compile_features(frenet_logic PUBLIC cxx_std_14)
target_compile_options(frenet_logic PRIVATE -Wall -Wextra -Wpedantic)
include(CTest)
if(BUILD_TESTING)
  add_executable(frenet_core_test test/test_frenet_core.cpp)
  target_link_libraries(frenet_core_test PRIVATE frenet_logic)
  add_test(NAME frenet_logic_core COMMAND frenet_core_test)
endif()
