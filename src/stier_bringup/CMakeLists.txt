cmake_minimum_required(VERSION 3.0.2)
project(stier_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/parking_pipeline_ros.py test/frenet_pipeline_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  foreach(course yongin hongik hongik_s)
    foreach(lookahead 1 2)
      add_rostest(test/frenet_pipeline.test ARGS course:=${course} lookahead:=${lookahead}
        DEPENDENCIES control_node path_planner_node)
    endforeach()
    foreach(kind t parallel)
      foreach(side left right)
        add_rostest(test/parking_pipeline.test ARGS course:=${course} kind:=${kind} side:=${side}
          DEPENDENCIES control_node)
      endforeach()
    endforeach()
  endforeach()
  roslaunch_add_file_check(launch)
endif()
