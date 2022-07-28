cmake_minimum_required(VERSION 2.8.3)
project(boustrophedon_server)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  boustrophedon_msgs
  actionlib
  tf
  geometry_msgs
  nav_msgs
  rosparam_shortcuts
  )
find_package(sentry)
find_package(CGAL REQUIRED COMPONENTS Core)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  boustrophedon_msgs
  actionlib
  tf
  geometry_msgs
  nav_msgs
  rosparam_shortcuts
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE SPLINE_SOURCES
  include/spline/*.cpp
)

add_executable(boustrophedon_planner_server
  src/boustrophedon_server/boustrophedon_planner_node.cpp
  src/boustrophedon_server/boustrophedon_planner_server.cpp
  src/boustrophedon_server/striping_planner.cpp
  src/boustrophedon_server/outline_planner.cpp
  src/boustrophedon_server/cgal_utils.cpp
  src/boustrophedon_server/boustrophedon_types.cpp
  src/boustrophedon_server/cellular_decomposition/polygon_decomposer.cpp
  src/boustrophedon_server/cellular_decomposition/cell.cpp
  ${SPLINE_SOURCES}
)

target_link_libraries(boustrophedon_planner_server
  ${catkin_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core)
add_dependencies(boustrophedon_planner_server ${catkin_EXPORTED_TARGETS})

target_link_libraries(boustrophedon_planner_server sentry)

install(
  TARGETS
  boustrophedon_planner_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

# create a gtest executable with target name "test_mynode"
# which is not built by "make all" but only by "make tests"
# add one/multiple source files to the executable target
# register the rostest launch file "test/mynode.test"
# add_rostest_gtest(tests_mynode test/mynode.test src/test/test_mynode.cpp [more cpp files])

# you will likely need to link other libraries to your test executable
# target_link_libraries(tests_mynode [libraries to depend on, e.g. ${catkin_LIBRARIES}])

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)


  # Level 1 (Unit tests sans ROS)
  catkin_add_gtest(boustrophedon_planner_utest
    test/src/striping_planner_test.cpp
    src/boustrophedon_server/striping_planner.cpp
    src/boustrophedon_server/cgal_utils.cpp
    src/boustrophedon_server/boustrophedon_types.cpp
    ${SPLINE_SOURCES}
  )

  target_link_libraries(boustrophedon_planner_utest
    ${catkin_LIBRARIES}
    CGAL::CGAL
    CGAL::CGAL_Core
    gtest_main
  )
  add_dependencies(boustrophedon_planner_utest boustrophedon_planner_server)


  # Level 2 (ROS node tests -- uses rostest to launch its own roscore)
  add_rostest_gtest(boustrophedon_planner_client_tests
    test/bplanner_client_test.test
    test/src/boustrophedon_planner_client_test.cpp
    test/gtest_ros_main.cpp
  )

  target_link_libraries(boustrophedon_planner_client_tests
    ${catkin_LIBRARIES}
    CGAL::CGAL
    CGAL::CGAL_Core
  )
  add_dependencies(boustrophedon_planner_client_tests boustrophedon_planner_server ${catkin_EXPORTED_TARGETS})

  # Example:
  ## Using gtest
  # add_rostest_gtest(tests_mynode test/mynode.test src/test/test_mynode.cpp [more cpp files])
  # target_link_libraries(tests_mynode ${catkin_LIBRARIES})
  ## Using python nosetest
  # catkin_add_nosetests(path/to/my_test.py)

endif()
