cmake_minimum_required(VERSION 2.8.3)
project(boustrophedon_server)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  boustrophedon_msgs
  actionlib
  tf
  geometry_msgs
  nav_msgs
  rosparam_shortcuts
  )

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
