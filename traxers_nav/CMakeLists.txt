cmake_minimum_required(VERSION 2.8.3)
project(traxers_nav)

find_package(catkin REQUIRED COMPONENTS
  map_server
  nav_msgs
  roscpp
)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES value_iteration
  CATKIN_DEPENDS map_server nav_msgs roscpp
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

FILE(GLOB VALUE_ITERATION_CPP src/lib/*.cpp)

add_library(value_iteration ${VALUE_ITERATION_CPP})
add_dependencies(value_iteration ${catkin_EXPORTED_TARGETS})
target_link_libraries(value_iteration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(occ_grid_to_costmap_node src/occ_grid_to_costmap_node.cpp)
target_link_libraries(occ_grid_to_costmap_node ${catkin_LIBRARIES} ${roslib_LIBRARIES} value_iteration)
