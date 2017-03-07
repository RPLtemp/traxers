cmake_minimum_required(VERSION 2.8.3)
project(traxers_nav)

find_package(catkin REQUIRED COMPONENTS
  map_server
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS map_server nav_msgs roscpp
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_executable(occ_grid_to_costmap_node src/occ_grid_to_costmap_node.cpp)
target_link_libraries(occ_grid_to_costmap_node ${catkin_LIBRARIES} ${roslib_LIBRARIES})
