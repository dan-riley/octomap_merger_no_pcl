cmake_minimum_required(VERSION 2.8.3)
project(octomap_merger)

find_package(catkin REQUIRED COMPONENTS
  octomap_ros
  octomap_msgs
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  OctomapArray.msg
)

generate_messages(
   DEPENDENCIES
   nav_msgs
   octomap_msgs
 )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(map_merger src/map_merger.cpp)
target_link_libraries(map_merger ${catkin_LIBRARIES})

add_executable(octomap_merger src/octomap_merger.cpp)
target_link_libraries(octomap_merger map_merger ${catkin_LIBRARIES})

add_executable(octomap_merger_node src/octomap_merger_node.cpp)
target_link_libraries(octomap_merger_node map_merger ${catkin_LIBRARIES})
