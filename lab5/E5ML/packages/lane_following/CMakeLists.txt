cmake_minimum_required(VERSION 2.8.3)
project(lane_following)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  cv_bridge
  duckietown_msgs
  message_generation
)

add_service_files(
  FILES 
  updatepos.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

add_dependencies(${catkin_EXPORTED_TARGETS})

catkin_package(
   CATKIN_DEPENDS
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)