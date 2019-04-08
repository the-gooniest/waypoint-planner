cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

