cmake_minimum_required(VERSION 2.8.3)
project(second_race)
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
