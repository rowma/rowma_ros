cmake_minimum_required(VERSION 2.8.3)
project(rowma_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  Eigen3
)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
