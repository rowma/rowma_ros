cmake_minimum_required(VERSION 2.8.3)
project(rowma_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
