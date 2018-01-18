cmake_minimum_required(VERSION 2.8.3)
project(simple_joint_subscriber)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joint_subscriber
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
