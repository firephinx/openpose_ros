cmake_minimum_required(VERSION 2.8.7)
project(openpose_ros_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

add_message_files(
  FILES
  BoundingBox.msg
  OpenPoseHuman.msg
  OpenPoseHumanList.msg
  PointWithProb.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)