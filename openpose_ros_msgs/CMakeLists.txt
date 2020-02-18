cmake_minimum_required(VERSION 3.5)
project(openpose_ros_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

set(msg_files
  "msg/BoundingBox.msg"
  "msg/OpenPoseHuman.msg"
  "msg/OpenPoseHumanList.msg"
  "msg/PointWithProb.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
