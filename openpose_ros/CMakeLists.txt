cmake_minimum_required(VERSION 3.5)
project(openpose_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(openpose_ros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

#find_package(CUDA REQUIRED) # Uncomment this line if you have trouble compiling
find_package(OpenCV REQUIRED)
find_package(GFlags)
find_package(Glog)
find_package(OpenPose REQUIRED)

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${CAFFE_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/openpose_ros_node.cpp
  src/openpose.cpp
  src/openpose_ros_io.cpp
  src/openpose_flags.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  openpose_ros_msgs
  cv_bridge
  image_transport
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  ${OpenPose_LIBS} 
  ${GFLAGS_LIBRARY} 
  ${GLOG_LIBRARY} 
)

ament_export_include_directories(include)

ament_export_dependencies(rclcpp std_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
