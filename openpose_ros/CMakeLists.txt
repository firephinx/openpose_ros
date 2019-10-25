cmake_minimum_required(VERSION 2.8.7)
project(openpose_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  openpose_ros_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

# find_package(CUDA REQUIRED) # Uncomment this line if you have trouble compiling
find_package(OpenCV REQUIRED)
find_package(GFlags)
find_package(Glog)
find_package(OpenPose REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${CAFFE_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS} 
  ${GFLAGS_INCLUDE_DIR} 
  ${GLOG_INCLUDE_DIR} 
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/openpose_ros_node.cpp src/openpose.cpp src/openpose_ros_io.cpp 
                                    src/openpose_flags.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenPose_LIBS} 
  ${GFLAGS_LIBRARY} 
  ${GLOG_LIBRARY} 
)