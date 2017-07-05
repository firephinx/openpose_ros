# openpose_ros

Example ros catkin package that utilizes the openpose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose.

## System
Tested on:
* Ubuntu 14.04.5 
* Ros Indigo
* CUDA 8.0
* cuDNN 5.1
* OpenCV 3.2

## Installation Steps

1. Clone the glog_catkin repository into your catkin_workspace/src directory.
   ```bash
   git clone https://github.com/ethz-asl/glog_catkin.git
   ```
2. Clone the gflags repository into your catkin_workspace/src directory.
   ```bash
   git clone https://github.com/davetcoleman/gflags.git
   ```
3. Clone the catkin_simple repository into your catkin_workspace/src directory.
   ```bash
   git clone https://github.com/catkin/catkin_simple.git
   ```
4. Install openpose from here (not in catkin_workspace): https://github.com/firephinx/openpose
5. Clone this repository into your catkin_workspace/src directory.
6. Modify the following lines in the CMakeLists.txt to the proper directories of where you installed caffe and openpose:
   ```bash
   set(CAFFE_DIR /path/to/caffe)
   set(OPENPOSE_DIR /path/to/openpose)
   ```
7. Modify the model_folder line in src/openpose_ros_node.cpp to where openpose is installed.
   ```bash
   DEFINE_string(model_folder,             "/path/to/openpose/models/",      "Folder where the pose models (COCO and MPI) are located.");
   ```
8. Modify the image_topic line in src/openpose_ros_node.cpp to the image_topic you want to process.
   ```bash
   DEFINE_string(camera_topic,             "/camera/image_raw",      "Image topic that OpenPose will process.");
   ```
9. Modify the other parameters in src/openpose_ros_node.cpp to your liking such as the net_resolution and resolution of the image.
10. Run catkin_make from your catkin_workspace directory. (It will fail the first time so run it twice.)
11. If cv_bridge is causing you errors and/or you decide to use OpenCV 3.2, copy the cv_bridge folder from https://github.com/ros-perception/vision_opencv into your catkin_workspace/src directory. 
12. If you have problems with CUDA during catkin_make, uncomment this line # find_package(CUDA REQUIRED)

## Running
```bash
source catkin_workspace/devel/setup.bash
rosrun openpose_ros openpose_ros_node
```
