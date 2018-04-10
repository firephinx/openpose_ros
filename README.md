# openpose_ros

Example ROS catkin package that utilizes the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose.

## System
Tested on:
* Ubuntu 14.04 / Ubuntu 16.04
* ROS Indigo / Kinetic
* CUDA 8.0
* cuDNN 5.1 / cuDNN 6.0
* OpenCV 3.3 / OpenCV 3.4

## Installation Steps

1. Install openpose (not in catkin_workspace) using instructions from here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md. Make sure to run `sudo make install` in the build folder at the end.
   ```bash
   git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
   ```
2. Clone this repository into your catkin_workspace/src directory.
   ```bash
   git clone https://github.com/firephinx/openpose_ros.git
   ```
3. Modify the model_folder line in openpose_ros/src/gflags_options.cpp to where openpose is installed.
   ```bash
   DEFINE_string(model_folder,             "/path/to/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
   ```
4. Modify the image_topic parameter in openpose_ros/launch/openpose_ros.launch to the image_topic you want to process.
   ```bash
   <param name="image_topic"     value="/camera/image_raw" />
   ```
5. Modify the other parameters in openpose_ros/src/gflags_options.cpp and openpose_ros/launch/openpose_ros.launch to your liking such as enabling face and hands detection.
6. Run catkin_make from your catkin_workspace directory.

### Potential Installation Issues
1. If cv_bridge is causing you errors and/or you decide to use OpenCV 3.2+, copy the cv_bridge folder from https://github.com/ros-perception/vision_opencv into your catkin_workspace/src directory. 
2. If you have problems with CUDA during catkin_make, uncomment this line in CMakeLists.txt # find_package(CUDA REQUIRED)

## Running
```bash
source catkin_workspace/devel/setup.bash
roslaunch openpose_ros openpose_ros.launch
```
