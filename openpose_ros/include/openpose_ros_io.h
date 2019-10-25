#ifndef _OPENPOSE_ROS_IO
#define _OPENPOSE_ROS_IO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write

#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>

#include <openpose.h>
#include <openpose_flags.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros {

    class OpenPoseROSIO
    {
        private:
            ros::NodeHandle nh_;
            ros::Publisher openpose_human_list_pub_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            cv_bridge::CvImagePtr cv_img_ptr_;
            std_msgs::Header image_header_;

            OpenPose* openpose_;

            bool display_output_flag_;
            bool print_keypoints_flag_;

            bool save_original_video_flag_;
            std::string original_video_file_name_;
            bool original_video_writer_initialized_;
            cv::VideoWriter original_video_writer_;

            bool save_openpose_video_flag_;
            std::string openpose_video_file_name_;
            bool openpose_video_writer_initialized_;
            cv::VideoWriter openpose_video_writer_;

            int video_fps_;

        public:
            OpenPoseROSIO(OpenPose &openPose);

            ~OpenPoseROSIO(){}

            void processImage(const sensor_msgs::ImageConstPtr& msg);

            void convertImage(const sensor_msgs::ImageConstPtr& msg);

            std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum();

            bool display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            bool saveOriginalVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            bool saveOpenPoseVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            cv_bridge::CvImagePtr& getCvImagePtr();

            void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            void publish(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            void stop();
    };
}

#endif