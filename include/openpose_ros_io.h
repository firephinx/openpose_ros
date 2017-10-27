#ifndef _OPENPOSE_ROS_IO
#define _OPENPOSE_ROS_IO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>

#include <openpose_ros/OpenPoseHuman.h>
#include <openpose_ros/OpenPoseHumanList.h>
#include <openpose_ros/PointWithProb.h>

#include <gflags_options.h>

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

        public:
            OpenPoseROSIO(const std::string& image_topic, const std::string& openpose_output_topic);

            ~OpenPoseROSIO(){}

            void convertImage(const sensor_msgs::ImageConstPtr& msg);

            std::shared_ptr<std::vector<op::Datum>> createDatum();

            bool display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);

            cv_bridge::CvImagePtr& getCvImagePtr();

            void printKeypoints(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);

            void publish(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);
    };
}

#endif