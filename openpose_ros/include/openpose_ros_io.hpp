#ifndef _OPENPOSE_ROS_IO
#define _OPENPOSE_ROS_IO

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write

#include <openpose_ros_msgs/msg/bounding_box.hpp>
#include <openpose_ros_msgs/msg/open_pose_human.hpp>
#include <openpose_ros_msgs/msg/open_pose_human_list.hpp>
#include <openpose_ros_msgs/msg/point_with_prob.hpp>

#include <openpose.hpp>
#include <openpose_flags.hpp>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros {

    class OpenPoseROSIO : public rclcpp::Node
    {
        private:
            rclcpp::Publisher<openpose_ros_msgs::msg::OpenPoseHumanList>::SharedPtr openpose_human_list_pub_;
            cv_bridge::CvImagePtr cv_img_ptr_;
            std_msgs::msg::Header image_header_;

            OpenPose* openpose_;

            rclcpp::Parameter output_topic_;

            rclcpp::Parameter display_output_flag_;
            rclcpp::Parameter print_keypoints_flag_;

            rclcpp::Parameter save_original_video_flag_;
            rclcpp::Parameter original_video_file_name_;
            bool original_video_writer_initialized_;
            cv::VideoWriter original_video_writer_;

            rclcpp::Parameter save_openpose_video_flag_;
            rclcpp::Parameter openpose_video_file_name_;
            bool openpose_video_writer_initialized_;
            cv::VideoWriter openpose_video_writer_;

            rclcpp::Parameter video_fps_;

        public:
            OpenPoseROSIO(const rclcpp::NodeOptions& options, OpenPose &openPose);

            ~OpenPoseROSIO(){}

            rclcpp::Parameter image_topic_;
            rclcpp::Parameter input_image_transport_type_;

            std::shared_ptr<rclcpp::Node> getPtr();

            void processImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

            void convertImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

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