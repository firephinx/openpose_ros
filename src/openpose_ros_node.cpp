#define USE_CAFFE

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <string> // std::string

#include <opencv2/core/core.hpp> // cv::Mat & cv::Size

// ------------------------- OpenPose Library ROS Tutorial - Pose -------------------------
// This first example shows the user how to:
    // 1. Subscribe to a ros image topic
    // 2. Extract the pose of that image (`pose` module)
    // 3. Render the pose on a resized copy of the input image (`pose` module)
    // 4. Display the rendered pose (`gui` module)
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module: for the Array<float> class that the `pose` module needs
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively

// 3rdparty dependencies
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <glog/logging.h> // google::InitGoogleLogging
// OpenPose dependencies
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

// See all the available parameter options withe the `--help` flag. E.g. `./build/examples/openpose/openpose.bin --help`.
// Note: This command will show you flags for other unnecessary 3rdparty files. Check only the flags for the OpenPose
// executable. E.g. for `openpose.bin`, look for `Flags from examples/openpose/openpose.cpp:`.
// Debugging
DEFINE_int32(logging_level,             4,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
// Camera Topic
DEFINE_string(camera_topic,             "/camera/image_raw",      "Image topic that OpenPose will process.");
// OpenPose
DEFINE_string(model_folder,             "/path/to/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
DEFINE_string(model_pose,               "COCO",         "Model to be used (e.g. COCO, MPI, MPI_4_layers).");
DEFINE_string(net_resolution,           "656x368",      "Multiples of 16. If it is increased, the accuracy usually increases. If it is decreased,"
                                                        " the speed increases.");
DEFINE_string(resolution,               "1280x720",     "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " default images resolution.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
// OpenPose Rendering
DEFINE_bool(disable_blending,           false,          "If blending is enabled, it will merge the results with the original frame. If disabled, it"
                                                        " will only display the results.");
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e. wrong detections).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");

class RosImgSub
{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        cv_bridge::CvImagePtr cv_img_ptr_;

    public:
        RosImgSub(const std::string& image_topic): it_(nh_)
        {
            // Subscribe to input video feed and publish output video feed
            image_sub_ = it_.subscribe(image_topic, 1, &RosImgSub::convertImage, this);
            cv_img_ptr_ = nullptr;
        }

        ~RosImgSub(){}

        void convertImage(const sensor_msgs::ImageConstPtr& msg)
        {
            try
            {
                cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }

        cv_bridge::CvImagePtr& getCvImagePtr()
        {
            return cv_img_ptr_;
        }

};

int openPoseROSTutorial()
{
    op::log("OpenPose ROS Node", op::Priority::High);
    // ------------------------- INITIALIZATION -------------------------
    // Step 1 - Set logging level
        // - 0 will output all the logging messages
        // - 255 will output nothing
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 2 - Read Google flags (user defined configuration)
    // outputSize
    const auto outputSize = op::flagsToPoint(FLAGS_resolution, "1280x720");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "656x368");
    // netOutputSize
    const auto netOutputSize = netInputSize;
    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.", __LINE__, __FUNCTION__, __FILE__);
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 3 - Initialize all required classes
    op::CvMatToOpInput cvMatToOpInput{netInputSize, FLAGS_scale_number, (float)FLAGS_scale_gap};
    op::CvMatToOpOutput cvMatToOpOutput{outputSize};
    op::PoseExtractorCaffe poseExtractorCaffe{netInputSize, netOutputSize, outputSize, FLAGS_scale_number, poseModel,
                                              FLAGS_model_folder, FLAGS_num_gpu_start};
    op::PoseRenderer poseRenderer{netOutputSize, outputSize, poseModel, nullptr, (float)FLAGS_render_threshold,
                                  !FLAGS_disable_blending, (float)FLAGS_alpha_pose};
    op::OpOutputToCvMat opOutputToCvMat{outputSize};
    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
    poseExtractorCaffe.initializationOnThread();
    poseRenderer.initializationOnThread();

    // Step 5 - Initialize the image subscriber
    RosImgSub ris(FLAGS_camera_topic);

    int frame_count = 0;
    const std::chrono::high_resolution_clock::time_point timerBegin = std::chrono::high_resolution_clock::now();

    ros::spinOnce();

    // Step 6 - Continuously process images from image subscriber
    while (ros::ok())
    {
        // ------------------------- POSE ESTIMATION AND RENDERING -------------------------
        // Step 1 - Get cv_image ptr and check that it is not null
        cv_bridge::CvImagePtr cvImagePtr = ris.getCvImagePtr();
        if(cvImagePtr != nullptr)
        {
            cv::Mat inputImage = cvImagePtr->image;
    
            // Step 2 - Format input image to OpenPose input and output formats
            op::Array<float> netInputArray;
            std::vector<float> scaleRatios;
            std::tie(netInputArray, scaleRatios) = cvMatToOpInput.format(inputImage);
            double scaleInputToOutput;
            op::Array<float> outputArray;
            std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput.format(inputImage);
            // Step 3 - Estimate poseKeypoints
            poseExtractorCaffe.forwardPass(netInputArray, {inputImage.cols, inputImage.rows}, scaleRatios);
            const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();
            // Step 4 - Render poseKeypoints
            poseRenderer.renderPose(outputArray, poseKeypoints);
            // Step 5 - OpenPose output format to cv::Mat
            auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);

            // ------------------------- SHOWING RESULT AND CLOSING -------------------------
            // Step 1 - Show results
            cv::imshow("OpenPose ROS", outputImage);
            cv::waitKey(1);
            frame_count++;
        }
        ros::spinOnce();
    }

    // Measuring total time
    const double totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>
                              (std::chrono::high_resolution_clock::now()-timerBegin).count() * 1e-9;
    const std::string message = "Real-time pose estimation demo successfully finished. Total time: " 
                                + std::to_string(totalTimeSec) + " seconds. " + std::to_string(frame_count)
                                + " frames processed. Average fps is " + std::to_string(frame_count/totalTimeSec) + ".";
    op::log(message, op::Priority::Max);

    // Return successful message
    return 0;
}

int main(int argc, char** argv)
{
  google::InitGoogleLogging("openpose_ros_node");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "openpose_ros_node");

  return openPoseROSTutorial();
}