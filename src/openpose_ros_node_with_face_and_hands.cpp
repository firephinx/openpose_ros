#define USE_CAFFE

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp> // cv::Mat & cv::Size

// ------------------------- OpenPose Library ROS Tutorial -------------------------
// Asynchronous mode: ideal for fast prototyping when performance is not an issue. The user emplaces/pushes and pops frames from the OpenPose wrapper when he desires to.

// This example shows the user how to use the OpenPose wrapper class:
    // 1. Extract and render keypoint / heatmap / PAF of that image
    // 2. Save the results on disc
    // 3. Display the rendered pose
    // Everything in a multi-thread scenario
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module:
        // For the Array<float> class that the `pose` module needs
        // For the Datum struct that the `thread` module sends between the queues
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively
// This file should only be used for the user to take specific examples.

// C++ std library dependencies
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <string>
#include <thread> // std::this_thread
#include <vector>
// Other 3rdparty dependencies
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <glog/logging.h> // google::InitGoogleLogging

// OpenPose dependencies
#include <openpose/headers.hpp>

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
DEFINE_string(resolution,               "1280x720",     "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " default images resolution.");
DEFINE_int32(num_gpu,                   -1,             "The number of GPU devices to use. If negative, it will use all the available GPUs in your"
                                                        " machine.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_int32(keypoint_scale,            0,              "Scaling of the (x,y) coordinates of the final pose data array, i.e. the scale of the (x,y)"
                                                        " coordinates that will be saved with the `write_keypoint` & `write_keypoint_json` flags."
                                                        " Select `0` to scale it to the original source resolution, `1`to scale it to the net output"
                                                        " size (set with `net_resolution`), `2` to scale it to the final output size (set with"
                                                        " `resolution`), `3` to scale it in the range [0,1], and 4 for range [-1,1]. Non related"
                                                        " with `scale_number` and `scale_gap`.");
// OpenPose Body Pose
DEFINE_string(model_pose,               "COCO",         "Model to be used (e.g. COCO, MPI, MPI_4_layers).");
DEFINE_string(net_resolution,           "656x368",      "Multiples of 16. If it is increased, the accuracy usually increases. If it is decreased,"
                                                        " the speed increases.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_bool(heatmaps_add_parts,         false,          "If true, it will add the body part heatmaps to the final op::Datum::poseHeatMaps array"
                                                        " (program speed will decrease). Not required for our library, enable it only if you intend"
                                                        " to process this information later. If more than one `add_heatmaps_X` flag is enabled, it"
                                                        " will place then in sequential memory order: body parts + bkg + PAFs. It will follow the"
                                                        " order on POSE_BODY_PART_MAPPING in `include/openpose/pose/poseParameters.hpp`.");
DEFINE_bool(heatmaps_add_bkg,           false,          "Same functionality as `add_heatmaps_parts`, but adding the heatmap corresponding to"
                                                        " background.");
DEFINE_bool(heatmaps_add_PAFs,          false,          "Same functionality as `add_heatmaps_parts`, but adding the PAFs.");
DEFINE_int32(heatmaps_scale,            2,              "Set 0 to scale op::Datum::poseHeatMaps in the range [0,1], 1 for [-1,1]; and 2 for integer"
                                                        " rounded [0,255].");
// OpenPose Face
DEFINE_bool(face,                       true,           "Enables face keypoint detection. It will share some parameters from the body pose, e.g."
                                                        " `model_folder`.");
DEFINE_string(face_net_resolution,      "368x368",      "Multiples of 16. Analogous to `net_resolution` but applied to the face keypoint detector."
                                                        " 320x320 usually works fine while giving a substantial speed up when multiple faces on the"
                                                        " image.");
// OpenPose Hand
DEFINE_bool(hand,                       true,           "Enables hand keypoint detection. It will share some parameters from the body pose, e.g."
                                                        " `model_folder`.");
DEFINE_string(hand_net_resolution,      "368x368",      "Multiples of 16. Analogous to `net_resolution` but applied to the hand keypoint detector.");
DEFINE_int32(hand_scale_number,         1,              "Analogous to `scale_number` but applied to the hand keypoint detector. Our best results"
                                                        " were found with `hand_scale_number` = 6 and `hand_scale_range` = 0.4");
DEFINE_double(hand_scale_range,         0.4,            "Analogous purpose than `scale_gap` but applied to the hand keypoint detector. Total range"
                                                        " between smallest and biggest scale. The scales will be centered in ratio 1. E.g. if"
                                                        " scaleRange = 0.4 and scalesNumber = 2, then there will be 2 scales, 0.8 and 1.2.");

DEFINE_bool(hand_tracking,              false,          "Adding hand tracking might improve hand keypoints detection for webcam (if the frame rate"
                                                        " is high enough, i.e. >7 FPS per GPU) and video. This is not person ID tracking, it"
                                                        " simply looks for hands in positions at which hands were located in previous frames, but"
                                                        " it does not guarantee the same person ID among frames");
// OpenPose Rendering
DEFINE_int32(part_to_show,              0,              "Part to show from the start.");
DEFINE_bool(disable_blending,           false,          "If blending is enabled, it will merge the results with the original frame. If disabled, it"
                                                        " will only display the results.");
// OpenPose Rendering Pose
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e. wrong detections).");
DEFINE_int32(render_pose,               2,              "Set to 0 for no rendering, 1 for CPU rendering (slightly faster), and 2 for GPU rendering"
                                                        " (slower but greater functionality, e.g. `alpha_X` flags). If rendering is enabled, it will"
                                                        " render both `outputData` and `cvOutputData` with the original image and desired body part"
                                                        " to be shown (i.e. keypoints, heat maps or PAFs).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");
DEFINE_double(alpha_heatmap,            0.7,            "Blending factor (range 0-1) between heatmap and original frame. 1 will only show the"
                                                        " heatmap, 0 will only show the frame. Only valid for GPU rendering.");
// OpenPose Rendering Face
DEFINE_double(face_render_threshold,    0.4,            "Analogous to `render_threshold`, but applied to the face keypoints.");
DEFINE_int32(face_render,               -1,             "Analogous to `render_pose` but applied to the face. Extra option: -1 to use the same"
                                                        " configuration that `render_pose` is using.");
DEFINE_double(face_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to face.");
DEFINE_double(face_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to face.");
// OpenPose Rendering Hand
DEFINE_double(hand_render_threshold,    0.2,            "Analogous to `render_threshold`, but applied to the hand keypoints.");
DEFINE_int32(hand_render,               -1,             "Analogous to `render_pose` but applied to the hand. Extra option: -1 to use the same"
                                                        " configuration that `render_pose` is using.");
DEFINE_double(hand_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to hand.");
DEFINE_double(hand_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to hand.");
// Result Saving
DEFINE_string(write_images,             "",             "Directory to write rendered frames in `write_images_format` image format.");
DEFINE_string(write_images_format,      "png",          "File extension and format for `write_images`, e.g. png, jpg or bmp. Check the OpenCV"
                                                        " function cv::imwrite for all compatible extensions.");
DEFINE_string(write_video,              "",             "Full file path to write rendered frames in motion JPEG video format. It might fail if the"
                                                        " final path does not finish in `.avi`. It internally uses cv::VideoWriter.");
DEFINE_string(write_keypoint,           "",             "Directory to write the people body pose keypoint data. Set format with `write_keypoint_format`.");
DEFINE_string(write_keypoint_format,    "yml",          "File extension and format for `write_keypoint`: json, xml, yaml & yml. Json not available"
                                                        " for OpenCV < 3.0, use `write_keypoint_json` instead.");
DEFINE_string(write_keypoint_json,      "",             "Directory to write people pose data in *.json format, compatible with any OpenCV version.");
DEFINE_string(write_coco_json,          "",             "Full file path to write people pose data with *.json COCO validation format.");
DEFINE_string(write_heatmaps,           "",             "Directory to write heatmaps in *.png format. At least 1 `add_heatmaps_X` flag must be"
                                                        " enabled.");
DEFINE_string(write_heatmaps_format,    "png",          "File extension and format for `write_heatmaps`, analogous to `write_images_format`."
                                                        " Recommended `png` or any compressed and lossless format.");

// This worker will just subscribe to the image topic specified above.
class UserInputClass
{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        cv_bridge::CvImagePtr cv_img_ptr_;

    public:
        UserInputClass(const std::string& image_topic): it_(nh_)
        {
            // Subscribe to input video feed and publish output video feed
            image_sub_ = it_.subscribe(image_topic, 1, &UserInputClass::convertImage, this);
            cv_img_ptr_ = nullptr;
        }

        ~UserInputClass(){}

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

        std::shared_ptr<std::vector<op::Datum>> createDatum()
        {
            // Close program when empty frame
            if (cv_img_ptr_ == nullptr)
            {
                return nullptr;
            }
            else // if (cv_img_ptr_ == nullptr)
            {
                // Create new datum
                auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
                datumsPtr->emplace_back();
                auto& datum = datumsPtr->at(0);

                // Fill datum
                datum.cvInputData = cv_img_ptr_->image;

                return datumsPtr;
            }
        }

        cv_bridge::CvImagePtr& getCvImagePtr()
        {
            return cv_img_ptr_;
        }
};

// This worker will just display the result
class UserOutputClass
{
public:
    void display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
    {
        // User's displaying/saving/other processing here
            // datum.cvOutputData: rendered frame with pose or heatmaps
            // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            cv::imshow("OpenPose ROS", datumsPtr->at(0).cvOutputData);
            cv::waitKey(1); // It displays the image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    }
};

int openPoseROSTutorialWithFaceAndHands()
{
    // logging_level
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    // op::ConfigureLog::setPriorityThreshold(op::Priority::None); // To print all logging messages

    op::log("OpenPose ROS Node with Face and Hands", op::Priority::High);
    const auto timerBegin = std::chrono::high_resolution_clock::now();

    // Applying user defined configuration - Google flags to program variables
    // outputSize
    const auto outputSize = op::flagsToPoint(FLAGS_resolution, "1280x720");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "656x368");
    // faceNetInputSize
    const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
    // handNetInputSize
    const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // keypointScale
    const auto keypointScale = op::flagsToScaleMode(FLAGS_keypoint_scale);
    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg, FLAGS_heatmaps_add_PAFs);
    op::check(FLAGS_heatmaps_scale >= 0 && FLAGS_heatmaps_scale <= 2, "Non valid `heatmaps_scale`.", __LINE__, __FUNCTION__, __FILE__);
    const auto heatMapScale = (FLAGS_heatmaps_scale == 0 ? op::ScaleMode::PlusMinusOne
                               : (FLAGS_heatmaps_scale == 1 ? op::ScaleMode::ZeroToOne : op::ScaleMode::UnsignedChar ));
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    // Configure OpenPose
    op::Wrapper<std::vector<op::Datum>> opWrapper{op::ThreadManagerMode::Asynchronous};
    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{netInputSize, outputSize, keypointScale, FLAGS_num_gpu,
                                                  FLAGS_num_gpu_start, FLAGS_scale_number, (float)FLAGS_scale_gap,
                                                  op::flagsToRenderMode(FLAGS_render_pose), poseModel,
                                                  !FLAGS_disable_blending, (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, FLAGS_model_folder,
                                                  heatMapTypes, heatMapScale, (float)FLAGS_render_threshold};
    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{FLAGS_face, faceNetInputSize, op::flagsToRenderMode(FLAGS_face_render, FLAGS_render_pose),
                                                  (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{FLAGS_hand, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
                                                  FLAGS_hand_tracking, op::flagsToRenderMode(FLAGS_hand_render, FLAGS_render_pose),
                                                  (float)FLAGS_hand_alpha_pose, (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
    // Consumer (comment or use default argument to disable any output)
    const bool displayGui = false;
    const bool guiVerbose = false;
    const bool fullScreen = false;
    const op::WrapperStructOutput wrapperStructOutput{displayGui, guiVerbose, fullScreen, FLAGS_write_keypoint,
                                                      op::stringToDataFormat(FLAGS_write_keypoint_format), FLAGS_write_keypoint_json,
                                                      FLAGS_write_coco_json, FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video,
                                                      FLAGS_write_heatmaps, FLAGS_write_heatmaps_format};
    // Configure wrapper
    opWrapper.configure(wrapperStructPose, wrapperStructFace, wrapperStructHand, op::WrapperStructInput{}, wrapperStructOutput);
    // Set to single-thread running (e.g. for debugging purposes)
    // opWrapper.disableMultiThreading();

    op::log("Starting thread(s)", op::Priority::High);
    opWrapper.start();

    // User processing
    UserInputClass userInputClass(FLAGS_camera_topic);
    UserOutputClass userOutputClass;
    while (ros::ok())
    {
        // Push frame
        auto datumToProcess = userInputClass.createDatum();
        if (datumToProcess != nullptr)
        {
            auto successfullyEmplaced = opWrapper.waitAndEmplace(datumToProcess);
            // Pop frame
            std::shared_ptr<std::vector<op::Datum>> datumProcessed;
            if (successfullyEmplaced && opWrapper.waitAndPop(datumProcessed))
                userOutputClass.display(datumProcessed);
            else
                op::log("Processed datum could not be emplaced.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }

        ros::spinOnce();
    }

    op::log("Stopping thread(s)", op::Priority::High);
    opWrapper.stop();

    // Measuring total time
    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count() * 1e-9;
    const auto message = "Real-time pose estimation demo successfully finished. Total time: " + std::to_string(totalTimeSec) + " seconds.";
    op::log(message, op::Priority::High);

    return 0;
}

int main(int argc, char *argv[])
{
    // Initializing google logging (Caffe uses it for logging)
    google::InitGoogleLogging("openpose_ros_node_with_face_and_hands");

    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Initializing ros
    ros::init(argc, argv, "openpose_ros_node_with_face_and_hands");

    // Running openPoseROSTutorialWithFaceAndHands
    return openPoseROSTutorialWithFaceAndHands();
}
