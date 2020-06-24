// ------------------------- OpenPose Library Tutorial - Wrapper - Example 1 - Asynchronous -------------------------
// Asynchronous mode: ideal for fast prototyping when performance is not an issue. The user emplaces/pushes and pops frames from the OpenPose wrapper
// when he desires to.

// This example shows the user how to use the OpenPose wrapper class:
    // 1. User reads images
    // 2. Extract and render keypoint / heatmap / PAF of that image
    // 3. Save the results on disk
    // 4. User displays the rendered pose
    // Everything in a multi-thread scenario
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module:
        // For the Array<float> class that the `pose` module needs
        // For the Datum struct that the `thread` module sends between the queues
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::opLog respectively
// This file should only be used for the user to take specific examples.

// C++ std library dependencies
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <thread> // std::this_thread

#include <openpose.hpp>
#include <openpose_ros_io.hpp>
#include <openpose_flags.hpp>

int openPoseROS()
{
    // logging_level
    op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    op::opLog("Starting pose estimation demo.", op::Priority::High);
    const auto timerBegin = std::chrono::high_resolution_clock::now();

    openpose_ros::OpenPose openPose;

    op::opLog("Starting thread(s)", op::Priority::High);
    openPose.start();

    // OpenPose processing
    auto options = rclcpp::NodeOptions().
        allow_undeclared_parameters(true).
        automatically_declare_parameters_from_overrides(true);
    auto openPoseROSIO = std::make_shared<openpose_ros::OpenPoseROSIO>(options, openPose);
    auto sub = image_transport::create_subscription(openPoseROSIO->getPtr().get(),
        openPoseROSIO->image_topic_.as_string(),
        std::bind(&openpose_ros::OpenPoseROSIO::processImage, openPoseROSIO, std::placeholders::_1),
        openPoseROSIO->input_image_transport_type_.as_string()
    );

    rclcpp::spin(openPoseROSIO);

    op::opLog("Stopping thread(s)", op::Priority::High);
    openPose.stop();
    openPoseROSIO->stop();

    // Measuring total time
    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count()
                            * 1e-9;
    const auto message = "Real-time pose estimation demo successfully finished. Total time: "
                       + std::to_string(totalTimeSec) + " seconds.";
    op::opLog(message, op::Priority::High);

    rclcpp::shutdown();
    return 0;
}

int main(int argc, char *argv[])
{
    // Initializing ros
    rclcpp::init(argc, argv);

    // Parsing command line flags
    gflags::AllowCommandLineReparsing();
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    openPoseROS();
}
