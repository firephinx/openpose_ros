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

#include <openpose.h>
#include <openpose_ros_io.h>
#include <openpose_flags.h>

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
    openpose_ros::OpenPoseROSIO openPoseROSIO(openPose);
    
    ros::spin();

    op::opLog("Stopping thread(s)", op::Priority::High);
    openPose.stop();
    openPoseROSIO.stop();

    // Measuring total time
    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count()
                            * 1e-9;
    const auto message = "Real-time pose estimation demo successfully finished. Total time: "
                       + std::to_string(totalTimeSec) + " seconds.";
    op::opLog(message, op::Priority::High);

    return 0;
}

int main(int argc, char *argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Initializing ros
    ros::init(argc, argv, "openpose_ros_node");

    // Running openPoseROS
    return openPoseROS();
}
