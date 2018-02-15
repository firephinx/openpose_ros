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
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively
// This file should only be used for the user to take specific examples.

// C++ std library dependencies
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <thread> // std::this_thread

#include <openpose_ros_io.h>
#include <gflags_options.h>

int openPoseROS()
{
    // logging_level
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    op::log("Starting pose estimation demo.", op::Priority::High);
    const auto timerBegin = std::chrono::high_resolution_clock::now();

    // Applying user defined configuration - Google flags to program variables
    // outputSize
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    // faceNetInputSize
    const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
    // handNetInputSize
    const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // JSON saving
    const auto writeJson = (!FLAGS_write_json.empty() ? FLAGS_write_json : FLAGS_write_keypoint_json);
    if (!FLAGS_write_keypoint.empty() || !FLAGS_write_keypoint_json.empty())
        op::log("Flags `write_keypoint` and `write_keypoint_json` are deprecated and will eventually be removed."
                " Please, use `write_json` instead.", op::Priority::Max);
    // keypointScale
    const auto keypointScale = op::flagsToScaleMode(FLAGS_keypoint_scale);
    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);
    const auto heatMapScale = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    // Enabling Google Logging
    const bool enableGoogleLogging = true;
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    // Configure OpenPose
    op::Wrapper<std::vector<op::Datum>> opWrapper{op::ThreadManagerMode::Asynchronous};
    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{!FLAGS_body_disable, netInputSize, outputSize, keypointScale,
                                                  FLAGS_num_gpu, FLAGS_num_gpu_start, FLAGS_scale_number,
                                                  (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose),
                                                  poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, FLAGS_model_folder,
                                                  heatMapTypes, heatMapScale, FLAGS_part_candidates,
                                                  (float)FLAGS_render_threshold, FLAGS_number_people_max,
                                                  enableGoogleLogging, FLAGS_3d};
    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{FLAGS_face, faceNetInputSize,
                                                  op::flagsToRenderMode(FLAGS_face_render, FLAGS_render_pose),
                                                  (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap,
                                                  (float)FLAGS_face_render_threshold};
    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{FLAGS_hand, handNetInputSize, FLAGS_hand_scale_number,
                                                  (float)FLAGS_hand_scale_range, FLAGS_hand_tracking,
                                                  op::flagsToRenderMode(FLAGS_hand_render, FLAGS_render_pose),
                                                  (float)FLAGS_hand_alpha_pose, (float)FLAGS_hand_alpha_heatmap,
                                                  (float)FLAGS_hand_render_threshold};
    // Consumer (comment or use default argument to disable any output)
    const auto displayMode = op::DisplayMode::NoDisplay;
    const bool guiVerbose = false;
    const bool fullScreen = false;
    const op::WrapperStructOutput wrapperStructOutput{displayMode, guiVerbose, fullScreen, FLAGS_write_keypoint,
                                                      op::stringToDataFormat(FLAGS_write_keypoint_format),
                                                      writeJson, FLAGS_write_coco_json,
                                                      FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video,
                                                      FLAGS_write_heatmaps, FLAGS_write_heatmaps_format};
    // Configure wrapper
    op::log("Configuring OpenPose wrapper.", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    opWrapper.configure(wrapperStructPose, wrapperStructFace, wrapperStructHand, op::WrapperStructInput{},
                        wrapperStructOutput);
    // Set to single-thread running (to debug and/or reduce latency)
    if (FLAGS_disable_multi_thread)
       opWrapper.disableMultiThreading();

    op::log("Starting thread(s)", op::Priority::High);
    opWrapper.start();

    // OpenPose processing
    openpose_ros::OpenPoseROSIO openPoseROSIO(FLAGS_camera_topic, FLAGS_openpose_output_topic);
    bool userWantsToExit = false;
    while (!userWantsToExit && ros::ok())
    {
        // Push frame
        auto datumToProcess = openPoseROSIO.createDatum();
        if (datumToProcess != nullptr)
        {
            auto successfullyEmplaced = opWrapper.waitAndEmplace(datumToProcess);
            // Pop frame
            std::shared_ptr<std::vector<op::Datum>> datumProcessed;
            if (successfullyEmplaced && opWrapper.waitAndPop(datumProcessed))
            {
                userWantsToExit = openPoseROSIO.display(datumProcessed);
                openPoseROSIO.printKeypoints(datumProcessed);
                openPoseROSIO.publish(datumProcessed);
            }
            else
                op::log("Processed datum could not be emplaced.", op::Priority::High,
                        __LINE__, __FUNCTION__, __FILE__);
        }

        ros::spinOnce();
    }

    op::log("Stopping thread(s)", op::Priority::High);
    opWrapper.stop();

    // Measuring total time
    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count()
                            * 1e-9;
    const auto message = "Real-time pose estimation demo successfully finished. Total time: "
                       + std::to_string(totalTimeSec) + " seconds.";
    op::log(message, op::Priority::High);

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
