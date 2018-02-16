#include <openpose.h>

using namespace openpose_ros;

OpenPose::OpenPose() : outputSize(op::flagsToPoint(FLAGS_output_resolution, "-1x-1")),
                       netInputSize(op::flagsToPoint(FLAGS_net_resolution, "-1x368")),
                       faceNetInputSize(op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)")),
                       handNetInputSize(op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)")),
                       poseModel(op::flagsToPoseModel(FLAGS_model_pose)),
                       writeJson((!FLAGS_write_json.empty() ? FLAGS_write_json : FLAGS_write_keypoint_json)),
                       keypointScale(op::flagsToScaleMode(FLAGS_keypoint_scale)),
                       heatMapTypes(op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                        FLAGS_heatmaps_add_PAFs)),
                       heatMapScale(op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale)),
                       enableGoogleLogging(true),
                       opWrapper(op::ThreadManagerMode::Asynchronous),
                       wrapperStructPose(!FLAGS_body_disable, netInputSize, outputSize, keypointScale,
                                         FLAGS_num_gpu, FLAGS_num_gpu_start, FLAGS_scale_number,
                                         (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose),
                                         poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose,
                                         (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, FLAGS_model_folder,
                                         heatMapTypes, heatMapScale, FLAGS_part_candidates,
                                         (float)FLAGS_render_threshold, FLAGS_number_people_max,
                                         enableGoogleLogging, FLAGS_3d),
                       wrapperStructFace(FLAGS_face, faceNetInputSize,
                                         op::flagsToRenderMode(FLAGS_face_render, FLAGS_render_pose),
                                         (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap,
                                         (float)FLAGS_face_render_threshold),
                       wrapperStructHand(FLAGS_hand, handNetInputSize, FLAGS_hand_scale_number,
                                         (float)FLAGS_hand_scale_range, FLAGS_hand_tracking,
                                         op::flagsToRenderMode(FLAGS_hand_render, FLAGS_render_pose),
                                         (float)FLAGS_hand_alpha_pose, (float)FLAGS_hand_alpha_heatmap,
                                         (float)FLAGS_hand_render_threshold),
                       displayMode(op::DisplayMode::NoDisplay),
                       guiVerbose(false),
                       fullScreen(false),
                       wrapperStructOutput(displayMode, guiVerbose, fullScreen, FLAGS_write_keypoint,
                                           op::stringToDataFormat(FLAGS_write_keypoint_format),
                                           writeJson, FLAGS_write_coco_json,
                                           FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video,
                                           FLAGS_write_heatmaps, FLAGS_write_heatmaps_format)


{
    if (!FLAGS_write_keypoint.empty() || !FLAGS_write_keypoint_json.empty())
    {
        op::log("Flags `write_keypoint` and `write_keypoint_json` are deprecated and will eventually be removed."
                " Please, use `write_json` instead.", op::Priority::Max);
    }

    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    // Configure wrapper
    op::log("Configuring OpenPose wrapper.", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    opWrapper.configure(wrapperStructPose, wrapperStructFace, wrapperStructHand, op::WrapperStructInput{},
                        wrapperStructOutput);

    // Set to single-thread running (to debug and/or reduce latency)
    if (FLAGS_disable_multi_thread)
    {
       opWrapper.disableMultiThreading();
    }

}

void OpenPose::start()
{
    opWrapper.start();
}

bool OpenPose::waitAndEmplace(std::shared_ptr<std::vector<op::Datum>> &datumToProcess)
{
    opWrapper.waitAndEmplace(datumToProcess);
}

bool OpenPose::waitAndPop(std::shared_ptr<std::vector<op::Datum>> &datumProcessed)
{
    opWrapper.waitAndPop(datumProcessed);
}

void OpenPose::stop()
{
    opWrapper.stop();
}