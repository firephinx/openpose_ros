#ifndef _GFLAGS_OPTIONS
#define _GFLAGS_OPTIONS

// GFlags: DEFINE_bool, _int32, _int64, _uint64, _double, _string
#include <gflags/gflags.h>
// Allow Google Flags in Ubuntu 14
#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif

DECLARE_int32(logging_level);
DECLARE_bool(disable_multi_thread);
DECLARE_int32(profile_speed);
// OpenPose
DECLARE_string(model_folder);
DECLARE_string(prototxt_path);
DECLARE_string(caffemodel_path);
DECLARE_string(output_resolution);
DECLARE_int32(num_gpu);
DECLARE_int32(num_gpu_start);
DECLARE_int32(keypoint_scale);
DECLARE_int32(number_people_max);
DECLARE_bool(maximize_positives);
DECLARE_double(fps_max);
// OpenPose Body Pose
DECLARE_int32(body);
DECLARE_string(model_pose);
DECLARE_string(net_resolution);
DECLARE_int32(scale_number);
DECLARE_double(scale_gap);
// OpenPose Body Pose Heatmaps and Part Candidates
DECLARE_bool(heatmaps_add_parts);
DECLARE_bool(heatmaps_add_bkg);
DECLARE_bool(heatmaps_add_PAFs);
DECLARE_int32(heatmaps_scale);
DECLARE_bool(part_candidates);
DECLARE_double(upsampling_ratio);
// OpenPose Face
DECLARE_bool(face);
DECLARE_int32(face_detector);
DECLARE_string(face_net_resolution);
// OpenPose Hand
DECLARE_bool(hand);
DECLARE_int32(hand_detector);
DECLARE_string(hand_net_resolution);
DECLARE_int32(hand_scale_number);
DECLARE_double(hand_scale_range);
// OpenPose 3-D Reconstruction
DECLARE_bool(3d);
DECLARE_int32(3d_min_views);
DECLARE_int32(3d_views);
// Extra algorithms
DECLARE_bool(identification);
DECLARE_int32(tracking);
DECLARE_int32(ik_threads);
// OpenPose Rendering
DECLARE_int32(part_to_show);
DECLARE_bool(disable_blending);
// OpenPose Rendering Pose
DECLARE_double(render_threshold);
DECLARE_int32(render_pose);
DECLARE_double(alpha_pose);
DECLARE_double(alpha_heatmap);
// OpenPose Rendering Face
DECLARE_double(face_render_threshold);
DECLARE_int32(face_render);
DECLARE_double(face_alpha_pose);
DECLARE_double(face_alpha_heatmap);
// OpenPose Rendering Hand
DECLARE_double(hand_render_threshold);
DECLARE_int32(hand_render);
DECLARE_double(hand_alpha_pose);
DECLARE_double(hand_alpha_heatmap);
// Display
DECLARE_bool(fullscreen);
DECLARE_bool(no_gui_verbose);
DECLARE_int32(display);
// Command Line Interface Verbose
DECLARE_double(cli_verbose);
// Result Saving
DECLARE_string(write_images);
DECLARE_string(write_images_format);
DECLARE_string(write_video);
DECLARE_double(write_video_fps);
DECLARE_bool(write_video_with_audio);
DECLARE_string(write_video_3d);
DECLARE_string(write_video_adam);
DECLARE_string(write_json);
DECLARE_string(write_coco_json);
DECLARE_int32(write_coco_json_variants);
DECLARE_int32(write_coco_json_variant);
DECLARE_string(write_heatmaps);
DECLARE_string(write_heatmaps_format);
DECLARE_string(write_keypoint);
DECLARE_string(write_keypoint_format);
// Result Saving - Extra Algorithms
DECLARE_string(write_bvh);
// UDP Communication
DECLARE_string(udp_host);
DECLARE_string(udp_port);

#endif