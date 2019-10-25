#include <openpose_ros_io.h>

using namespace openpose_ros;

OpenPoseROSIO::OpenPoseROSIO(OpenPose &openPose): nh_("/openpose_ros_node"), it_(nh_)
{
    // Subscribe to input video feed and publish human lists as output
    std::string image_topic;
    std::string output_topic;
    std::string input_image_transport_type;

    nh_.param("image_topic", image_topic, std::string("/camera/image_raw"));
    nh_.param("input_image_transport_type", input_image_transport_type, std::string("raw"));
    nh_.param("output_topic", output_topic, std::string("/openpose_ros/human_list"));
    nh_.param("display_output", display_output_flag_, true);
    nh_.param("print_keypoints", print_keypoints_flag_, false);
    nh_.param("save_original_video", save_original_video_flag_, false);
    nh_.param("save_openpose_video", save_openpose_video_flag_, false);
    nh_.param("original_video_file_name", original_video_file_name_, std::string(""));
    nh_.param("openpose_video_file_name", openpose_video_file_name_, std::string(""));
    nh_.param("video_fps", video_fps_, 10);

    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseROSIO::processImage, this, image_transport::TransportHints(input_image_transport_type));
    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
    cv_img_ptr_ = nullptr;
    openpose_ = &openPose;

    if(save_original_video_flag_)
    {
        if(original_video_file_name_.empty())
        {
            std::cout << "No original video filename was provided. Not saving original video." << std::endl; 
            save_original_video_flag_ = false;
        }
        else
        {
            original_video_writer_initialized_ = false;
        }
    }
    if(save_openpose_video_flag_)
    {
        if(openpose_video_file_name_.empty())
        {
            std::cout << "No openpose video filename was provided. Not saving openpose video." << std::endl; 
            save_openpose_video_flag_ = false;
        }
        else
        {
            openpose_video_writer_initialized_ = false;
        }
    }
}

void OpenPoseROSIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    convertImage(msg);
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumToProcess = createDatum();

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);
    
    // Pop frame
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;
    if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
    {
        if(display_output_flag_)
        {
            display(datumProcessed);
        }
        if(print_keypoints_flag_)
        {
            printKeypoints(datumProcessed);
        }
        if(save_original_video_flag_)
        {
            saveOriginalVideo(datumToProcess);
        }
        if(save_openpose_video_flag_)
        {
            saveOpenPoseVideo(datumProcessed);
        }
        publish(datumProcessed);
    }
    else
    {
        op::opLog("Processed datum could not be emplaced.", op::Priority::High,
                __LINE__, __FUNCTION__, __FILE__);
    }
}

void OpenPoseROSIO::convertImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> OpenPoseROSIO::createDatum()
{
    // Close program when empty frame
    if (cv_img_ptr_ == nullptr)
    {
        return nullptr;
    }
    else // if (cv_img_ptr_ == nullptr)
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<op::Datum>();

        // Fill datum
        datumPtr->cvInputData = OP_CV2OPCONSTMAT(cv_img_ptr_->image);

        return datumsPtr;
    }
}

bool OpenPoseROSIO::display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::imshow("User worker GUI", OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData));
        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        key = (char)cv::waitKey(1);
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

bool OpenPoseROSIO::saveOriginalVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::Mat current_image = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvInputData);
        if(!current_image.empty())
        {
            if(!original_video_writer_initialized_)
            {
                original_video_writer_ = cv::VideoWriter(original_video_file_name_, CV_FOURCC('M','J','P','G'), video_fps_, current_image.size());
                original_video_writer_initialized_ = true;
            }   
            original_video_writer_.write(current_image);
        }
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

bool OpenPoseROSIO::saveOpenPoseVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::Mat current_image = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
        if(!current_image.empty())
        {
            if(!openpose_video_writer_initialized_)
            {
                openpose_video_writer_ = cv::VideoWriter(openpose_video_file_name_, CV_FOURCC('M','J','P','G'), video_fps_, current_image.size());
                openpose_video_writer_initialized_ = true;
            }   
            openpose_video_writer_.write(current_image);
        }
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

cv_bridge::CvImagePtr& OpenPoseROSIO::getCvImagePtr()
{
    return cv_img_ptr_;
}

void OpenPoseROSIO::printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    // Example: How to use the pose keypoints
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        op::opLog("\nKeypoints:");
        // Accesing each element of the keypoints
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        op::opLog("Person pose keypoints:");
        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            op::opLog("Person " + std::to_string(person) + " (x, y, score):");
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                std::string valueToPrint;
                for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                {
                    valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                }
                op::opLog(valueToPrint);
            }
        }
        op::opLog(" ");
        // Alternative: just getting std::string equivalent
        if(FLAGS_face)
        {
            op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
        }
        if(FLAGS_hand)
        {
            op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
            op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
        }
        // Heatmaps
        const auto& poseHeatMaps = datumsPtr->at(0)->poseHeatMaps;
        if (!poseHeatMaps.empty())
        {
            op::opLog("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
                    + std::to_string(poseHeatMaps.getSize(1)) + ", "
                    + std::to_string(poseHeatMaps.getSize(2)) + "]");
            const auto& faceHeatMaps = datumsPtr->at(0)->faceHeatMaps;
            op::opLog("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
                    + std::to_string(faceHeatMaps.getSize(1)) + ", "
                    + std::to_string(faceHeatMaps.getSize(2)) + ", "
                    + std::to_string(faceHeatMaps.getSize(3)) + "]");
            const auto& handHeatMaps = datumsPtr->at(0)->handHeatMaps;
            op::opLog("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(3)) + "]");
            op::opLog("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(3)) + "]");
        }
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
}

void OpenPoseROSIO::publish(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        const auto& faceKeypoints = datumsPtr->at(0)->faceKeypoints;
        const auto& leftHandKeypoints = datumsPtr->at(0)->handKeypoints[0];
        const auto& rightHandKeypoints = datumsPtr->at(0)->handKeypoints[1];
        std::vector<op::Rectangle<float>>& face_rectangles = datumsPtr->at(0)->faceRectangles;

        openpose_ros_msgs::OpenPoseHumanList human_list_msg;
        human_list_msg.header.stamp = ros::Time::now();
        human_list_msg.image_header = image_header_;
        human_list_msg.num_humans = poseKeypoints.getSize(0);
        
        std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(poseKeypoints.getSize(0));

        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            openpose_ros_msgs::OpenPoseHuman human;

            double body_min_x = -1;
            double body_max_x = -1;
            double body_min_y = -1;
            double body_max_y = -1;

            int num_body_key_points_with_non_zero_prob = 0;
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                openpose_ros_msgs::PointWithProb body_point_with_prob;
                body_point_with_prob.x = poseKeypoints[{person, bodyPart, 0}];
                body_point_with_prob.y = poseKeypoints[{person, bodyPart, 1}];
                body_point_with_prob.prob = poseKeypoints[{person, bodyPart, 2}];
                if(body_point_with_prob.prob > 0)
                {
                    num_body_key_points_with_non_zero_prob++;

                    if(body_min_x == -1 || body_point_with_prob.x < body_min_x)
                    {
                        body_min_x = body_point_with_prob.x;
                    }
                    if(body_point_with_prob.x > body_max_x)
                    {
                        body_max_x = body_point_with_prob.x;
                    }

                    if(body_min_y == -1 || body_point_with_prob.y < body_min_y)
                    {
                        body_min_y = body_point_with_prob.y;
                    }
                    if(body_point_with_prob.y > body_max_y)
                    {
                        body_max_y = body_point_with_prob.y;
                    }
                }
                human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
            }
            human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;
            human.body_bounding_box.x = body_min_x;
            human.body_bounding_box.y = body_min_y;
            human.body_bounding_box.width = body_max_x - body_min_x;
            human.body_bounding_box.height = body_max_y - body_min_y;

            if(FLAGS_face)
            {
                int num_face_key_points_with_non_zero_prob = 0;

                for (auto facePart = 0 ; facePart < faceKeypoints.getSize(1) ; facePart++)
                {
                    openpose_ros_msgs::PointWithProb face_point_with_prob;
                    face_point_with_prob.x = faceKeypoints[{person, facePart, 0}];
                    face_point_with_prob.y = faceKeypoints[{person, facePart, 1}];
                    face_point_with_prob.prob = faceKeypoints[{person, facePart, 2}];
                    if(face_point_with_prob.prob > 0)
                    {
                        num_face_key_points_with_non_zero_prob++;
                    }
                    human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
                }  
                human.num_face_key_points_with_non_zero_prob = num_face_key_points_with_non_zero_prob;

                openpose_ros_msgs::BoundingBox face_bounding_box;
                face_bounding_box.x = face_rectangles.at(person).x;
                face_bounding_box.y = face_rectangles.at(person).y;
                face_bounding_box.width = face_rectangles.at(person).width;
                face_bounding_box.height = face_rectangles.at(person).height;
                human.face_bounding_box = face_bounding_box;
            }
            
            if(FLAGS_hand)
            {

                int num_right_hand_key_points_with_non_zero_prob = 0;
                int num_left_hand_key_points_with_non_zero_prob = 0;

                for (auto handPart = 0 ; handPart < rightHandKeypoints.getSize(1) ; handPart++)
                {
                    openpose_ros_msgs::PointWithProb right_hand_point_with_prob;
                    openpose_ros_msgs::PointWithProb left_hand_point_with_prob;
                    right_hand_point_with_prob.x = rightHandKeypoints[{person, handPart, 0}];
                    right_hand_point_with_prob.y = rightHandKeypoints[{person, handPart, 1}];
                    right_hand_point_with_prob.prob = rightHandKeypoints[{person, handPart, 2}];
                    if(right_hand_point_with_prob.prob > 0)
                    {
                        num_right_hand_key_points_with_non_zero_prob++;
                    }
                    left_hand_point_with_prob.x = leftHandKeypoints[{person, handPart, 0}];
                    left_hand_point_with_prob.y = leftHandKeypoints[{person, handPart, 1}];
                    left_hand_point_with_prob.prob = leftHandKeypoints[{person, handPart, 2}];
                    if(left_hand_point_with_prob.prob > 0)
                    {
                        num_left_hand_key_points_with_non_zero_prob++;
                    }
                    human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;
                    human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;
                }
                human.num_right_hand_key_points_with_non_zero_prob = num_right_hand_key_points_with_non_zero_prob;
                human.num_left_hand_key_points_with_non_zero_prob = num_left_hand_key_points_with_non_zero_prob;
            }

            human_list.at(person) = human;
        }

        human_list_msg.human_list = human_list;

        openpose_human_list_pub_.publish(human_list_msg);

    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
}

void OpenPoseROSIO::stop()
{
    if(save_original_video_flag_)
    {
        original_video_writer_.release();
    }
    if(save_openpose_video_flag_)
    {
        openpose_video_writer_.release();
    }
}