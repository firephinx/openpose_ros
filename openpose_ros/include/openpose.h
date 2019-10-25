#ifndef _OPENPOSE
#define _OPENPOSE

#include <openpose_flags.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros {

    class OpenPose
    {
        private:
            const op::Point<int> outputSize;
            const op::Point<int> netInputSize;
            const op::Point<int> faceNetInputSize;
            const op::Point<int> handNetInputSize;
            const op::PoseMode poseMode;
            const op::PoseModel poseModel;
            const op::ScaleMode keypointScaleMode;
            const std::vector<op::HeatMapType> heatMapTypes;
            const op::ScaleMode heatMapScaleMode;
            const op::Detector faceDetector;
            const op::Detector handDetector;
            const bool multipleView;
            const bool enableGoogleLogging;

            op::Wrapper opWrapper;

            const op::WrapperStructPose wrapperStructPose;
            const op::WrapperStructFace wrapperStructFace;
            const op::WrapperStructHand wrapperStructHand;
            const op::WrapperStructExtra wrapperStructExtra;
            const op::WrapperStructOutput wrapperStructOutput;
            // const op::WrapperStructGui wrapperStructGui;

        public:
            OpenPose();

            ~OpenPose(){}

            void start();

            bool waitAndEmplace(std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumToProcess);

            bool waitAndPop(std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumProcessed);

            void stop();
    };
}

#endif