#ifndef _OPENPOSE
#define _OPENPOSE

#include <gflags_options.h>

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
            const op::PoseModel poseModel;
            const std::string writeJson;
            const op::ScaleMode keypointScale;
            const std::vector<op::HeatMapType> heatMapTypes;
            const op::ScaleMode heatMapScale;
            const bool enableGoogleLogging;

            op::Wrapper<std::vector<op::Datum>> opWrapper;

            const op::WrapperStructPose wrapperStructPose;
            const op::WrapperStructFace wrapperStructFace;
            const op::WrapperStructHand wrapperStructHand;

            const op::DisplayMode displayMode;
            const bool guiVerbose;
            const bool fullScreen;

            const op::WrapperStructOutput wrapperStructOutput;

        public:
            OpenPose();

            ~OpenPose(){}

            void start();

            bool waitAndEmplace(std::shared_ptr<std::vector<op::Datum>> &datumToProcess);

            bool waitAndPop(std::shared_ptr<std::vector<op::Datum>> &datumProcessed);

            void stop();
    };
}

#endif