#pragma once

#include <sophus/se3.hpp>
#include "SLAMModule.h"
#include "SLAMThread.h"
#include "SLAMPipeline.h"

struct SLAMEngineInput
{
    size_t id;
    double timestamp;
    cv::Mat3b image;
};

struct SLAMEngineOutput
{
    bool success;
    size_t id;
    double timestamp;
    bool aligned_wrt_previous;
    Sophus::SE3d camera_to_world;
    cv::Mat3b debug_image;
};

class SLAMEngine
{
public:

    SLAMEngine();

    void startup(SLAMPipeline& pipeline);

    void feed(SLAMEngineInput& input, SLAMEngineOutput& output);

    void shutdown();

protected:

    std::vector<SLAMModuleWrapper> mModules;
    std::vector<SLAMThreadPtr> mThreads;
    std::vector<SLAMFrame> mFrames;
    size_t mPipelineLength;
    size_t mFrameOffset;
    size_t mNextFrameId;
};

