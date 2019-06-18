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

    void init(SLAMPipelinePtr pipeline);

    void feed(SLAMEngineInput& input, SLAMEngineOutput& output);

    void halt();

protected:

    SLAMPipelinePtr mPipeline;
    size_t mPipelineLength;
    size_t mFrameOffset;
    std::vector<SLAMWorkLoad> mWorkLoads;
    std::vector<SLAMThread> mThreads;
    std::vector<SLAMFrame> mFrames;
};

