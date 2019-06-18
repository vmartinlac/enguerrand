#include <numeric>
#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
    mFrameOffset = 0;
    mPipelineLength = 0;
}

void SLAMEngine::start(SLAMPipelinePtr pipeline)
{
    mPipeline = std::move(pipeline);

    mPipelineLength = pipeline->computeLength();

    mNextFrameId = 0;

    mFrameOffset = 0;

    mFrames.resize(mPipelineLength);

    for(SLAMFrame& f : mFrames)
    {
        f.header.ready = false;
    }
}


void SLAMEngine::feed(SLAMEngineInput& input, SLAMEngineOutput& output)
{
    mFrameOffset = (mFrameOffset + mPipelineLength - 1) % mPipelineLength;

    SLAMFrame& curr_frame = mFrames[mFrameOffset];

    curr_frame.header.ready = true;
    curr_frame.header.num_views = 1;
    curr_frame.header.views[0] = input.image;
    curr_frame.header.video_frame_id = input.id;
    curr_frame.header.slam_frame_id = mNextFrameId++;
    curr_frame.header.video_timestamp = input.timestamp;

    for(int i=0; i<mPipeline->num_threads; i++)
    {
        ;
    }

    for(int i=1; i<mPipeline->num_threads; i++)
    {
        mThreads[i-1].feed(&mWorkLoads[i]);
    }

    mWorkLoads.front().execute();

    for(int i=1; i<mPipeline->num_threads; i++)
    {
        mThreads[i-1].wait();
    }
}

void SLAMEngine::stop()
{
    for(SLAMThread& t : mThreads)
    {
        t.halt();
    }

    mPipeline.reset();
    mPipelineLength = 0;
    mFrameOffset = 0;
    mNextFrameId = 0;
    mWorkLoads.clear();
    mThreads.clear();
    mFrames.clear();
}

