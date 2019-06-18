#include <numeric>
#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
    mFrameOffset = 0;
    mPipelineLength = 0;
}

void SLAMEngine::init(SLAMPipelinePtr pipeline)
{
    mPipeline = std::move(pipeline);

    mPipelineLength = pipeline->computeLength();

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

    for(int i=0; i<mPipeline->num_cpu_threads; i++)
    {
        ;
    }

    for(int i=1; i<mPipeline->num_cpu_threads; i++)
    {
        mThreads[i-1].feed(&mWorkLoads[i]);
    }

    mWorkLoads.front().execute();

    for(int i=1; i<mPipeline->num_cpu_threads; i++)
    {
        mThreads[i-1].wait();
    }
}

void SLAMEngine::halt()
{
    for(SLAMThread& t : mThreads)
    {
        t.halt();
    }

    mPipeline.reset();
    mPipelineLength = 0;
    mFrameOffset = 0;
    mWorkLoads.clear();
    mThreads.clear();
    mFrames.clear();
}

