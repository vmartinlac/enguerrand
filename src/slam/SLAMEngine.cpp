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

    mPipelineLength = computePipelineLength(pipeline);

    mFrameOffset = 0;
    mFrames.resize(mPipelineLength);

    mThreads.resize(mPipeline->cpu_modules.size());

    for(SLAMThreadPtr& t : mThreads)
    {
        t.reset(new SLAMThread());
        t->init();
    }
}


void SLAMEngine::feed(SLAMFramePtr frame, SLAMResult& result)
{
    mFrameOffset = (mFrameOffset + mPipelineLength - 1) % mPipelineLength;
    mFrames[mFrameOffset] = frame;
}

void SLAMEngine::halt()
{
    for(SLAMThreadPtr t : mThreads)
    {
        t->halt();
    }

    mThreads.clear();
    mFrames.clear();
    mPipeline.reset();
    mPipelineLength = 0;
}

