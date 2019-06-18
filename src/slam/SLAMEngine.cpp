#include <numeric>
#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
    mFrameOffset = 0;
    mPipelineLength = 0;
    mNextFrameId = 0;
}

void SLAMEngine::startup(SLAMPipeline& pipeline)
{
    // check input.

    if( pipeline.modules.size() != pipeline.lags.size() )
    {
        throw std::runtime_error("internal error");
    }

    if( std::accumulate(pipeline.thread_partition.begin(), pipeline.thread_partition.end(), 0) != pipeline.modules.size() )
    {
        throw std::runtime_error("internal error");
    }

    if( std::find(pipeline.thread_partition.begin(), pipeline.thread_partition.end(), 0) != pipeline.thread_partition.end() )
    {
        throw std::runtime_error("internal error");
    }

    // allocate frames.

    {
        mPipelineLength = 1;

        for(size_t lag : pipeline.lags)
        {
            mPipelineLength = std::max(mPipelineLength, lag+1);
        }

        mNextFrameId = 0;
        mFrameOffset = 0;
        mFrames.resize(mPipelineLength);

        for(SLAMFrame& f : mFrames)
        {
            f.header.ready = false;
        }
    }

    // allocate modules.

    {
        mModules.resize(pipeline.modules.size());

        for(size_t i=0; i<mModules.size(); i++)
        {
            mModules[i].module = pipeline.modules[i];
            mModules[i].lag = pipeline.lags[i];
            mModules[i].current_frame = nullptr;
        }
    }

    // allocate threads.

    mThreads.resize(pipeline.thread_partition.size());

    {
        size_t k = 0;

        for(size_t i=0; i<mThreads.size(); i++)
        {
            SLAMModuleWrapper* first_module = &mModules[k];

            for(size_t j=0; j+1<pipeline.thread_partition[i]; j++)
            {
                mModules[k].next_in_thread = &mModules[k+1];
                k++;
            }

            mModules[k].next_in_thread = nullptr;
            k++;

            mThreads[i] = std::make_shared<SLAMThread>();
            mThreads[i]->startup(first_module);
        }
    }
}

void SLAMEngine::feed(SLAMEngineInput& input, SLAMEngineOutput& output)
{
    // push new frame and set expected frame to each module according to lag.

    mFrameOffset = (mFrameOffset + mPipelineLength - 1) % mPipelineLength;

    SLAMFrame& curr_frame = mFrames[mFrameOffset];

    curr_frame.header.ready = true;
    curr_frame.header.num_views = 1;
    curr_frame.header.views[0] = input.image;
    curr_frame.header.video_frame_id = input.id;
    curr_frame.header.slam_frame_id = mNextFrameId++;
    curr_frame.header.video_timestamp = input.timestamp;

    for(SLAMModuleWrapper& m : mModules)
    {
        const size_t index = (mFrameOffset + m.lag) % mPipelineLength;
        m.current_frame = &mFrames[index];
    }

    // compute.

    for(SLAMThreadPtr& t : mThreads)
    {
        t->trigger();
    }

    for(SLAMThreadPtr& t : mThreads)
    {
        t->wait();
    }

    // TODO: retrieve results.
}

void SLAMEngine::shutdown()
{
    for(SLAMThreadPtr& t : mThreads)
    {
        t->shutdown();
    }

    mPipelineLength = 0;
    mFrameOffset = 0;
    mNextFrameId = 0;
    mFrames.clear();
    mModules.clear();
    mThreads.clear();
}

