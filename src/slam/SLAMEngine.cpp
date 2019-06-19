#include <numeric>
#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
    mCycleOffset = 0;
    mPipelineLength = 0;
}

void SLAMEngine::exec(SLAMPipeline& pipeline)
{
    // check input.

    {
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

        auto pred = [] (size_t i, const std::unique_ptr<SLAMModule>& module) -> size_t
        {
            return i + module->getNumPorts();
        };

        if( pipeline.connections.size() != std::accumulate(pipeline.modules.begin(), pipeline.modules.end(), 0, pred) )
        {
            throw std::runtime_error("internal error");
        }
    }

    // allocate ports.

    {
        mPipelineLength = 1;

        for(size_t lag : pipeline.lags)
        {
            mPipelineLength = std::max(mPipelineLength, lag+1);
        }

        mCycleOffset = 0;

        for(auto& functor : pipeline.meta_ports)
        {
            for(size_t i=0; i<mPipelineLength; i++)
            {
                mPorts.emplace_back(functor());
            }
        }
    }

    // allocate modules.

    {
        mModules.resize(pipeline.modules.size());

        for(size_t i=0; i<mModules.size(); i++)
        {
            mModules[i].module = pipeline.modules[i].get();
            mModules[i].lag = pipeline.lags[i];
            mModules[i].ports = nullptr;
            mModules[i].next_in_thread = nullptr;
        }
    }

    // allocate port table.

    {
        mPortTable.assign(pipeline.connections.size(), nullptr);

        size_t s = 0;

        for(SLAMModuleWrapper& m : mModules)
        {
            m.ports = &mPortTable[s];
            s += m.module->getNumPorts();
        }
    }

    // allocate threads.

    {
        mThreads.resize(pipeline.thread_partition.size());

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

            mThreads[i].reset(new SLAMThread());
            mThreads[i]->startup(first_module);
        }
    }

    // initialize modules.

    {
        for(SLAMModuleWrapper& m : mModules)
        {
            m.enabled = false;
            m.module->initialize();
        }
    }

    // run pipeline.

    {
        bool go_on = true;
        size_t num_frames_in_pipeline = 0;

        mCycleOffset = 0;

        while(go_on)
        {
            // find which modules are ready to compute and update ports.

            mCycleOffset = (mCycleOffset + mPipelineLength - 1) % mPipelineLength;

            size_t s = 0;

            for(SLAMModuleWrapper& m : mModules)
            {
                m.enabled = (num_frames_in_pipeline >= m.lag);

                for(size_t i=0; i<m.module->getNumPorts(); i++)
                {
                    const size_t delta = (mCycleOffset + m.lag) % mPipelineLength;
                    mPortTable[s] = mPorts[mPipelineLength * pipeline.connections[s] + delta].get();
                    s++;
                }
            }

            num_frames_in_pipeline = std::min<size_t>(num_frames_in_pipeline+1, mPipelineLength);

            // compute.

            for(std::unique_ptr<SLAMThread>& t : mThreads)
            {
                t->trigger();
            }

            for(std::unique_ptr<SLAMThread>& t : mThreads)
            {
                t->wait();
            }
        }
    }

    // finalize modules.

    {
        for(SLAMModuleWrapper& m : mModules)
        {
            m.module->finalize();
        }
    }

    // clear pipeline.

    {
        for(std::unique_ptr<SLAMThread>& t : mThreads)
        {
            t->shutdown();
        }

        mPipelineLength = 0;
        mCycleOffset = 0;
        mPortTable.clear();
        mPorts.clear();
        mModules.clear();
        mThreads.clear();
    }
}

