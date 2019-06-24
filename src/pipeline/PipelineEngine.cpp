#include <chrono>
#include <numeric>
#include "PipelineEngine.h"

PipelineEngine::PipelineEngine()
{
    mCycleOffset = 0;
    mPipelineLength = 0;
}

void PipelineEngine::exec(PipelineDescription& pipeline)
{
    bool ok = true;

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

        auto pred = [] (size_t i, const std::unique_ptr<PipelineModule>& module) -> size_t
        {
            return i + module->getNumPorts();
        };

        if( pipeline.connections.size() != std::accumulate(pipeline.modules.begin(), pipeline.modules.end(), 0, pred) )
        {
            throw std::runtime_error("internal error");
        }
    }

    std::cout << "Initializing pipeline..." << std::endl;

    // initialize modules.

    if(ok)
    {
        for(std::unique_ptr<PipelineModule>& m : pipeline.modules)
        {
            ok = ok && m->initialize();
        }
    }

    std::cout << "Num modules: " << pipeline.modules.size() << std::endl;

    // allocate ports.

    if(ok)
    {
        mPipelineLength = 1;

        for(size_t lag : pipeline.lags)
        {
            mPipelineLength = std::max(mPipelineLength, lag+1);
        }

        mCycleOffset = 0;

        for( std::unique_ptr<PipelinePortFactory>& factory : pipeline.ports )
        {
            for(size_t i=0; i<mPipelineLength; i++)
            {
                PipelinePort* port = factory->create();
                ok = ok && bool(port);
                mPorts.emplace_back(port);
            }
        }
    }

    std::cout << "Pipeline length: " << mPipelineLength << std::endl;
    std::cout << "Num ports: " << pipeline.ports.size() << " / " << mPorts.size() << std::endl;

    // allocate modules.

    if(ok)
    {
        mModules.resize(pipeline.modules.size());

        for(size_t i=0; i<mModules.size(); i++)
        {
            mModules[i].enabled = false;
            mModules[i].module = pipeline.modules[i].get();
            mModules[i].lag = pipeline.lags[i];
            mModules[i].ports = nullptr;
            mModules[i].next_in_thread = nullptr;
        }
    }

    // allocate port table.

    if(ok)
    {
        mPortTable.assign(pipeline.connections.size(), nullptr);

        size_t s = 0;

        for(PipelineModuleWrapper& m : mModules)
        {
            m.ports = &mPortTable[s];
            s += m.module->getNumPorts();
        }
    }

    // allocate threads.

    if(ok)
    {
        mThreads.resize(pipeline.thread_partition.size());

        size_t k = 0;

        for(size_t i=0; i<mThreads.size(); i++)
        {
            PipelineModuleWrapper* first_module = &mModules[k];

            for(size_t j=0; j+1<pipeline.thread_partition[i]; j++)
            {
                mModules[k].next_in_thread = &mModules[k+1];
                k++;
            }

            mModules[k].next_in_thread = nullptr;
            k++;

            mThreads[i].reset(new PipelineThread());
            mThreads[i]->startup(first_module);
        }
    }

    std::cout << "Num threads: " << mThreads.size() << std::endl;

    // run pipeline.

    size_t cycle_id = 0;

    if(ok)
    {
        bool go_on = true;
        size_t num_frames_in_pipeline = 0;

        mCycleOffset = 0;

        while(go_on)
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> t0 = std::chrono::high_resolution_clock::now();

            std::cout << "CYCLE " << cycle_id << std::endl;

            // find which modules are ready to compute and update ports.

            mCycleOffset = (mCycleOffset + mPipelineLength - 1) % mPipelineLength;

            size_t s = 0;

            for(PipelineModuleWrapper& m : mModules)
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

            // reset incoming ports.

            for(size_t i=0; i<pipeline.ports.size(); i++)
            {
                mPorts[mPipelineLength*i+mCycleOffset]->reset();
            }

            // compute.

            for(std::unique_ptr<PipelineThread>& t : mThreads)
            {
                t->trigger();
            }

            for(std::unique_ptr<PipelineThread>& t : mThreads)
            {
                t->wait();
            }

            // print computation time.

            std::chrono::time_point<std::chrono::high_resolution_clock> t1 = std::chrono::high_resolution_clock::now();

            std::cout << "   Modules runtime:" << std::endl;
            for(PipelineModuleWrapper& m : mModules)
            {
                std::cout << "      " << m.module->getName() << " (" PIPELINE_TIME_UNIT_NAME "): " << m.elapsed.count() << std::endl;
            }
            std::cout << "   Cycle runtime (" PIPELINE_TIME_UNIT_NAME "): " << std::chrono::duration_cast<PipelineTimeUnit>(t1-t0).count() << std::endl;

            cycle_id++;
        }
    }

    // finalize modules.

    if(ok)
    {
        for(PipelineModuleWrapper& m : mModules)
        {
            m.module->finalize();
        }
    }

    // clear pipeline.

    if(ok)
    {
        for(std::unique_ptr<PipelineThread>& t : mThreads)
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

