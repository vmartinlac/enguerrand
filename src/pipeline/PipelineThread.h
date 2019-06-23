#pragma once

#include <thread>
#include "Semaphore.h"
#include "PipelineModuleWrapper.h"

class PipelineThread
{
public:

    PipelineThread();

    void startup(PipelineModuleWrapper* module_list);

    void trigger();

    void wait();

    void shutdown();

protected:

    void threadProcedure();

protected:

    std::thread mThread;

    Semaphore mSemaphoreStart;
    Semaphore mSemaphoreFinished;

    bool mInterruptionRequested;

    PipelineModuleWrapper* mModuleList;
};

