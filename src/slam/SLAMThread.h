#pragma once

#include <thread>
#include <memory>
#include "Semaphore.h"
#include "SLAMModuleWrapper.h"

class SLAMThread
{
public:

    SLAMThread();

    void startup(SLAMModuleWrapper* module_list);

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

    SLAMModuleWrapper* mModuleList;
};

using SLAMThreadPtr = std::shared_ptr<SLAMThread>;

