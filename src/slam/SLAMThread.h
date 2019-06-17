#pragma once

#include <thread>
#include "Semaphore.h"
#include "SLAMModule.h"

struct SLAMWorkItem
{
    SLAMModulePtr module;
    SLAMFrame* frame;
};

struct SLAMWork
{
    std::vector<SLAMWorkItem> items;
};

class SLAMThread
{
public:

    SLAMThread();

    void init();

    void feed(SLAMWorkPtr work);

    void wait();

    void halt();

protected:

    void threadProcedure();

protected:

    std::thread mThread;

    bool mInterruptionRequested;
    SLAMWorkPtr mCurrentWork;

    Semaphore mSemaphoreStart;
    Semaphore mSemaphoreFinished;
};

