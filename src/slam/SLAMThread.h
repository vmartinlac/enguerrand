#pragma once

#include <thread>
#include "Semaphore.h"
#include "SLAMModule.h"
#include "SLAMWorkLoad.h"

class SLAMThread
{
public:

    SLAMThread();

    void init();

    void feed(SLAMWorkLoad* load);

    void wait();

    void halt();

protected:

    void threadProcedure();

protected:

    std::thread mThread;

    bool mInterruptionRequested;

    SLAMWorkLoad* mWorkLoad;

    Semaphore mSemaphoreStart;
    Semaphore mSemaphoreFinished;
};

