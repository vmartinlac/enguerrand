#include "SLAMThread.h"

SLAMThread::SLAMThread()
{
    mInterruptionRequested = false;
    mModuleList = nullptr;
}

void SLAMThread::startup(SLAMModuleWrapper* list)
{
    mModuleList = list;
    mInterruptionRequested = false;
    mThread = std::thread( [this] () { this->threadProcedure(); } );
}

void SLAMThread::trigger()
{
    mSemaphoreStart.up();
}

void SLAMThread::wait()
{
    mSemaphoreFinished.down();
}

void SLAMThread::threadProcedure()
{
    bool go_on = true;

    while(go_on)
    {
        mSemaphoreStart.down();

        if(mInterruptionRequested)
        {
            go_on = false;
        }
        else if(mModuleList)
        {
            mModuleList->executeSequence();
        }

        mSemaphoreFinished.up();
    }
}

void SLAMThread::shutdown()
{
    mInterruptionRequested = true;
    mSemaphoreStart.up();
    mThread.join();
}

