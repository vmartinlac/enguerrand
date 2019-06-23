#include "PipelineThread.h"

PipelineThread::PipelineThread()
{
    mInterruptionRequested = false;
    mModuleList = nullptr;
}

void PipelineThread::startup(PipelineModuleWrapper* list)
{
    mModuleList = list;
    mInterruptionRequested = false;
    mThread = std::thread( [this] () { this->threadProcedure(); } );
}

void PipelineThread::trigger()
{
    mSemaphoreStart.up();
}

void PipelineThread::wait()
{
    mSemaphoreFinished.down();
}

void PipelineThread::threadProcedure()
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

void PipelineThread::shutdown()
{
    mInterruptionRequested = true;
    mSemaphoreStart.up();
    mThread.join();
}

