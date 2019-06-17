#include "SLAMThread.h"

SLAMThread::SLAMThread()
{
}

void SLAMThread::init()
{
    mInterruptionRequested = false;
    mThread = std::thread( [this] () { this->threadProcedure(); } );
}

void SLAMThread::feed(SLAMWorkPtr work)
{
    mCurrentWork = std::move(work);
    mSemaphoreStart.up();
}

void SLAMThread::wait()
{
    mSemaphoreFinished.down();
    mCurrentWork.reset();
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
        else if(mCurrentWork)
        {
            for(SLAMWorkItem& item : mCurrentWork->items)
            {
                if( item.module->getType() == SLAM_MODULE_CPU )
                {
                    item.module->computeCPU( item.frame );
                }
            }
        }

        mSemaphoreFinished.up();
    }
}

void SLAMThread::halt()
{
    mInterruptionRequested = true;
    mSemaphoreStart.up();
    mThread.join();
}

