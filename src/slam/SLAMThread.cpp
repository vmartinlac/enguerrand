#include "SLAMThread.h"

SLAMThread::SLAMThread()
{
    mWorkLoad = nullptr;
}

void SLAMThread::init()
{
    mInterruptionRequested = false;
    mThread = std::thread( [this] () { this->threadProcedure(); } );
}

void SLAMThread::feed(SLAMWorkLoad* load)
{
    mWorkLoad = load;
    mSemaphoreStart.up();
}

void SLAMThread::wait()
{
    mSemaphoreFinished.down();
    mWorkLoad = nullptr;
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
        else if(mWorkLoad)
        {
            mWorkLoad->execute();
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

