#pragma once

#include <thread>

class SLAMWorker
{
public:

    SLAMWorker();

    void feed(SLAMFramePtr frame);

protected:

    static void threadProc();

protected:

    std::thread mThread;
};

