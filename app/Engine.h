#pragma once

#include <tbb/flow_graph.h>
#include <thread>
#include <mutex>
#include "VideoFrame.h"
#include "EngineFrame.h"

class Engine
{
public:

    void start();
    void stop();
    void grabFrame(VideoFrame&& frame, bool block);

protected:

    void mainProc();

protected:

    std::thread mThread;
    tbb::flow::graph* mGraph;
    tbb::flow::overwrite_node<EngineFramePtr>* mInputNode;
    std::mutex mInitMutex;
    bool mIsReady;
};
