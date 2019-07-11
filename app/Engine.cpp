#include <tbb/scalable_allocator.h>
#include "VideoSource.h"
#include "EdgeDetectionCPU.h"
#include "CirclesDetection.h"
#include "Engine.h"
#include "EngineFrame.h"

/////

class EdgeNodeBody
{
public:

    EdgeNodeBody()
    {
    }

    EngineFramePtr operator()(EngineFramePtr frame)
    {
        // TODO: check input.

        mDetector.detect(
            frame->video_frame.getView(0),
            frame->edge_circles_data);

        return frame;
    }

protected:

    EdgeDetectionCPU mDetector;
};

class CirclesNodeBody
{
public:

    CirclesNodeBody()
    {
    }

    EngineFramePtr operator()(EngineFramePtr frame)
    {
        // TODO: check input.

        mDetector.detect(
            frame->video_frame.getView(0),
            frame->edge_circles_data );

        return frame;
    }

protected:

    CirclesDetection mDetector;
};


/////

void Engine::start()
{
    auto proc = [this] () { this->mainProc(); };

    mInitMutex.lock();
    mThread = std::thread(proc);
    mInitMutex.lock();
    mInitMutex.unlock();
    mIsReady = true;
}

void Engine::stop()
{
    mIsReady = false;
    mGraph->release_wait();
    mThread.join();
}

void Engine::grabFrame(VideoFrame&& video_frame, bool realtime)
{
    if(mIsReady)
    {
        EngineFramePtr engine_frame = std::allocate_shared<EngineFrame>( tbb::scalable_allocator<EngineFrame>() );
        //EngineFramePtr engine_frame = std::make_shared<EngineFrame>();
        engine_frame->video_frame = std::move(video_frame);

        mInputNode->try_put(std::move(engine_frame));
    }
}

void Engine::mainProc()
{
    tbb::flow::graph g;

    tbb::flow::overwrite_node<EngineFramePtr> input_node(g);
    tbb::flow::function_node<EngineFramePtr,EngineFramePtr> edge_node(g, 1, EdgeNodeBody());
    tbb::flow::function_node<EngineFramePtr,EngineFramePtr> circles_node(g, 1, CirclesNodeBody());

    make_edge(input_node, edge_node);
    make_edge(edge_node, circles_node);

    mGraph = &g;
    mInputNode = &input_node;

    g.reserve_wait();

    mInitMutex.unlock();
    g.wait_for_all();
}

