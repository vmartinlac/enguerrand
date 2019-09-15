#include <thread>
#include <iostream>
#include "Engine.h"
#include "EngineGraph.h"

Engine::Engine(QObject* parent) : QThread(parent)
{
    myIsEngineRunning = false;
}

void Engine::startEngine(EngineConfigPtr config)
{
    stopEngine();

    myExitRequested = false;
    myConfig = config;

    auto exit_pred = [this] () -> bool
    {
        return myExitRequested;
    };

    auto terminal_pred = [this] (EngineOutputPtr output)
    {
        newFrame(std::move(output));
    };

    const VideoSource::SynchronicityType synchronicity = myConfig->video_input->getSynchronicity();
    SynchronousVideoSourcePtr syncvideo = myConfig->video_input->asSynchronous();
    AsynchronousVideoSourcePtr asyncvideo = myConfig->video_input->asAsynchronous();

    myGraph.reset(new tbb::flow::graph);

    if( synchronicity == VideoSource::SYNCHRONOUS )
    {
        myVideoNode.reset(new EngineGraph::VideoNode(*myGraph, EngineGraph::VideoBody(exit_pred, syncvideo), false));
    }

    myVideoLimiterNode.reset(new EngineGraph::VideoLimiterNode(*myGraph, 1));

    myEdgeNode.reset(new EngineGraph::EdgeNode(*myGraph, 1, EngineGraph::EdgeBody()));

    myVideoEdgeJoinNode.reset(new EngineGraph::VideoEdgeJoinNode(*myGraph));

    myCirclesNode.reset(new EngineGraph::CircleNode(*myGraph, 1, EngineGraph::CirclesBody(myConfig->balls_histogram)));

    myOdometryNode.reset(new EngineGraph::OdometryNode(*myGraph, 1, EngineGraph::OdometryBody(myConfig->odometry_code)));

    myTracesNode.reset(new EngineGraph::TracesNode(*myGraph, 1, EngineGraph::TracesBody()));

    myVideoEdgeCirclesOdometryTracesJoinNode.reset(new EngineGraph::VideoEdgeCirclesOdometryTracesJoinNode(*myGraph));

    myTerminalNode.reset(new EngineGraph::TerminalNode(*myGraph, 1, EngineGraph::TerminalBody(terminal_pred)));

    if(synchronicity == VideoSource::SYNCHRONOUS)
    {
        make_edge(*myVideoNode, *myVideoLimiterNode);
    }

    make_edge(*myVideoLimiterNode, *myEdgeNode);

    make_edge(*myVideoLimiterNode, tbb::flow::input_port<0>(*myVideoEdgeJoinNode) );
    make_edge(*myEdgeNode, tbb::flow::input_port<1>(*myVideoEdgeJoinNode) );

    make_edge(*myVideoEdgeJoinNode, *myCirclesNode);

    make_edge(*myCirclesNode, *myOdometryNode);

    make_edge(*myCirclesNode, *myTracesNode);

    make_edge(*myVideoLimiterNode, tbb::flow::input_port<0>(*myVideoEdgeCirclesOdometryTracesJoinNode) );
    make_edge(*myEdgeNode, tbb::flow::input_port<1>(*myVideoEdgeCirclesOdometryTracesJoinNode) );
    make_edge(*myCirclesNode, tbb::flow::input_port<2>(*myVideoEdgeCirclesOdometryTracesJoinNode) );
    make_edge(*myOdometryNode, tbb::flow::input_port<3>(*myVideoEdgeCirclesOdometryTracesJoinNode) );
    make_edge(*myTracesNode, tbb::flow::input_port<4>(*myVideoEdgeCirclesOdometryTracesJoinNode) );

    make_edge(*myVideoEdgeCirclesOdometryTracesJoinNode, *myTerminalNode);

    make_edge(*myTerminalNode, myVideoLimiterNode->decrement);

    if(synchronicity == VideoSource::SYNCHRONOUS)
    {
        myExitRequested = false;
        syncvideo->open();
        myVideoNode->activate();
    }
    else if(synchronicity == VideoSource::ASYNCHRONOUS)
    {
        myAsyncVideoCallback.reset(new EngineGraph::AsyncVideoCallback(*myVideoLimiterNode));
        asyncvideo->setCallback(myAsyncVideoCallback.get());
        myGraph->reserve_wait();
        asyncvideo->start();
    }
    else
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    myIsEngineRunning = true;

    std::cout << "Engine started!" << std::endl;

    engineStarted();
}

void Engine::stopEngine()
{
    if( myIsEngineRunning )
    {
        const VideoSource::SynchronicityType synchronicity = myConfig->video_input->getSynchronicity();
        SynchronousVideoSourcePtr syncvideo = myConfig->video_input->asSynchronous();
        AsynchronousVideoSourcePtr asyncvideo = myConfig->video_input->asAsynchronous();

        if(synchronicity == VideoSource::SYNCHRONOUS)
        {
            myExitRequested = true;
            myGraph->wait_for_all();
            syncvideo->close();
        }
        else if(synchronicity == VideoSource::ASYNCHRONOUS)
        {
            asyncvideo->stop();
            myGraph->release_wait();
            myGraph->wait_for_all();
        }
        else
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        myConfig.reset();
        myExitRequested = false;
        myVideoNode.reset();
        myVideoLimiterNode.reset();
        myEdgeNode.reset();
        myVideoEdgeJoinNode.reset();
        myCirclesNode.reset();
        myOdometryNode.reset();
        myTracesNode.reset();
        myVideoEdgeCirclesOdometryTracesJoinNode.reset();
        myTerminalNode.reset();
        myGraph.reset();
        myAsyncVideoCallback.reset();

        myIsEngineRunning = false;

        std::cout << "Engine stopped!" << std::endl;

        engineStopped();
    }
}

