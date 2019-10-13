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

    myVideoLimiterNode.reset(new EngineGraph::VideoLimiterNode(*myGraph, 4));

    myEdgeNode.reset(new EngineGraph::EdgeNode(*myGraph, tbb::flow::unlimited, EngineGraph::EdgeBody( myConfig->edge_detector )));

    myCirclesSequencerNode.reset(new EngineGraph::CirclesSequencerNode(*myGraph, [] (const EngineGraph::CirclesDetectionMessagePtr& msg) { return msg->header.frame_id; }));

    myVideoEdgeJoinNode.reset(new EngineGraph::VideoEdgeJoinNode(*myGraph,
        [] (const EngineGraph::VideoMessagePtr& msg) { return msg->header.frame_id; },
        [] (const EngineGraph::EdgeMessagePtr& msg) { return msg->header.frame_id; } ));

    myCirclesDetectionNode.reset(new EngineGraph::CirclesDetectionNode(*myGraph, tbb::flow::unlimited, EngineGraph::CirclesDetectionBody(myConfig->observation_validator)));

    myCirclesTrackingNode.reset(new EngineGraph::CirclesTrackingNode(*myGraph, 1, EngineGraph::CirclesTrackingBody()));

    myOdometryNode.reset(new EngineGraph::OdometryNode(*myGraph, 1, EngineGraph::OdometryBody(myConfig->odometry_code)));

    myTracesNode.reset(new EngineGraph::TracesNode(*myGraph, 1, EngineGraph::TracesBody()));

    mySummaryJoinNode.reset(new EngineGraph::SummaryJoinNode(*myGraph,
        [] (const EngineGraph::VideoMessagePtr& msg) { return msg->header.frame_id; },
        [] (const EngineGraph::EdgeMessagePtr& msg) { return msg->header.frame_id; },
        [] (const EngineGraph::CirclesTrackingMessagePtr& msg) { return msg->header.frame_id; },
        [] (const EngineGraph::OdometryMessagePtr& msg) { return msg->header.frame_id; },
        [] (const EngineGraph::TracesMessagePtr& msg) { return msg->header.frame_id; } ));

    mySummarySequencerNode.reset(new EngineGraph::SummarySequencerNode(*myGraph,
        [] (const EngineGraph::SummaryTuple& msg) { return tbb::flow::get<0>(msg)->header.frame_id; } ));

    myTerminalNode.reset(new EngineGraph::TerminalNode(*myGraph, 1, EngineGraph::TerminalBody(terminal_pred)));


    // input for VideoLimiterNode.
    if(synchronicity == VideoSource::SYNCHRONOUS)
    {
        make_edge(*myVideoNode, *myVideoLimiterNode);
    }

    // input for EdgeNode.
    make_edge(*myVideoLimiterNode, *myEdgeNode);

    // input for VideoEdgeJoinNode.
    make_edge(*myVideoLimiterNode, tbb::flow::input_port<0>(*myVideoEdgeJoinNode) );
    make_edge(*myEdgeNode, tbb::flow::input_port<1>(*myVideoEdgeJoinNode) );

    // input for CirclesDetectionNode.
    make_edge(*myVideoEdgeJoinNode, *myCirclesDetectionNode);

    // input for CirclesSequencerNode.
    make_edge(*myCirclesDetectionNode, *myCirclesSequencerNode);

    // input for CirclesTrackingNode.
    make_edge(*myCirclesSequencerNode, *myCirclesTrackingNode);

    // input for OdometryNode.
    make_edge(*myCirclesTrackingNode, *myOdometryNode);

    // input for TracesNode.
    make_edge(*myCirclesTrackingNode, *myTracesNode);

    // input for SummaryJoinNode.
    make_edge(*myVideoLimiterNode, tbb::flow::input_port<0>(*mySummaryJoinNode) );
    make_edge(*myEdgeNode, tbb::flow::input_port<1>(*mySummaryJoinNode) );
    make_edge(*myCirclesTrackingNode, tbb::flow::input_port<2>(*mySummaryJoinNode) );
    make_edge(*myOdometryNode, tbb::flow::input_port<3>(*mySummaryJoinNode) );
    make_edge(*myTracesNode, tbb::flow::input_port<4>(*mySummaryJoinNode) );

    // input for SummarySequencerNode.
    make_edge(*mySummaryJoinNode, *mySummarySequencerNode);

    // input for TerminalNode.
    make_edge(*mySummarySequencerNode, *myTerminalNode);

    // decrement video limiter.
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
        myCirclesDetectionNode.reset();
        myCirclesSequencerNode.reset();
        myCirclesTrackingNode.reset();
        myOdometryNode.reset();
        myTracesNode.reset();
        mySummaryJoinNode.reset();
        mySummarySequencerNode.reset();
        myTerminalNode.reset();

        myGraph.reset();

        myAsyncVideoCallback.reset();

        myIsEngineRunning = false;

        std::cout << "Engine stopped!" << std::endl;

        engineStopped();
    }
}

