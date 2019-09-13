#include <thread>
#include <iostream>
#include "Engine.h"
#include "EngineGraph.h"

Engine::Engine(QObject* parent) : QThread(parent)
{
}

void Engine::setConfig(EngineConfigPtr config)
{
    myConfig = config;
}

void Engine::run()
{
    auto exit_pred = [this] () -> bool
    {
        return isInterruptionRequested();
    };

    auto terminal_pred = [this] (EngineOutputPtr output)
    {
        newFrame(std::move(output));
    };

    const VideoSource::SynchronicityType synchronicity = myConfig->video_input->getSynchronicity();
    SynchronousVideoSourcePtr syncvideo = myConfig->video_input->asSynchronous();
    AsynchronousVideoSourcePtr asyncvideo = myConfig->video_input->asAsynchronous();

    if( synchronicity == VideoSource::ASYNCHRONOUS || syncvideo->open() )
    {
        tbb::flow::graph g;

        EngineGraph::VideoNode video_node(g, EngineGraph::VideoBody(exit_pred, syncvideo), false);

        EngineGraph::VideoLimiterNode limiter_node(g, 1);

        EngineGraph::EdgeNode edge_node(g, 1, EngineGraph::EdgeBody());

        EngineGraph::VideoEdgeJoinNode video_edge_join(g);

        EngineGraph::CircleNode circles_node(g, 1, EngineGraph::CirclesBody(myConfig->balls_histogram));

        EngineGraph::OdometryNode odometry_node(g, 1, EngineGraph::OdometryBody(myConfig->odometry_code));

        EngineGraph::TracesNode traces_node(g, 1, EngineGraph::TracesBody());

        EngineGraph::VideoEdgeCirclesOdometryTracesJoinNode video_edge_circles_odometry_traces_join(g);

        EngineGraph::TerminalNode terminal_node(g, 1, EngineGraph::TerminalBody(terminal_pred));

        make_edge(video_node, limiter_node);

        make_edge(limiter_node, edge_node);

        make_edge(limiter_node, tbb::flow::input_port<0>(video_edge_join) );
        make_edge(edge_node, tbb::flow::input_port<1>(video_edge_join) );
        make_edge(video_edge_join, circles_node);

        make_edge(circles_node, odometry_node);
        make_edge(circles_node, traces_node);

        make_edge(limiter_node, tbb::flow::input_port<0>(video_edge_circles_odometry_traces_join) );
        make_edge(edge_node, tbb::flow::input_port<1>(video_edge_circles_odometry_traces_join) );
        make_edge(circles_node, tbb::flow::input_port<2>(video_edge_circles_odometry_traces_join) );
        make_edge(odometry_node, tbb::flow::input_port<3>(video_edge_circles_odometry_traces_join) );
        make_edge(traces_node, tbb::flow::input_port<4>(video_edge_circles_odometry_traces_join) );

        make_edge(video_edge_circles_odometry_traces_join, terminal_node);

        make_edge(terminal_node, limiter_node.decrement);

        if(synchronicity == VideoSource::SYNCHRONOUS)
        {
            video_node.activate();
        }

        g.wait_for_all();

        if(synchronicity == VideoSource::SYNCHRONOUS)
        {
            syncvideo->close();
        }
    }
}

