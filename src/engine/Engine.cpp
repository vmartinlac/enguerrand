#include <thread>
#include <iostream>
#include "Engine.h"
#include "EngineGraph.h"
#include "EngineListener.h"

Engine::Engine(QObject* parent) : QThread(parent)
{
    myListener = nullptr;
}

void Engine::setListener(EngineListener* listener)
{
    myListener = listener;
}

void Engine::setConfig(EngineConfigPtr config)
{
    myConfig = config;
}

void Engine::run()
{
    if( myConfig->video_input->open() )
    {
        tbb::flow::graph g;

        EngineGraph::VideoNode video_node(g, EngineGraph::VideoBody(this, myConfig->video_input), false);

        EngineGraph::VideoLimiterNode limiter_node(g, 3);

        EngineGraph::EdgeNode edge_node(g, 1, EngineGraph::EdgeBody());

        EngineGraph::VideoEdgeJoinNode join_node(g);

        EngineGraph::CircleNode circles_node(g, 1, EngineGraph::CirclesBody(myConfig->balls_histogram));

        EngineGraph::CircleBroadcastNode broadcast_node(g);

        EngineGraph::OdometryNode odometry_node(g, 1, EngineGraph::OdometryBody(myConfig->odometry_code));

        EngineGraph::CircleTracerNode circles_tracer_node(g, 1, EngineGraph::CirclesTracerBody());

        EngineGraph::TerminalNode terminal_node(g, 1, EngineGraph::TerminalBody());

        make_edge(video_node, limiter_node);
        make_edge(limiter_node, edge_node);
        make_edge(limiter_node, tbb::flow::input_port<0>(join_node) );
        make_edge(edge_node, tbb::flow::input_port<1>(join_node) );
        make_edge(join_node, circles_node);
        make_edge(circles_node, broadcast_node);
        make_edge(broadcast_node, odometry_node);
        make_edge(broadcast_node, circles_tracer_node);
        make_edge(odometry_node, terminal_node);
        make_edge(terminal_node, limiter_node.decrement);

        video_node.activate();
        g.wait_for_all();

        myConfig->video_input->close();
    }
}

