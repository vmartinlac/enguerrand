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

        EngineGraph::VideoEdgeJoinNode video_edge_join(g);

        EngineGraph::CircleNode circles_node(g, 1, EngineGraph::CirclesBody(myConfig->balls_histogram));

        EngineGraph::VideoEdgeCirclesJoinNode video_edge_circles_join(g);

        EngineGraph::OdometryNode odometry_node(g, 1, EngineGraph::OdometryBody(myConfig->odometry_code));

        EngineGraph::CircleTracerNode circles_tracer_node(g, 1, EngineGraph::CirclesTracerBody());

        EngineGraph::VideoEdgeCirclesOdometryJoinNode video_edge_circles_odometry_join(g);

        EngineGraph::TerminalNode terminal_node(g, 1, EngineGraph::TerminalBody(myListener));

        make_edge(video_node, limiter_node);

        make_edge(limiter_node, edge_node);

        make_edge(limiter_node, tbb::flow::input_port<0>(video_edge_join) );
        make_edge(edge_node, tbb::flow::input_port<1>(video_edge_join) );
        make_edge(video_edge_join, circles_node);

        make_edge(limiter_node, tbb::flow::input_port<0>(video_edge_circles_join) );
        make_edge(edge_node, tbb::flow::input_port<1>(video_edge_circles_join) );
        make_edge(circles_node, tbb::flow::input_port<2>(video_edge_circles_join) );

        make_edge(circles_node, odometry_node);
        make_edge(circles_node, circles_tracer_node);

        make_edge(limiter_node, tbb::flow::input_port<0>(video_edge_circles_odometry_join) );
        make_edge(edge_node, tbb::flow::input_port<1>(video_edge_circles_odometry_join) );
        make_edge(circles_node, tbb::flow::input_port<2>(video_edge_circles_odometry_join) );
        make_edge(odometry_node, tbb::flow::input_port<3>(video_edge_circles_odometry_join) );

        make_edge(video_edge_circles_odometry_join, terminal_node);

        make_edge(terminal_node, limiter_node.decrement);

        video_node.activate();
        g.wait_for_all();
        myConfig->video_input->close();
    }
}

