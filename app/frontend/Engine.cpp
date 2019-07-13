#include <iostream>
#include "VideoSource.h"
#include "Engine.h"
#include "EngineGraph.h"

bool Engine::exec(VideoSourcePtr video, OdometryCodePtr odometry_code)
{
    const char* err = "";
    bool ok = true;

    if(ok)
    {
        ok = video->open();
        err = "Could not open video input.";
    }

    if(ok)
    {
        tbb::flow::graph g;

        tbb::flow::source_node<VideoMessagePtr> video_node(g, VideoBody(video));

        tbb::flow::limiter_node<VideoMessagePtr> limiter_node(g, 4);

        tbb::flow::function_node<VideoMessagePtr,EdgeMessagePtr> edge_node(g, 2, EdgeBody());

        tbb::flow::join_node< tbb::flow::tuple<VideoMessagePtr,EdgeMessagePtr> > join_node(g);

        tbb::flow::function_node< tbb::flow::tuple<VideoMessagePtr,EdgeMessagePtr>, CirclesMessagePtr> circles_node(g, 1, CirclesBody());

        tbb::flow::broadcast_node<CirclesMessagePtr> broadcast_node(g);

        tbb::flow::function_node<CirclesMessagePtr,OdometryMessagePtr> odometry_node(g, 1, OdometryBody(odometry_code));

        tbb::flow::function_node<CirclesMessagePtr> circles_tracer_node(g, 1, CirclesTracerBody());

        make_edge(video_node, limiter_node);
        make_edge(limiter_node, edge_node);
        make_edge(limiter_node, tbb::flow::input_port<0>(join_node) );
        make_edge(edge_node, tbb::flow::input_port<1>(join_node) );
        make_edge(join_node, circles_node);
        make_edge(circles_node, broadcast_node);
        make_edge(broadcast_node, odometry_node);
        make_edge(broadcast_node, circles_tracer_node);

        // TMP
        tbb::flow::function_node<OdometryMessagePtr,tbb::flow::continue_msg> endnode(g, 1, [] (const OdometryMessagePtr msg) { return tbb::flow::continue_msg(); });
        make_edge(odometry_node, endnode);
        make_edge(endnode, limiter_node.decrement);
        //

        g.wait_for_all();

        video->close();
    }

    if(ok == false)
    {
        std::cerr << err << std::endl;
        exit(1);
    }

    return ok;
}

