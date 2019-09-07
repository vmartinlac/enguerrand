#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <tbb/scalable_allocator.h>
#include "EngineGraph.h"
#include "Engine.h"

EngineGraph::VideoBody::VideoBody(ExitCheckerFunction exit_predicate, VideoSourcePtr input)
{
    mExitPredicate = std::move(exit_predicate);
    mInput = input;
    mNextFrameId = 0;
}

bool EngineGraph::VideoBody::operator()(EngineGraph::VideoMessagePtr& message)
{
    VideoFrame frame;
    bool ret = false;

    mInput->trigger();
    mInput->read(frame);

    if( mExitPredicate() == false && frame.isValid() )
    {
        message = std::allocate_shared<VideoMessage>( tbb::scalable_allocator<VideoMessage>() );
        message->header.available = true;
        message->header.frame_id = mNextFrameId++;
        message->frame = std::move(frame);
        message->received_time = ClockType::now();
        ret = true;
        /*
        cv::imshow("rien", message->frame.getView(0));
        cv::waitKey(0);
        */
    }
    else
    {
        message.reset();
    }

    if(message)
    {
        std::cout << "VideoBody " << message->header.frame_id << std::endl;
    }

    return ret;
}

EngineGraph::EdgeBody::EdgeBody()
{
}

EngineGraph::EdgeMessagePtr EngineGraph::EdgeBody::operator()(const EngineGraph::VideoMessagePtr video_msg)
{
    EdgeMessagePtr edge_msg;

    if( bool(video_msg) && video_msg->header.available )
    {
        edge_msg = std::allocate_shared<EdgeMessage>( tbb::scalable_allocator<EdgeMessage>() );

        edge_msg->header.available = true;
        edge_msg->header.frame_id = video_msg->header.frame_id;

        mDetector.detect(
            video_msg->frame.getView(0),
            edge_msg->edges,
            edge_msg->normals);

        //cv::imshow("rien", ret->edges);
        //cv::waitKey(0);
    }

    if(edge_msg)
    {
        std::cout << "EdgeBody " << edge_msg->header.frame_id << std::endl;
    }

    return edge_msg;
}

EngineGraph::CirclesBody::CirclesBody(HistogramPtr reference_histogram)
{
    mTracker.setReferenceHistogram(std::move(reference_histogram));
}

EngineGraph::CirclesMessagePtr EngineGraph::CirclesBody::operator()(const tbb::flow::tuple<VideoMessagePtr,EdgeMessagePtr> frame)
{
    const VideoMessagePtr video_frame = tbb::flow::get<0>(frame);
    const EdgeMessagePtr edge_frame = tbb::flow::get<1>(frame);

    CirclesMessagePtr circles_frame;

    if( bool(video_frame) && bool(edge_frame) && video_frame->header.available && edge_frame->header.available )
    {
        if( video_frame->header.frame_id != edge_frame->header.frame_id )
        {
            std::cerr << video_frame->header.frame_id << " " << edge_frame->header.frame_id << std::endl;
            std::cerr << "Fatal internal error!" << std::endl;
            exit(1);
        }

        circles_frame = std::allocate_shared<CirclesMessage>( tbb::scalable_allocator<CirclesMessage>() );

        circles_frame->header.available = true;
        circles_frame->header.frame_id = video_frame->header.frame_id;

        mTracker.track(
            video_frame->frame.getView(0),
            edge_frame->edges,
            edge_frame->normals,
            circles_frame->circles);

        circles_frame->timestamp = video_frame->frame.getTimestamp();
        circles_frame->image_size = video_frame->frame.getView(0).size();
    }

    if(circles_frame)
    {
        std::cout << "CirclesBody " << circles_frame->header.frame_id << std::endl;
    }

    return circles_frame;
}

EngineGraph::OdometryBody::OdometryBody(OdometryCodePtr odom)
{
    mOdometryCode = odom;
}

EngineGraph::OdometryMessagePtr EngineGraph::OdometryBody::operator()(const EngineGraph::CirclesMessagePtr circles)
{
    OdometryMessagePtr odometry;

    if( circles && circles->header.available )
    {
        Sophus::SE3d camera_to_world;
        bool aligned_wrt_previous;

        const bool ret = mOdometryCode->track(
            circles->timestamp,
            circles->circles,
            camera_to_world,
            aligned_wrt_previous);

        if(ret)
        {
            odometry = std::allocate_shared<OdometryMessage>( tbb::scalable_allocator<OdometryMessage>() );

            odometry->header.available = true;
            odometry->header.frame_id = circles->header.frame_id;

            odometry->camera_to_world = camera_to_world;
            odometry->aligned_wrt_previous = aligned_wrt_previous;
        }
    }

    if(odometry)
    {
        std::cout << "OdometryBody " << odometry->header.frame_id << std::endl;
    }

    return odometry;
}

EngineGraph::TracesBody::TracesBody()
{
    //mOutputFileName = "circles_trajectories.png";
}

EngineGraph::TracesMessagePtr EngineGraph::TracesBody::operator()(const EngineGraph::CirclesMessagePtr circles)
{
    std::uniform_int_distribution<uint8_t> color_distrib(64,192);

    TracesMessagePtr ret;

    if(circles && circles->header.available)
    {
        // update tracks.

        {
            std::vector<Track> new_tracks(circles->circles.size());

            for(size_t i=0; i<circles->circles.size(); i++)
            {
                Track& nt = new_tracks[i];
                const TrackedCircle& tc = circles->circles[i];

                if( tc.has_previous )
                {
                    nt.trajectory = std::move( mTracks[tc.previous].trajectory );
                    nt.color = mTracks[tc.previous].color;
                }
                else
                {
                    nt.color[0] = color_distrib(mEngine);
                    nt.color[1] = color_distrib(mEngine);
                    nt.color[2] = color_distrib(mEngine);
                }

                nt.trajectory.emplace_back( tc.circle );
            }

            mTracks = std::move(new_tracks);
        }

        //  produce output image.

        {
            cv::Mat3b output(circles->image_size);
            std::fill(output.begin(), output.end(), cv::Vec3b(0,0,0));

            for(Track& t : mTracks)
            {
                int k = 0;
                const int n = t.trajectory.size();
                const int m = 10;
                for(cv::Vec3f circle : t.trajectory)
                {
                    const cv::Point center( circle[0], circle[1] );
                    const double radius = circle[2];
                    const double gamma = std::max<double>(0.0, 1.0 - double(n-k-1)/m);
                    const cv::Vec3b color = t.color * gamma;
                    cv::circle(output, center, radius, color, -1);

                    k++;
                }
            }

            //cv::imwrite(mOutputFileName, output);
            ret.reset(new TracesMessage());
            ret->header.frame_id = circles->header.frame_id;
            ret->header.available = true;
            ret->image = output;
        }
    }

    if(ret)
    {
        std::cout << "TracesBody " << ret->header.frame_id << std::endl;
    }

    return ret;
}

EngineGraph::TerminalBody::TerminalBody(FrameListenerFunction fn)
{
    myListener = std::move(fn);
}

tbb::flow::continue_msg EngineGraph::TerminalBody::operator()(const EngineGraph::VideoEdgeCirclesOdometryTracesTuple& data)
{
    VideoMessagePtr video = tbb::flow::get<0>(data);
    EdgeMessagePtr edges = tbb::flow::get<1>(data);
    CirclesMessagePtr circles = tbb::flow::get<2>(data);
    OdometryMessagePtr odometry = tbb::flow::get<3>(data);
    TracesMessagePtr traces = tbb::flow::get<4>(data);

    const bool input_ok =
        bool(video) &&
        bool(edges) &&
        bool(circles) &&
        bool(odometry) &&
        bool(traces) &&
        video->header.frame_id == edges->header.frame_id &&
        video->header.frame_id == circles->header.frame_id &&
        video->header.frame_id == odometry->header.frame_id &&
        video->header.frame_id == traces->header.frame_id;

    if(input_ok == false)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    const bool available =
        video->header.available &&
        edges->header.available &&
        circles->header.available &&
        odometry->header.available &&
        traces->header.available;

    if(available)
    {
        EngineOutputPtr output = EngineOutputPtr::create();

        //output->frame_id = video->header.frame_id;
        //output->timestamp = video->frame.getTimestamp();
        output->input_image = video->frame.getView();
        output->edges_image = edges->edges;
        output->traces_image = traces->image;
        //output->detection_image = circles->
        output->frame_runtime = std::chrono::duration_cast<std::chrono::microseconds>(ClockType::now() - video->received_time);

        myListener(std::move(output));
    }

    return tbb::flow::continue_msg();
}

