#include <iostream>
#include <tbb/scalable_allocator.h>
#include "EngineGraph.h"

VideoBody::VideoBody(VideoSourcePtr input)
{
    mInput = input;
    mNextFrameId = 0;
}

bool VideoBody::operator()(VideoMessagePtr& message)
{
    VideoFrame frame;
    bool ret = false;

    mInput->trigger();
    mInput->read(frame);

    if(frame.isValid())
    {
        message = std::allocate_shared<VideoMessage>( tbb::scalable_allocator<VideoMessage>() );
        message->header.available = true;
        message->header.frame_id = mNextFrameId++;
        message->frame = std::move(frame);
        ret = true;
    }
    else
    {
        message.reset();
    }

    return ret;
}

EdgeBody::EdgeBody()
{
}

EdgeMessagePtr EdgeBody::operator()(const VideoMessagePtr frame)
{
    EdgeMessagePtr ret;

    if( bool(frame) && frame->header.available )
    {
        ret = std::allocate_shared<EdgeMessage>( tbb::scalable_allocator<EdgeMessage>() );

        ret->header.available = true;
        ret->header.frame_id = frame->header.frame_id;

        mDetector.detect(
            frame->frame.getView(0),
            ret->edges,
            ret->normals);
    }

    return ret;
}

CirclesBody::CirclesBody()
{
}

CirclesMessagePtr CirclesBody::operator()(const tbb::flow::tuple<VideoMessagePtr,EdgeMessagePtr> frame)
{
    const VideoMessagePtr video_frame = tbb::flow::get<0>(frame);
    const EdgeMessagePtr edge_frame = tbb::flow::get<1>(frame);

    CirclesMessagePtr circles_frame;

    if( bool(video_frame) && bool(edge_frame) && video_frame->header.available && edge_frame->header.available )
    {
        if( video_frame->header.frame_id != edge_frame->header.frame_id )
        {
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
    }

    return circles_frame;
}

OdometryBody::OdometryBody(OdometryCodePtr odom)
{
    mOdometryCode = odom;
}

OdometryMessagePtr OdometryBody::operator()(const CirclesMessagePtr circles)
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

    return odometry;
}

