#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
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

EdgeBody::EdgeBody()
{
}

EdgeMessagePtr EdgeBody::operator()(const VideoMessagePtr video_msg)
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

CirclesBody::CirclesBody(HistogramPtr reference_histogram)
{
    mTracker.setReferenceHistogram(std::move(reference_histogram));
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

    if(odometry)
    {
        std::cout << "OdometryBody " << odometry->header.frame_id << std::endl;
    }

    return odometry;
}

CirclesTracerBody::CirclesTracerBody()
{
    mOutputFileName = "circles_trajectories.png";
}

tbb::flow::continue_msg CirclesTracerBody::operator()(const CirclesMessagePtr circles)
{
    std::uniform_int_distribution<uint8_t> color_distrib(64,192);

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

            cv::imwrite(mOutputFileName, output);
        }
    }

    return tbb::flow::continue_msg();
}

