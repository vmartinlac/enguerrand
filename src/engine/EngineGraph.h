
#pragma once

#include <random>
#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <tbb/flow_graph.h>
#include "EdgeDetectionCPU.h"
#include "CirclesTracker.h"
#include "VideoFrame.h"
#include "VideoSource.h"
#include "TrackedCircle.h"
#include "OdometryCode.h"
#include "EngineConfig.h"
#include "EngineOutput.h"

namespace EngineGraph
{
    using ClockType = std::chrono::high_resolution_clock;

    struct MessageHeader
    {
        MessageHeader()
        {
            available = false;
            frame_id = 0;
        }

        bool available;
        size_t frame_id;
    };


    struct VideoMessage
    {
        MessageHeader header;
        VideoFrame frame;
        ClockType::time_point received_time;
    };

    using VideoMessagePtr = std::shared_ptr<VideoMessage>;


    struct EdgeMessage
    {
        MessageHeader header;
        cv::Mat1b edges;
        cv::Mat2f normals;
    };

    using EdgeMessagePtr = std::shared_ptr<EdgeMessage>;


    struct CirclesMessage
    {
        MessageHeader header;
        std::vector<TrackedCircle> circles;
        double timestamp;
        cv::Size image_size;
    };

    using CirclesMessagePtr = std::shared_ptr<CirclesMessage>;


    struct OdometryMessage
    {
        MessageHeader header;
        bool aligned_wrt_previous;
        Sophus::SE3d camera_to_world;
    };

    using OdometryMessagePtr = std::shared_ptr<OdometryMessage>;


    struct TracesMessage
    {
        MessageHeader header;
        cv::Mat3b image;
    };

    using TracesMessagePtr = std::shared_ptr<TracesMessage>;


    using VideoEdgeTuple = tbb::flow::tuple<VideoMessagePtr,EdgeMessagePtr>;

    using VideoEdgeCirclesOdometryTracesTuple = tbb::flow::tuple<VideoMessagePtr, EdgeMessagePtr, CirclesMessagePtr, OdometryMessagePtr, TracesMessagePtr>;


    using ExitCheckerFunction = std::function<bool()>;

    using FrameListenerFunction = std::function<void(EngineOutputPtr)>;

    class VideoBody
    {
    public:

        VideoBody(ExitCheckerFunction exit_predicate, VideoSourcePtr input);

        bool operator()(VideoMessagePtr& message);

    protected:

        size_t mNextFrameId;
        VideoSourcePtr mInput;
        ExitCheckerFunction mExitPredicate;
    };

    class EdgeBody
    {
    public:

        EdgeBody();

        EdgeMessagePtr operator()(const VideoMessagePtr frame);

    protected:

        EdgeDetectionCPU mDetector;
    };

    class CirclesBody
    {
    public:

        CirclesBody(HistogramPtr reference_histogram);

        CirclesMessagePtr operator()(const tbb::flow::tuple<VideoMessagePtr,EdgeMessagePtr> frame);

    protected:

        CirclesTracker mTracker;
    };

    class OdometryBody
    {
    public:

        OdometryBody(OdometryCodePtr odom);

        OdometryMessagePtr operator()(const CirclesMessagePtr circles);

    protected:

        OdometryCodePtr mOdometryCode;
    };

    class TracesBody
    {
    public:

        TracesBody();

        TracesMessagePtr operator()(const CirclesMessagePtr circles);

    protected:

        struct Track
        {
            std::vector<cv::Vec3f> trajectory;
            cv::Vec3b color;
        };

        std::vector<Track> mTracks;
        std::default_random_engine mEngine;
    };

    class TerminalBody
    {
    public:

        TerminalBody(FrameListenerFunction listener);

        tbb::flow::continue_msg operator()(const VideoEdgeCirclesOdometryTracesTuple& items);

    protected:

        FrameListenerFunction myListener;
    };

    using VideoNode = tbb::flow::source_node<VideoMessagePtr>;

    using VideoLimiterNode = tbb::flow::limiter_node<VideoMessagePtr>;

    using EdgeNode = tbb::flow::function_node<VideoMessagePtr,EdgeMessagePtr>;

    using VideoEdgeJoinNode = tbb::flow::join_node<VideoEdgeTuple>;

    using CircleNode = tbb::flow::function_node<VideoEdgeTuple, CirclesMessagePtr>;

    using OdometryNode = tbb::flow::function_node<CirclesMessagePtr,OdometryMessagePtr>;

    using TracesNode = tbb::flow::function_node<CirclesMessagePtr,TracesMessagePtr>;

    using VideoEdgeCirclesOdometryTracesJoinNode = tbb::flow::join_node<VideoEdgeCirclesOdometryTracesTuple>;

    using TerminalNode = tbb::flow::function_node<VideoEdgeCirclesOdometryTracesTuple,tbb::flow::continue_msg>;
};

