
#pragma once

#include <random>
#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <tbb/flow_graph.h>
#include "EdgeDetection.h"
#include "CircleTracker.h"
#include "CircleDetector.h"
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


    struct CirclesDetectionMessage
    {
        MessageHeader header;
        double timestamp;
        cv::Size image_size;
        std::vector<cv::Vec3f> circles;
    };

    using CirclesDetectionMessagePtr = std::shared_ptr<CirclesDetectionMessage>;


    struct CirclesTrackingMessage
    {
        MessageHeader header;
        double timestamp;
        cv::Size image_size;
        std::vector<TrackedCircle> circles;
    };

    using CirclesTrackingMessagePtr = std::shared_ptr<CirclesTrackingMessage>;


    struct OdometryMessage
    {
        MessageHeader header;
        OdometryFrame frame;
    };

    using OdometryMessagePtr = std::shared_ptr<OdometryMessage>;


    struct TracesMessage
    {
        MessageHeader header;
        cv::Mat3b image;
    };

    using TracesMessagePtr = std::shared_ptr<TracesMessage>;


    using VideoEdgeTuple = tbb::flow::tuple<VideoMessagePtr,EdgeMessagePtr>;

    using SummaryTuple = tbb::flow::tuple<VideoMessagePtr, EdgeMessagePtr, CirclesTrackingMessagePtr, OdometryMessagePtr, TracesMessagePtr>;

    class AsyncVideoCallback : public AsynchronousVideoCallback
    {
    public:

        AsyncVideoCallback(tbb::flow::receiver<VideoMessagePtr>& receiver);

        void operator()(VideoFrame&& frame) final;

    protected:

        tbb::flow::receiver<VideoMessagePtr>& myReceiver;
        size_t myFrameCount;
    };

    using ExitCheckerFunction = std::function<bool()>;

    using FrameListenerFunction = std::function<void(EngineOutputPtr)>;

    class VideoBody
    {
    public:

        VideoBody(ExitCheckerFunction exit_predicate, SynchronousVideoSourcePtr input);

        bool operator()(VideoMessagePtr& message);

    protected:

        size_t mNextFrameId;
        SynchronousVideoSourcePtr mInput;
        ExitCheckerFunction mExitPredicate;
    };

    class EdgeBody
    {
    public:

        EdgeBody(EdgeDetectionPtr filter);

        EdgeMessagePtr operator()(const VideoMessagePtr frame);

    protected:

        EdgeDetectionPtr mDetector;
    };

    class CirclesDetectionBody
    {
    public:

        CirclesDetectionBody(ObservationValidatorPtr observation_validator);

        CirclesDetectionMessagePtr operator()(const VideoEdgeTuple& frame);

    protected:

        CircleDetector mDetector;
    };

    class CirclesTrackingBody
    {
    public:

        CirclesTrackingBody();

        CirclesTrackingMessagePtr operator()(const CirclesDetectionMessagePtr frame);

    protected:

        CircleTracker mTracker;
    };

    class OdometryBody
    {
    public:

        OdometryBody(OdometryCodePtr odom);

        OdometryMessagePtr operator()(const CirclesTrackingMessagePtr circles);

    protected:

        OdometryCodePtr mOdometryCode;
    };

    class TracesBody
    {
    public:

        TracesBody();

        TracesMessagePtr operator()(const CirclesTrackingMessagePtr circles);

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

        tbb::flow::continue_msg operator()(const SummaryTuple& items);

    protected:

        FrameListenerFunction myListener;
    };

    using VideoNode = tbb::flow::source_node<VideoMessagePtr>;

    using VideoLimiterNode = tbb::flow::limiter_node<VideoMessagePtr>;

    using EdgeNode = tbb::flow::function_node<VideoMessagePtr,EdgeMessagePtr>;

    using VideoEdgeJoinNode = tbb::flow::join_node<VideoEdgeTuple, tbb::flow::tag_matching>;

    using CirclesDetectionNode = tbb::flow::function_node<VideoEdgeTuple, CirclesDetectionMessagePtr>;

    using CirclesTrackingNode = tbb::flow::function_node<CirclesDetectionMessagePtr, CirclesTrackingMessagePtr>;

    using CirclesSequencerNode = tbb::flow::sequencer_node<CirclesDetectionMessagePtr>;

    using OdometryNode = tbb::flow::function_node<CirclesTrackingMessagePtr,OdometryMessagePtr>;

    using TracesNode = tbb::flow::function_node<CirclesTrackingMessagePtr,TracesMessagePtr>;

    using SummaryJoinNode = tbb::flow::join_node<SummaryTuple, tbb::flow::tag_matching>;

    using TerminalNode = tbb::flow::function_node<SummaryTuple,tbb::flow::continue_msg>;
};

