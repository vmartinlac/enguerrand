
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


class VideoBody
{
public:

    VideoBody(VideoSourcePtr input);

    bool operator()(VideoMessagePtr& message);

protected:

    size_t mNextFrameId;
    VideoSourcePtr mInput;
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

class CirclesTracerBody
{
public:

    CirclesTracerBody();

    tbb::flow::continue_msg operator()(const CirclesMessagePtr circles);

protected:

    struct Track
    {
        std::vector<cv::Vec3f> trajectory;
        cv::Vec3b color;
    };

    std::string mOutputFileName;
    std::vector<Track> mTracks;
    std::default_random_engine mEngine;
};
