#pragma once

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#define SLAM_FRAME_MAX_VIEWS 2

struct SLAMFrameHeader
{
    bool ready;
    size_t id;
    double timestamp;
    size_t num_views;
    std::array<cv::Mat3b,SLAM_FRAME_MAX_VIEWS> views;
};

class SLAMFrame
{
public:

    SLAMFrame();

public:

    SLAMFrameHeader header;

    /*
    std::shared_ptr<RectificationOutput> rectification;
    edge_detection
    circle_detection;
    */
    // TODO
};

