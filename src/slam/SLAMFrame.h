#pragma once

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#define SLAM_FRAME_MAX_VIEWS 2

struct SLAMFrameHeader
{
    SLAMFrameHeader()
    {
        ready = false;
        slam_frame_id = 0;
        video_frame_id = 0;
        video_timestamp = 0.0;
        num_views = 0;
    }

    bool ready;
    size_t slam_frame_id;
    size_t video_frame_id;
    double video_timestamp;
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
    rectification
    edge_detection
    circle_detection;
    */
};

