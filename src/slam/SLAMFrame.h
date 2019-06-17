#pragma once

#include <memory>
#include <opencv2/core.hpp>

class SLAMFrame
{
public:

    SLAMFrame();

public:

    size_t id;
    double timestamp;
    cv::Mat3b image;

    /*
    std::shared_ptr<RectificationOutput> rectification;
    edge_detection
    circle_detection;
    */
    // TODO
};

typedef std::shared_ptr<SLAMFrame> SLAMFramePtr;

