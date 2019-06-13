#pragma once

#include <random>
#include "LineFitter.h"
#include "CircleFitter.h"
#include "Tracker.h"

class TrackerGPUImpl : public Tracker
{
public:

    TrackerGPUImpl();

    void track(const cv::Mat& image, std::vector<TrackedLandmark>& result) override;

protected:

    enum PointFlag
    {
        FLAG_EDGE=1,
        FLAG_VISITED=2,
        FLAG_NO_SEED=4,
        FLAG_CIRCLE=8
    };

protected:

    void detectEdges(const cv::Mat& image);

protected:

    std::array<cv::Vec2i,8> mNeighbors;
    cv::Mat1b mFlags;
};

