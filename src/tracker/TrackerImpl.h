#pragma once

#include <random>
#include "Tracker.h"

class TrackerImpl : public Tracker
{
public:

    TrackerImpl();

    void track(const cv::Mat& image, std::vector<TrackedLandmark>& result) override;

protected:

    enum PointFlag
    {
        FLAG_EDGE=1,
        FLAG_VISITED=2,
        FLAG_NO_SEED=4

    };

protected:

    void detectEdges(const cv::Mat& image);
    void findCircles();
    void findCircle(const cv::Vec2i& seed);

    template<typename T>
    void growPatch(std::vector<cv::Vec2i>& patch, const T& pred);

protected:

    std::array<cv::Vec2i,8> mNeighbors;
    std::default_random_engine mEngine;

    cv::Mat1b mFlags;
};

