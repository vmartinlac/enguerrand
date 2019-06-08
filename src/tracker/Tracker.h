#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <memory>

struct TrackedLandmark
{
    cv::Point2d position;
    double radius;
    bool has_previous;
    size_t previous;
};

class Tracker;

typedef std::shared_ptr<Tracker> TrackerPtr;

class Tracker
{
public:

    Tracker();

    virtual void track(const cv::Mat& image, std::vector<TrackedLandmark>& result) = 0;

    static TrackerPtr createDefaultTracker();
};

