
#pragma once

#include "ObservationValidator.h"
#include "TrackedCircle.h"

class CircleTracker
{
public:

    CircleTracker();

    void track(
        const cv::Size& image_size,
        const std::vector<cv::Vec3f>& input,
        std::vector<TrackedCircle>& output);

    void reset();

protected:

    cv::Mat1i mLastDetectionMap;
    std::vector<TrackedCircle> mIntermediateOutput;
};

