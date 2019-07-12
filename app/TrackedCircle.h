#pragma once

#include <opencv2/core.hpp>

struct TrackedCircle
{
    cv::Vec3f circle;
    bool has_previous;
    size_t previous;
};

