
#pragma once

#include <memory>
#include <sophus/se3.hpp>
#include "TrackedCircle.h"

struct OdometryFrameLandmark
{
    bool first_seen;
    size_t id;
    bool seen_in_current_frame;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
};

struct OdometryFrame
{
    double timestamp;
    bool aligned_wrt_previous;
    Sophus::SE3d camera_to_world;
    std::vector<OdometryFrameLandmark> landmarks;
};

class OdometryCode
{
public:

    OdometryCode();

    virtual bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        OdometryFrame& output) = 0;

    virtual void reset() = 0;
};

using OdometryCodePtr = std::shared_ptr<OdometryCode>;

