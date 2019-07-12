
#pragma once

#include <memory>
#include <sophus/se3.hpp>
#include "TrackedCircle.h"

class OdometryCode
{
public:

    OdometryCode();

    virtual bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        Sophus::SE3d& camera_to_world,
        bool& aligned_wrt_previous) = 0;

    virtual void reset() = 0;
};

using OdometryCodePtr = std::shared_ptr<OdometryCode>;

