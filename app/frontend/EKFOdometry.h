
#pragma once

#include "OdometryCode.h"

class EKFOdometry : public OdometryCode
{
public:

    EKFOdometry();

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        Sophus::SE3d& camera_to_world,
        bool& aligned_wrt_previous) override;

    void reset() override;
};

