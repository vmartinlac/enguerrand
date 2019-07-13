#include "EKFOdometry.h"

EKFOdometry::EKFOdometry()
{
}

bool EKFOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    Sophus::SE3d& camera_to_world,
    bool& aligned_wrt_previous)
{
    // TODO
    return false;
}

void EKFOdometry::reset()
{
}

