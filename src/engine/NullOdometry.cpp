#include "NullOdometry.h"

NullOdometry::NullOdometry()
{
}

NullOdometry::~NullOdometry()
{
}

bool NullOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    OdometryFrame& output)
{
    output.timestamp = timestamp;
    output.aligned_wrt_previous = false;
    output.camera_to_world = Sophus::SE3d();
    output.landmarks.clear();

    return true;
}

void NullOdometry::reset()
{
}

