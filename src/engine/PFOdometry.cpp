#include "PFOdometry.h"

PFOdometry::PFOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
}

bool PFOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles, OdometryFrame& output)
{
    // TODO
    output.timestamp = timestamp;
    output.aligned_wrt_previous = false;
    output.camera_to_world = Sophus::SE3d();
    output.landmarks.clear();

    return true;
}

void PFOdometry::reset()
{
    // TODO
}

