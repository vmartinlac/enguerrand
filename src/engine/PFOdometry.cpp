#include "PFOdometry.h"

PFOdometry::PFOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
}

bool PFOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles, OdometryFrame& output)
{
    // TODO

    return false;
}

void PFOdometry::reset()
{
    // TODO
}

