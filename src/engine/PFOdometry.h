
#pragma once

#include "OdometryCode.h"
#include "CalibrationData.h"

class PFOdometry : public OdometryCode
{
public:

    PFOdometry(CalibrationDataPtr calibration);

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        OdometryFrame& output) override;

    void reset() override;

protected:

    CalibrationDataPtr myCalibration;
};

