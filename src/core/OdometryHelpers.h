#pragma once

#include "CalibrationData.h"

namespace OdometryHelpers
{
    bool triangulateLandmark(const cv::Vec3f& undistorted_circle, const CalibrationDataPtr calibration, Eigen::Vector3d& landmark);

    cv::Vec3f undistortCircle(const cv::Vec3f& circle, const CalibrationDataPtr calibration);
};

