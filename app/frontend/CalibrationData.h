
#pragma once

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <array>
#include <memory>

#define CALIBRATIONDATA_MAX_CAMERAS 5

class CameraCalibrationData
{
public:

    CameraCalibrationData();

    cv::Size image_size;
    cv::Matx33d calibration_matrix;
    cv::Matx33d inverse_calibration_matrix;
    std::vector<double> distortion_coefficients;
    Sophus::SE3d camera_to_robot;
};

class CalibrationData
{
public:

    CalibrationData();

    size_t num_cameras;
    std::array<CameraCalibrationData, CALIBRATIONDATA_MAX_CAMERAS> cameras;
};

using CalibrationDataPtr = std::shared_ptr<CalibrationData>;

