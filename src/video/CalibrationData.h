
#pragma once

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <vector>
#include <memory>

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

    bool loadFromFile(const std::string& file);

    void dump() const;

    std::vector<CameraCalibrationData> cameras;
};

using CalibrationDataPtr = std::shared_ptr<CalibrationData>;

