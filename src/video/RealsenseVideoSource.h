
#pragma once

#include <chrono>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <opencv2/videoio.hpp>
#include "VideoSource.h"

struct RealsenseCalibration
{
    cv::Matx33d calibration_matrix;
    cv::Size image_size;
    std::vector<double> distortion_coefficients;
};

class RealsenseVideoSource : public AsynchronousVideoSource
{
public:

    RealsenseVideoSource(rs2::sensor sensor, rs2::stream_profile profile);

    bool start() override;

    void stop() override;

    int getNumViews() override;

    bool getCalibration(RealsenseCalibration& calibration) const;

protected:

    using ClockType = std::chrono::high_resolution_clock;

protected:

    rs2::sensor mySensor;
    rs2::stream_profile myProfile;
    size_t myNextId;
    ClockType::time_point myTimeZero;
};

typedef std::shared_ptr<RealsenseVideoSource> RealsenseVideoSourcePtr;

