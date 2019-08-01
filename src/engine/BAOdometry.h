
#pragma once

#include <deque>
#include "OdometryCode.h"
#include "CalibrationData.h"

class BAOdometry : public OdometryCode
{
public:

    BAOdometry(CalibrationDataPtr calibration);

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        Sophus::SE3d& camera_to_world,
        bool& aligned_wrt_previous) override;

    void reset() override;

protected:

    struct Landmark
    {
        Eigen::Vector3d position;
        size_t seen_count;
        size_t reference_count;
    };

    struct Projection
    {
        size_t landmark;
        cv::Vec3f projection;
    };

    struct Frame
    {
        size_t id;
        double timestamp;
        Sophus::SE3d camera_to_world;
        double ceres_camera_to_world_t[3];
        double ceres_camera_to_world_q[4];
        std::vector<Projection> projections;
    };

    class LandmarkTriangulation;

protected:

    void clear();

protected:

    double myLandmarkRadius;
    double myMaxKeyFrames;
    double myMaxProjections;
    CalibrationDataPtr myCalibration;
    std::vector<Landmark> myLandmarks;
    std::vector<size_t> myAvailableLandmarks;
    std::deque<Frame> myKeyFrames;
};

