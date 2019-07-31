
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
        size_t id;
        Eigen::Vector3d position_in_world;
    };

    using LandmarkPtr = std::shared_ptr<Landmark>;

    struct Projection
    {
        LandmarkPtr landmark;
        cv::Vec2f projection;
    };

    struct Frame
    {
        std::vector<Projection> projections;
    };

    using FramePtr = std::shared_ptr<Frame>;

protected:

    class LandmarkTriangulation;

protected:

    std::deque<FramePtr> mFrames;
    CalibrationDataPtr mCalibration;
};

