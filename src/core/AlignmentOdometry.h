
#pragma once

#include <deque>
#include "OdometryCode.h"
#include "CalibrationData.h"

class AlignmentOdometry : public OdometryCode
{
public:

    AlignmentOdometry(CalibrationDataPtr calibration);

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        OdometryFrame& output) override;

    void reset() override;

protected:

    struct Observation
    {
        bool valid;
        Eigen::Vector3d position_in_camera;
        Eigen::Vector3d position_in_world;
    };

    struct Frame
    {
        size_t odometry_frame_id;
        double timestamp;
        Sophus::SE3d camera_to_world;
        std::vector<Observation> observations;
    };

    using FramePtr = std::shared_ptr<Frame>;

protected:

    void initialize(double timestamp, const std::vector<TrackedCircle>& circles);

    bool track(double timestamp, const std::vector<TrackedCircle>& circles);

    bool performAlignment(FramePtr frame);

protected:

    struct Alignment;

protected:

    double myLandmarkRadius;
    CalibrationDataPtr myCalibration;
    FramePtr myLastFrame;
};

