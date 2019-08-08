
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
        bool visited;
        Eigen::Vector3d position;
        double ceres_position[3];
        size_t seen_count;
    };

    using LandmarkPtr = std::shared_ptr<Landmark>;

    struct Observation
    {
        LandmarkPtr landmark;
        cv::Vec3f circle;
    };

    struct Frame
    {
        double timestamp;
        Sophus::SE3d camera_to_world;
        double ceres_camera_to_world_t[3];
        double ceres_camera_to_world_q[4];
        std::vector<Observation> observations;
    };

    using FramePtr = std::shared_ptr<Frame>;

    class BundleAdjustment;

protected:

    void initialize(double timestamp, const std::vector<TrackedCircle>& circles);

    bool track(double timestamp, const std::vector<TrackedCircle>& circles);

    LandmarkPtr triangulateInitialLandmark(const cv::Vec3f& circle);

    void triangulationAdjustment();

    void localBundleAdjustment();

    void PnPAdjustment();

protected:

    double myLandmarkRadius;
    double myMaxKeyFrames;
    double myMaxProjections;
    CalibrationDataPtr myCalibration;

    FramePtr myLastFrame;
    std::deque<FramePtr> myKeyFrames;
    std::vector<LandmarkPtr> myObservedLandmarks;
};

