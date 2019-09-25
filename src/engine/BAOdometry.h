
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
        OdometryFrame& output) override;

    void reset() override;

protected:

    struct Landmark
    {
        bool visited;
        size_t local_id;
        Eigen::Vector3d position;
        double ceres_position[3];
        size_t seen_count;

        void preBundleAdjustment();
        void postBundleAdjustment();
    };

    using LandmarkPtr = std::shared_ptr<Landmark>;

    struct Observation
    {
        LandmarkPtr landmark;
        cv::Vec3f circle;
        cv::Vec3f undistorted_circle;
    };

    struct Frame
    {
        double timestamp;
        Sophus::SE3d camera_to_world;
        double ceres_camera_to_world_t[3];
        double ceres_camera_to_world_q[4];
        std::vector<Observation> observations;
        bool keyframe;

        void preBundleAdjustment();
        void postBundleAdjustment();
    };

    using FramePtr = std::shared_ptr<Frame>;

    struct BundleAdjustment;

    enum BundleAdjustmentType
    {
        BA_TRIANGULATION,
        BA_PNP, // Perspective N Points
        BA_LBA // Local Bundle Adjustment
    };

protected:

    void initialize(double timestamp, const std::vector<TrackedCircle>& circles);

    bool track(double timestamp, const std::vector<TrackedCircle>& circles);

    LandmarkPtr triangulateInitialLandmark(const Sophus::SE3d& camera_to_world, const cv::Vec3f& circle);

    void performBundleAdjustment(BundleAdjustmentType type);

    cv::Vec3f undistortCircle(const cv::Vec3f& c);

    template<typename FramePtrContainer>
    size_t buildLocalMap(FramePtrContainer& frames, std::vector<LandmarkPtr>& local_map);

protected:

    double myLandmarkRadius;
    size_t myMaxKeyFrames;
    CalibrationDataPtr myCalibration;

    std::deque<FramePtr> myFrames;
};

