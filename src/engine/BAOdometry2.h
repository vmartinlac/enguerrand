
#pragma once

#include <deque>
#include "OdometryCode.h"
#include "CalibrationData.h"

class BAOdometry2 : public OdometryCode
{
public:

    BAOdometry2(CalibrationDataPtr calibration);

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
    };

    using LandmarkPtr = std::shared_ptr<Landmark>;

    struct Observation
    {
        cv::Vec3f circle;

        bool triangulated;
        Eigen::Vector3d position_in_camera_frame;
        Eigen::Matrix3d position_information_matrix;

        LandmarkPtr landmark;
    };

    struct Frame
    {
        double timestamp;
        Sophus::SE3d camera_to_world;
        std::vector<Observation> observations;
        bool keyframe;
    };

    using FramePtr = std::shared_ptr<Frame>;

    struct BundleAdjustment;

    enum BundleAdjustmentType
    {
        BA_PNP, // Perspective N Points
        BA_LBA // Local Bundle Adjustment
    };

protected:

    void initialize(double timestamp, const std::vector<TrackedCircle>& circles);

    bool track(double timestamp, const std::vector<TrackedCircle>& circles);

    void triangulateObservation(Observation& observation);

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

