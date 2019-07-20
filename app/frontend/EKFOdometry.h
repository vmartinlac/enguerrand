
#pragma once

#include "OdometryCode.h"
#include "CalibrationData.h"

class EKFOdometry : public OdometryCode
{
public:

    EKFOdometry(CalibrationDataPtr calibration);

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        Sophus::SE3d& camera_to_world,
        bool& aligned_wrt_previous) override;

    void reset() override;

protected:

    struct TriangulationFunctor;
    struct PredictionFunctor;
    struct UpdateFunctor;

    struct Landmark
    {
        Eigen::Vector3d position;
    };

    struct State
    {
        double timestamp;
        Sophus::SE3d camera_to_world;
        Eigen::Vector3d linear_momentum;
        Eigen::Vector3d angular_momentum;
        std::vector<Landmark> landmarks;
        Eigen::MatrixXd covariance; //* \brief Dimension of covariance matrix is 12 + 3*num_landmarks.
    };

    struct CircleToLandmark
    {
        bool has_landmark;
        size_t landmark;
    };

protected:

    void initialize(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        State& new_state);

    /**
    * \brief Triangulated landmark is in camera frame.
    * \return true on success false on failure.
    */
    bool triangulateLandmark(
        const TrackedCircle& tc,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    cv::Vec3f undistortCircle(const cv::Vec3f& c);

protected:

    bool mInitialized;
    double mLandmarkRadius;
    int mStateOffset;
    State mState[2];
    CalibrationDataPtr mCalibration;
    std::vector<CircleToLandmark> mCirclesToLandmark;
};

