
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

    struct Landmark
    {
        Eigen::Vector3d position_in_world;
    };

    struct State
    {
        double timestamp;
        bool aligned_wrt_previous;
        Sophus::SE3d camera_to_world;
        Eigen::Vector3d linear_momentum;
        Eigen::Vector3d angular_momentum;
        std::vector<Landmark> landmarks;
        Eigen::MatrixXd covariance; //* \brief Dimension of covariance matrix is 12 + 3*num_landmarks.
    };

protected:

    void initialize(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        State& new_state);

    void estimateLandmarkPositionInCameraFrame(
        const TrackedCircle& tc,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

protected:

    bool mInitialized;
    int mStateOffset;
    State mState[2];
    CalibrationDataPtr mCalibration;
};

