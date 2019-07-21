
#pragma once

//#include <random>
#include <ceres/ceres.h>
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

    struct TriangulationFunction;
    struct PredictionFunction;
    struct ObservationFunction;

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

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        State& old_state,
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

    double mLandmarkRadius;
    size_t mMaxLandmarks;
    bool mInitialized;
    int mStateOffset;
    State mState[2];
    CalibrationDataPtr mCalibration;
    std::vector<CircleToLandmark> mCirclesToLandmark;
    TriangulationFunction* mTriangulationFunction;
    PredictionFunction* mPredictionFunction;
    ObservationFunction* mObservationFunction;
    std::unique_ptr<ceres::CostFunction> mTriangulationCostFunction;
    std::unique_ptr<ceres::CostFunction> mPredictionCostFunction;
    std::unique_ptr<ceres::CostFunction> mObservationCostFunction;
    //std::default_random_engine mEngine;
};

