
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
    struct AugmentationFunction;

    struct Landmark
    {
        Landmark();

        size_t seen_count;
        Eigen::Vector3d position;
    };

    struct State
    {
        State();

        size_t getDimension();

        Eigen::VectorXd toVector();

        void dump();

        double timestamp;
        Sophus::SE3d camera_to_world;
        Eigen::Vector3d linear_momentum;
        Eigen::Vector3d angular_momentum;
        std::vector<Landmark> landmarks;
        Eigen::MatrixXd covariance; //* \brief Dimension of covariance matrix is 12 + 3*num_landmarks.
    };

    struct CircleToLandmark
    {
        CircleToLandmark();

        bool has_landmark;
        size_t landmark;
    };

    struct ObservedLandmark
    {
        size_t landmark;
        cv::Vec3f undistorted_circle;
    };

protected:

    void initialize(
        double timestamp,
        const std::vector<TrackedCircle>& circles);

    bool trackingPrediction(double timestamp);

    bool trackingUpdate(const std::vector<TrackedCircle>& circles);

    bool mappingAugment(const std::vector<TrackedCircle>& circles);

    /**
    * \brief Triangulated landmark is in camera frame.
    * \return true on success false on failure.
    */
    bool triangulateLandmark(
        const TrackedCircle& tc,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    cv::Vec3f undistortCircle(const cv::Vec3f& c);

    void switchStates();

    State& workingState();

    State& currentState();

protected:

    double mLandmarkRadius;
    size_t mMaxLandmarks;
    double mPredictionLinearMomentumSigmaRate;
    double mPredictionAngularMomentumSigmaRate;
    double mObservationRadiusSigma;
    double mObservationPositionSigma;
    double mLandmarkMinDistanceToCamera;
    CalibrationDataPtr mCalibration;

    bool mInitialized;
    std::unique_ptr<State> mStates[2];
    std::vector<CircleToLandmark> mCircleToLandmark;
};

inline void EKFOdometry::switchStates()
{
    std::swap(mStates[0], mStates[1]);
}

inline EKFOdometry::State& EKFOdometry::currentState()
{
    return *mStates[0];
}

inline EKFOdometry::State& EKFOdometry::workingState()
{
    return *mStates[1];
}

