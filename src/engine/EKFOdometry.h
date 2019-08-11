
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

        Eigen::Vector3d position;
        size_t seen_count;
        bool seen_in_current_frame;
    };

    struct NewLandmark
    {
        Eigen::Vector3d position;
        Eigen::Matrix<double,3,3> covariance_landmark_landmark;
        Eigen::Matrix<double,3,13> covariance_landmark_camera;
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

    bool mappingPruneAugment(const std::vector<TrackedCircle>& circles);

    /**
    * \brief Triangulated landmark is in camera frame.
    * \return true on success false on failure.
    */
    bool triangulateLandmarkInCameraFrame(
        const TrackedCircle& tc,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    bool triangulateLandmarkInWorldFrame(
        const Sophus::SE3d& camera_to_world,
        const Eigen::Matrix<double, 7, 7>& pose_covariance,
        const TrackedCircle& circle,
        NewLandmark& new_landmark);

    void switchStates();

    State& workingState();

    State& currentState();

    static Eigen::Matrix<double, 7, 1> poseToVector(const Sophus::SE3d& pose);

    static Sophus::SE3d vectorToPose(const Eigen::Matrix<double, 7, 1>& vector);

    cv::Vec3f undistortCircle(const cv::Vec3f& c);

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

