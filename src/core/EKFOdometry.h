
#pragma once

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include "OdometryCode.h"
#include "CalibrationData.h"

class EKFOdometry : public OdometryCode
{
public:

    EKFOdometry(CalibrationDataPtr calibration);

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        OdometryFrame& output) override;

    void reset() override;

protected:

    struct TriangulationFunction;
    struct PredictionFunction;
    struct ObservationFunction;
    struct AugmentationFunction;

    using CeresTriangulationFunction = ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>;
    using CeresPredictionFunction = ceres::DynamicAutoDiffCostFunction<PredictionFunction>;
    using CeresObservationFunction = ceres::DynamicAutoDiffCostFunction<ObservationFunction>;
    using CeresAugmentationFunction = ceres::AutoDiffCostFunction<AugmentationFunction, 3, Sophus::SE3d::num_parameters, 3>;

    using SE3SE3CovarianceMatrix = Eigen::Matrix<double, Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters>;
    using Vector3SE3CovarianceMatrix = Eigen::Matrix<double, 3, Sophus::SE3d::num_parameters>;

    struct Observation
    {
        Observation()
        {
            circle = cv::Vec3f(0.0, 0.0, 0.0);
            has_landmark = false;
            landmark = 0;
        }

        cv::Vec3f circle;
        bool has_landmark;
        size_t landmark;
    };

    struct Landmark
    {
        Landmark()
        {
            position.setZero();
            seen_count = 0;
            currently_seen = false;
            current_observation = 0;
            updated = false;
        }

        Eigen::Vector3d position;
        size_t seen_count;
        bool currently_seen;
        size_t current_observation;
        bool updated;
    };

    struct State
    {
        State()
        {
            timestamp = 0.0;
            valid = false;
        }

        size_t getDimension() const;
        Eigen::VectorXd toVector() const;
        void dump();

        bool valid;

        double timestamp;
        Sophus::SE3d camera_to_world;
        Sophus::SE3d::Tangent momentum;
        std::vector<Landmark> landmarks;
        Eigen::MatrixXd covariance; //* \brief Dimension of covariance matrix is 12 + 3*num_landmarks.
        std::vector<Observation> observations;
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

    bool trackingPrediction(double timestamp, const std::vector<TrackedCircle>& circles);

    bool trackingUpdate();

    bool mappingPruneAugment();

    bool triangulateLandmarkInCameraFrame(
        const cv::Vec3f& circle,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    bool triangulateLandmarkInWorldFrame(
        const cv::Vec3f& circle,
        const Sophus::SE3d& camera_to_world,
        const SE3SE3CovarianceMatrix& camera_to_world_covariance,
        Eigen::Vector3d& landmark_in_world,
        Eigen::Matrix3d& covariance_landmark_landmark,
        Vector3SE3CovarianceMatrix& covariance_landmark_camera);

    void switchStates();

    State& workingState();

    State& currentState();

protected:

    size_t myMaxLandmarks;
    double myPredictionLinearMomentumSigmaRate;
    double myPredictionAngularMomentumSigmaRate;
    double myObservationRadiusSigma;
    double myObservationPositionSigma;
    double myLandmarkMinDistanceToCamera;
    CalibrationDataPtr myCalibration;

    std::unique_ptr<State> myStates[2];
};

