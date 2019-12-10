
#pragma once

#include <random>
#include <ceres/ceres.h>
#include "OdometryCode.h"
#include "CalibrationData.h"
#include "Tensor.h"

class PFOdometry : public OdometryCode
{
public:

    PFOdometry(CalibrationDataPtr calibration);

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        OdometryFrame& output) override;

    void reset() override;

protected:

    using RandomEngine = std::default_random_engine;

    struct LandmarkEstimation
    {
        LandmarkEstimation();

        Eigen::Vector3d position;
        Eigen::Matrix3d covariance;
    };

    struct Particle
    {
        Particle();

        Sophus::SE3d camera_to_world;
    };

    struct Observation
    {
        Observation();

        cv::Vec3d undistorted_circle;
        bool has_landmark;
        size_t landmark;
    };

    struct State
    {
        State();

        void dump(const char* stage);
        void check();

        size_t frame_id;
        double timestamp;
        std::vector<Particle> particles;
        std::vector<Observation> observations;
        Tensor<LandmarkEstimation,2> landmark_estimations;
    };

    struct TriangulationFunction;
    struct LandmarkObservationFunction;

    using CeresTriangulationFunction = ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>;
    using CeresLandmarkObservationFunction = ceres::AutoDiffCostFunction<LandmarkObservationFunction, 3, 3>;

protected:

    void initialize(
        double timestamp,
        const std::vector<TrackedCircle>& circles);

    bool triangulateLandmark(
        const cv::Vec3d& circle, 
        const Sophus::SE3d& camera_to_world,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    bool updateLandmark(
        const Sophus::SE3d& camera_to_world,
        const cv::Vec3f& observation,
        LandmarkEstimation& landmark);

    bool exportCurrentState(OdometryFrame& output, bool aligned_wrt_previous);

    bool predictionStep(double timestamp, const std::vector<TrackedCircle>& circles);

    bool landmarkUpdateStep();

    bool resamplingStep();

    bool mappingStep();

protected:

    CalibrationDataPtr myCalibration;
    size_t myNumParticles;
    std::unique_ptr<State> myCurrentState;
    std::unique_ptr<State> myWorkingState;
    RandomEngine myRandomEngine;
    double myPredictionLinearVelocitySigma;
    double myPredictionAngularVelocitySigma;
    double myCirclePositionNoise;
    double myCircleRadiusNoise;
};

