
#pragma once

#include <random>
#include <ceres/ceres.h>
#include "OdometryCode.h"
#include "CalibrationData.h"

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

        double weight;
        Sophus::SE3d camera_to_world;
    };

    struct Observation
    {
        Observation();

        cv::Vec3f circle;
        bool has_landmark;
        size_t landmark;
    };

    struct State
    {
        State();

        size_t frame_id;
        double timestamp;
        size_t num_landmarks;
        std::vector<Particle> particles;
        std::vector<LandmarkEstimation> landmark_estimations;
        std::vector<Observation> observations;

        LandmarkEstimation& refLandmarkEstimation(size_t particle, size_t landmark);
        void save(OdometryFrame& output, bool aligned_wrt_previous);
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
        const cv::Vec3f& circle, 
        const Sophus::SE3d& camera_to_world,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    void updateLandmark(
        const Sophus::SE3d& camera_to_world,
        const cv::Vec3f& observation,
        LandmarkEstimation& landmark);

    bool trackAndMap(double timestamp, const std::vector<TrackedCircle>& circles);

protected:

    CalibrationDataPtr myCalibration;
    size_t myNumParticles;
    std::unique_ptr<State> myCurrentState;
    std::unique_ptr<State> myWorkingState;
    RandomEngine myRandomEngine;
    double myPredictionPositionNoise;
    double myPredictionAttitudeNoise;
    double myCirclePositionNoise;
    double myCircleRadiusNoise;
};

