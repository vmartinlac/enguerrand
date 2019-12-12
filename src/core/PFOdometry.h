
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

    struct Landmark
    {
        Eigen::Vector3d position;
        Eigen::Matrix3d covariance;
    };

    struct Particle
    {
        double importance_factor;
        Sophus::SE3d camera_to_world;
    };

    struct Observation
    {
        cv::Vec3d undistorted_circle;
        bool has_landmark;
        size_t landmark;
    };

    struct State
    {
        size_t getNumParticles()
        {
            return landmarks.size(0);
        }

        size_t getNumLandmarks()
        {
            return landmarks.size(1);
        }

        size_t frame_id;
        double timestamp;
        std::vector<Observation> observations;
        std::vector<Particle> particles;
        Tensor<Landmark,2> landmarks;
    };

    class TriangulationFunction
    {
    public:

        TriangulationFunction(CalibrationDataPtr calibration);

        template<typename T>
        bool operator()(const T* const circle, T* landmark) const;

    protected:

        CalibrationDataPtr myCalibration;
    };

    class ProjectionFunction
    {
    public:

        ProjectionFunction(CalibrationDataPtr calibration);

        template<typename T>
        bool operator()(const T* const landmark, T* observation) const;

    protected:

        CalibrationDataPtr myCalibration;
    };

    using CeresTriangulationFunction = ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>;
    using CeresProjectionFunction = ceres::AutoDiffCostFunction<ProjectionFunction, 3, 3>;

protected:

    void initialize(
        double timestamp,
        const std::vector<TrackedCircle>& circles);

    bool updateParticles(
        double timestamp,
        const std::vector<TrackedCircle>& circles);

    bool resampleParticles();

    bool triangulateLandmark(
        const Sophus::SE3d& camera_to_world,
        const cv::Vec3d& undistorted_circle, 
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    double computeObservationLikelihood(
        const Sophus::SE3d& camera_to_world,
        const cv::Vec3f& undistorted_circle,
        const Landmark& landmark);

    void updateLandmark(
        const Sophus::SE3d& camera_to_world,
        const cv::Vec3f& undistorted_circle,
        const Landmark& old_landmark,
        Landmark& new_landmark);

    void exportCurrentState(
        OdometryFrame& output,
        bool aligned_wrt_previous);

    void transformCameraToWorldFrame(
        const Eigen::Vector3d& position_in_camera,
        const Eigen::Matrix3d& covariance_in_camera,
        const Sophus::SE3d& camera_to_world,
        Eigen::Vector3d& position_in_world,
        Eigen::Matrix3d& covariance_in_world);

protected:

    CalibrationDataPtr myCalibration;

    double myPredictionLinearVelocitySigma;
    double myPredictionAngularVelocitySigma;

    double myCirclePositionNoise;
    double myCircleRadiusNoise;

    RandomEngine myRandomEngine;

    std::unique_ptr<CeresProjectionFunction> myProjectionFunction;
    std::unique_ptr<CeresTriangulationFunction> myTriangulationFunction;

    std::unique_ptr<State> myCurrentState;
    std::unique_ptr<State> myWorkingState;
};

