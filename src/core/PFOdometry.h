
#pragma once

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

    struct Landmark
    {
        size_t tracked_index;
        Eigen::Vector3d position;
        Eigen::Matrix3d covariance;
    };

    struct Particle
    {
        Sophus::SE3d camera_to_world;
        std::vector<Landmark> landmarks;
    };

    struct State
    {
        size_t frame_id;
        double timestamp;
        std::vector<Particle> particles;

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

    bool triangulateLandmarkInWorldFrame(
        const cv::Vec3f& circle, 
        const Sophus::SE3d& camera_to_world,
        Eigen::Vector3d& position,
        Eigen::Matrix3d& covariance);

    bool trackAndMap(double timestamp, const std::vector<TrackedCircle>& circles);

protected:

    CalibrationDataPtr myCalibration;
    size_t myNumParticles;
    std::unique_ptr<State> myCurrentState;
    std::unique_ptr<State> myWorkingState;
};

