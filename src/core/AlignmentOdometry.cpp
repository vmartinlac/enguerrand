#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "AlignmentOdometry.h"
#include "SE3Parameterization.h"
#include "OdometryHelpers.h"

struct AlignmentOdometry::Alignment
{
    FramePtr myFrame;

    Alignment(FramePtr frame)
    {
        myFrame = std::move(frame);
    }

    template<typename T>
    using Vector3 = Eigen::Matrix<T,3,1>;

    template<typename T>
    bool operator()(const T* const camera_to_world_, T* residuals) const
    {
        Eigen::Map< const Sophus::SE3<T> > camera_to_world(camera_to_world_);

        T* residual = residuals;

        size_t index = 0;

        for(Observation& o : myFrame->observations)
        {
            if(o.valid)
            {
                const Vector3<T> position_in_world = o.position_in_world.cast<T>();
                const Vector3<T> position_in_camera = o.position_in_camera.cast<T>();

                Eigen::Map< Vector3<T> >(residuals + 3*index) = position_in_world - camera_to_world * position_in_camera;

                index++;
            }
        }

        return true;
    }
};

AlignmentOdometry::AlignmentOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
    myLandmarkRadius = 1.0;
}

bool AlignmentOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    OdometryFrame& output)
{
    const bool successful_alignment = track(timestamp, circles);

    if(successful_alignment == false)
    {
        initialize(timestamp, circles);
    }

    if(!myLastFrame)
    {
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }

    output.timestamp = timestamp;
    output.aligned_wrt_previous = successful_alignment;
    output.camera_to_world = myLastFrame->camera_to_world;
    output.landmarks.clear();
    for(Observation& o : myLastFrame->observations)
    {
        if(o.valid)
        {
            output.landmarks.emplace_back();
            output.landmarks.back().position = o.position_in_world;
        }
    }

    return true;
}

bool AlignmentOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles)
{
    bool ret = false;

    if(myLastFrame)
    {
        FramePtr new_frame = std::make_shared<Frame>();
        new_frame->odometry_frame_id = myLastFrame->odometry_frame_id+1;
        new_frame->timestamp = timestamp;
        new_frame->camera_to_world = myLastFrame->camera_to_world;
        new_frame->observations.resize(circles.size());

        for(size_t i=0; i<circles.size(); i++)
        {
            new_frame->observations[i].valid = false;

            if( circles[i].has_previous && myLastFrame->observations[circles[i].previous].valid )
            {
                const cv::Vec3f undistorted = OdometryHelpers::undistortCircle(circles[i].circle, myCalibration);
                Eigen::Vector3d position_in_camera;
                if( OdometryHelpers::triangulateLandmark(undistorted, myCalibration, myLandmarkRadius, position_in_camera) )
                {
                    new_frame->observations[i].position_in_world = myLastFrame->observations[circles[i].previous].position_in_world;
                    new_frame->observations[i].position_in_camera = position_in_camera;
                    new_frame->observations[i].valid = true;
                }
            }

            if( new_frame->observations[i].valid == false )
            {
                new_frame->observations[i].position_in_world.setZero();
                new_frame->observations[i].position_in_camera.setZero();
            }
        }

        if( performAlignment(new_frame) )
        {
            for(size_t i=0; i<circles.size(); i++)
            {
                if( new_frame->observations[i].valid == false )
                {
                    const cv::Vec3f undistorted = OdometryHelpers::undistortCircle(circles[i].circle, myCalibration);
                    Eigen::Vector3d position_in_camera;
                    if( OdometryHelpers::triangulateLandmark(undistorted, myCalibration, myLandmarkRadius, position_in_camera) )
                    {
                        new_frame->observations[i].position_in_world = new_frame->camera_to_world * position_in_camera;
                        new_frame->observations[i].position_in_camera = position_in_camera;
                        new_frame->observations[i].valid = true;
                    }
                }
            }

            ret = true;
            myLastFrame = std::move(new_frame);
        }
    }

    if(ret == false)
    {
        reset();
    }

    return ret;
}

void AlignmentOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    reset();

    FramePtr new_frame = std::make_shared<Frame>();

    new_frame->odometry_frame_id = 0;
    new_frame->timestamp = timestamp;
    new_frame->camera_to_world = Sophus::SE3d(); // identity.
    new_frame->observations.resize(circles.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        Observation& obs = new_frame->observations[i];

        const cv::Vec3f undistorted_circle = OdometryHelpers::undistortCircle( circles[i].circle, myCalibration );

        Eigen::Vector3d position_in_camera;
        obs.valid = OdometryHelpers::triangulateLandmark(undistorted_circle, myCalibration, myLandmarkRadius, position_in_camera);

        if(obs.valid)
        {
            obs.position_in_camera = position_in_camera;
            obs.position_in_world = position_in_camera;
        }
        else
        {
            obs.position_in_world.setZero();
            obs.position_in_camera.setZero();
        }
    }

    myLastFrame = new_frame;
}

void AlignmentOdometry::reset()
{
    myLastFrame.reset();
}

bool AlignmentOdometry::performAlignment(FramePtr frame)
{
    bool ret = false;

    size_t num_observations = 0;

    for(Observation& o : frame->observations)
    {
        if(o.valid)
        {
            num_observations++;
        }
    }

    if(num_observations >= 3)
    {
        Alignment* cost0 = new Alignment(frame);

        ceres::CostFunction* cost1 = new ceres::AutoDiffCostFunction<Alignment, ceres::DYNAMIC, Sophus::SE3d::num_parameters>(cost0, num_observations*3);

        ceres::LocalParameterization* se3_parameterization = new SE3Parameterization();

        ceres::Problem problem;
        problem.AddResidualBlock(cost1, nullptr, frame->camera_to_world.data());

        problem.AddParameterBlock(frame->camera_to_world.data(), Sophus::SE3d::num_parameters, se3_parameterization);

        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        ceres::Solver solver;
        solver.Solve(options, &problem, &summary);

        ret = summary.IsSolutionUsable();
    }

    return ret;
}

