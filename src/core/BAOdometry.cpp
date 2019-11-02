#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "BAOdometry.h"
#include "SE3Parameterization.h"
#include "OdometryHelpers.h"

template<typename FramePtrContainer>
size_t BAOdometry::buildLocalMap(FramePtrContainer& frames, std::vector<LandmarkPtr>& local_map)
{
    for(FramePtr frame : frames)
    {
        for(Observation& obs : frame->observations)
        {
            obs.landmark->visited = false;
        }
    }

    size_t observation_count = 0;

    local_map.clear();

    for(FramePtr frame : frames)
    {
        for(Observation& obs : frame->observations)
        {
            observation_count++;

            if(obs.landmark->visited == false)
            {
                obs.landmark->visited = true;
                obs.landmark->local_id = local_map.size();
                local_map.push_back(obs.landmark);
            }
        }
    }

    return observation_count;
}

struct BAOdometry::BundleAdjustment
{
    BAOdometry* myParent;
    std::vector<FramePtr> myFrames;
    double mySigmaCenter;
    double mySigmaRadius;

    BundleAdjustment(BAOdometry* parent)
    {
        myParent = parent;
        mySigmaCenter = 1.0;
        mySigmaRadius = 3.0;
    }

    template<typename T>
    using Vector3 = Eigen::Matrix<T,3,1>;

    template<typename T>
    bool project(
        const T* camera_to_world_,
        const T* landmark_in_world_,
        T* projection) const
    {
        const Eigen::Map< const Sophus::SE3<T> > camera_to_world(camera_to_world_);
        const Eigen::Map< const Vector3<T> > landmark_in_world(landmark_in_world_);

        bool ok = true;

        const Vector3<T> landmark_in_camera = camera_to_world.inverse() * landmark_in_world;

        const T fx(myParent->myCalibration->cameras[0].calibration_matrix(0,0));
        const T fy(myParent->myCalibration->cameras[0].calibration_matrix(1,1));
        const T cx(myParent->myCalibration->cameras[0].calibration_matrix(0,2));
        const T cy(myParent->myCalibration->cameras[0].calibration_matrix(1,2));

        if( landmark_in_camera.z() < myParent->myCalibration->landmark_radius*0.1 )
        {
            ok = false;
        }
        else
        {
            const T distance = landmark_in_camera.norm();

            const T alpha = ceres::asin( T(myParent->myCalibration->landmark_radius) / distance );

            T center_los[2];
            center_los[0] = landmark_in_camera[0] / landmark_in_camera[2];
            center_los[1] = landmark_in_camera[1] / landmark_in_camera[2];

            T beta[2];
            beta[0] = ceres::atan( center_los[0] );
            beta[1] = ceres::atan( center_los[1] );

            T tangentlos0[2];
            tangentlos0[0] = ceres::tan( beta[0] - alpha );
            tangentlos0[1] = center_los[1];

            T tangentlos1[2];
            tangentlos1[0] = ceres::tan( beta[0] + alpha );
            tangentlos1[1] = center_los[1];

            T tangentlos2[2];
            tangentlos2[0] = center_los[0];
            tangentlos2[1] = ceres::tan( beta[1] - alpha );

            T tangentlos3[2];
            tangentlos3[0] = center_los[0];
            tangentlos3[1] = ceres::tan( beta[1] + alpha );


            T tanpoint0[2];
            tanpoint0[0] = fx * tangentlos0[0] + cx;
            tanpoint0[1] = fy * tangentlos0[1] + cy;

            T tanpoint1[2];
            tanpoint1[0] = fx * tangentlos1[0] + cx;
            tanpoint1[1] = fy * tangentlos1[1] + cy;

            T tanpoint2[2];
            tanpoint2[0] = fx * tangentlos2[0] + cx;
            tanpoint2[1] = fy * tangentlos2[1] + cy;

            T tanpoint3[2];
            tanpoint3[0] = fx * tangentlos3[0] + cx;
            tanpoint3[1] = fy * tangentlos3[1] + cy;


            T proj_x = ( tanpoint0[0] + tanpoint1[0] + tanpoint2[0] + tanpoint3[0] ) / 4.0;
            T proj_y = ( tanpoint0[1] + tanpoint1[1] + tanpoint2[1] + tanpoint3[1] ) / 4.0;
            T proj_radius = ( ceres::abs(tanpoint1[0] - tanpoint0[0]) + ceres::abs(tanpoint3[1] - tanpoint2[1]) ) / 4.0;

            projection[0] = proj_x;
            projection[1] = proj_y;
            projection[2] = proj_radius;

            ok = true;
        }

        return ok;
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {
        const size_t num_frames = myFrames.size();

        size_t frame_index = 0;
        size_t projection_index = 0;

        for(FramePtr f : myFrames)
        {
            const T* camera_to_world = parameters[frame_index];

            for(Observation& o : f->observations)
            {
                const T* landmark_in_world =  parameters[num_frames + o.landmark->local_id];

                T* projection_residual = residuals + 3*projection_index;

                T projection[3];

                if( project(camera_to_world, landmark_in_world, projection) )
                {
                    projection_residual[0] = (projection[0] - T(o.undistorted_circle[0])) / T(mySigmaCenter);
                    projection_residual[1] = (projection[1] - T(o.undistorted_circle[1])) / T(mySigmaCenter);
                    projection_residual[2] = (projection[2] - T(o.undistorted_circle[2])) / T(mySigmaRadius);
                }
                else
                {
                    projection_residual[0] = T(0.0);
                    projection_residual[1] = T(0.0);
                    projection_residual[2] = T(0.0);
                }

                projection_index++;
            }

            frame_index++;
        }

        return true;
    }
};

BAOdometry::BAOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;

    myMaxKeyFrames = 10;

    myKeyFrameSelectionTranslationThreshold = calibration->landmark_radius * 5.0;
    myKeyFrameSelectionRotationThreshold = 0.2*M_PI;
}

bool BAOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    OdometryFrame& output)
{
    const bool successful_alignment = track(timestamp, circles);

    if(successful_alignment == false)
    {
        initialize(timestamp, circles);
    }

    if(myFrames.empty())
    {
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }

    output.timestamp = timestamp;
    output.aligned_wrt_previous = successful_alignment;
    output.camera_to_world = myFrames.back()->camera_to_world;
    //output.pose_covariance.setIdentity(); // TODO set correct pose covariance.
    output.landmarks.clear();
    for(Observation& obs : myFrames.back()->observations)
    {
        if(obs.landmark)
        {
            output.landmarks.emplace_back();
            output.landmarks.back().position = obs.landmark->position;
        }
    }

    return true;
}

bool BAOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles)
{
    bool ret = false;

    if(myFrames.empty() == false)
    {
        FramePtr previous_frame = myFrames.back();

        FramePtr current_frame;

        if( previous_frame->keyframe )
        {
            current_frame = std::make_shared<Frame>();
            current_frame->camera_to_world = previous_frame->camera_to_world;
            current_frame->keyframe = false;

            myFrames.push_back(current_frame);

            if(myFrames.size() > myMaxKeyFrames)
            {
                myFrames.pop_front();
            }
        }
        else
        {
            current_frame = previous_frame;
        }

        current_frame->odometry_frame_id = previous_frame->odometry_frame_id+1;
        current_frame->timestamp = timestamp;

        std::vector<Observation> new_observations(circles.size());

        int num_tracked_landmarks = 0;
        int num_new_landmarks = 0;

        for(size_t i=0; i<circles.size(); i++)
        {
            new_observations[i].circle = circles[i].circle;
            new_observations[i].undistorted_circle = OdometryHelpers::undistortCircle(circles[i].circle, myCalibration);

            if( circles[i].has_previous && bool(previous_frame->observations[circles[i].previous].landmark) )
            {
                new_observations[i].landmark = previous_frame->observations[circles[i].previous].landmark;
                num_tracked_landmarks++;
            }
            else
            {
                new_observations[i].landmark = triangulateInitialLandmark(current_frame->camera_to_world, new_observations[i].undistorted_circle);

                if( new_observations[i].landmark )
                {
                    num_new_landmarks++;
                }
            }

        }

        current_frame->observations.swap(new_observations);

        if( num_tracked_landmarks >= 3 )
        {
            if(num_new_landmarks > 0)
            {
                ret = performBundleAdjustment(BA_LBA);
            }
            else
            {
                ret = performBundleAdjustment(BA_PNP);
            }

            if(ret)
            {
                FramePtr last_keyframe = myFrames[myFrames.size()-2];

                if(last_keyframe->keyframe == false)
                {
                    std::cerr << "Internal error!" << std::endl;
                    exit(1);
                }

                const Sophus::SE3d::Tangent distance = ( last_keyframe->camera_to_world.inverse() * current_frame->camera_to_world ).log();

                current_frame->keyframe = (distance.head<3>().norm() > myKeyFrameSelectionTranslationThreshold) || ( distance.tail<3>().norm() > myKeyFrameSelectionRotationThreshold );
            }
        }
    }

    if(ret == false)
    {
        reset();
    }

    return ret;
}

void BAOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    reset();

    FramePtr new_frame = std::make_shared<Frame>();

    new_frame->odometry_frame_id = 0;
    new_frame->timestamp = timestamp;
    new_frame->camera_to_world = Sophus::SE3d(); // identity.
    new_frame->keyframe = true;

    new_frame->observations.resize(circles.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        new_frame->observations[i].circle = circles[i].circle;
        new_frame->observations[i].undistorted_circle = OdometryHelpers::undistortCircle(circles[i].circle, myCalibration);
        new_frame->observations[i].landmark = triangulateInitialLandmark(new_frame->camera_to_world, new_frame->observations[i].undistorted_circle);
    }

    myFrames.assign({new_frame});

    // We dont check return value.
    // We assume nothing went wrong.
    performBundleAdjustment(BA_TRIANGULATION);
}

void BAOdometry::reset()
{
    myFrames.clear();
}

BAOdometry::LandmarkPtr BAOdometry::triangulateInitialLandmark(const Sophus::SE3d& camera_to_world, const cv::Vec3f& undistorted_circle)
{
    LandmarkPtr ret;

    Eigen::Vector3d position_in_camera;
    if( OdometryHelpers::triangulateLandmark(undistorted_circle, myCalibration, position_in_camera) )
    {
        ret = std::make_shared<Landmark>();
        ret->position = camera_to_world * position_in_camera;
    }

    return ret;
}

bool BAOdometry::performBundleAdjustment(BundleAdjustmentType type)
{
    bool ret = true;

    std::vector<FramePtr> frames;
    std::vector<LandmarkPtr> local_map;

    switch(type)
    {
    case BA_TRIANGULATION:
    case BA_PNP:
        frames.assign({ myFrames.back() });
        break;
    case BA_LBA:
        frames.assign(myFrames.begin(), myFrames.end());
        break;
    default:
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }

    const size_t num_observations = buildLocalMap(frames, local_map);

    if(num_observations == 0)
    {
        ret = true;
    }
    else
    {
        BundleAdjustment* cost0 = new BundleAdjustment(this);

        ceres::DynamicCostFunction* cost1 = new ceres::DynamicAutoDiffCostFunction<BundleAdjustment>(cost0);

        std::vector<double*> parameters;
        parameters.reserve(2*frames.size() + local_map.size());

        for(FramePtr frame : frames)
        {
            cost1->AddParameterBlock(Sophus::SE3d::num_parameters);
            parameters.push_back( frame->camera_to_world.data() );
        }

        for(size_t i=0; i<local_map.size(); i++)
        {
            cost1->AddParameterBlock(3);
            parameters.push_back( local_map[i]->position.data() );
        }

        cost1->SetNumResiduals(3*num_observations);

        ceres::LocalParameterization* se3_parameterization = new SE3Parameterization();

        ceres::Problem problem;
        problem.AddResidualBlock(cost1, nullptr, parameters);

        for(FramePtr frame : frames)
        {
            problem.AddParameterBlock(frame->camera_to_world.data(), Sophus::SE3d::num_parameters, se3_parameterization);
        }

        for(size_t i=0; i<local_map.size(); i++)
        {
            problem.AddParameterBlock(local_map[i]->position.data(), 3 );
        }

        // set fixed parameters.

        if(type == BA_LBA || type == BA_TRIANGULATION)
        {
            problem.SetParameterBlockConstant(frames.front()->camera_to_world.data());
        }
        else if(type == BA_PNP)
        {
            for(LandmarkPtr lm : local_map)
            {
                problem.SetParameterBlockConstant(lm->position.data());
            }
        }
        else
        {
            std::cerr << "Internal error" << std::endl;
            exit(1);
        }

        /*
        if(myFrames.size() >= 2)
        {
            Eigen::VectorXd resid(num_observations*3);
            const bool ok = cost0->operator()<double>(parameters.data(), resid.data());
            std::cout << ok << std::endl;
            std::cout << resid.format(Eigen::FullPrecision) << std::endl;
        }
        */

        cost0->myFrames.swap(frames);

        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        ceres::Solver solver;
        solver.Solve(options, &problem, &summary);

        ret = summary.IsSolutionUsable();
    }

    // std::cout << summary.BriefReport() << std::endl;
    // dump();

    return ret;
}

void BAOdometry::dump()
{
    std::vector<LandmarkPtr> landmarks;
    buildLocalMap(myFrames, landmarks);

    std::cout << "== BAOdometry dump ==" << std::endl;

    int i = 0;

    i = 0;
    for(FramePtr f : myFrames)
    {
        std::cout << "Frame " << i << std::endl;
        std::cout << "   camera_to_world_t = " << f->camera_to_world.translation().transpose() << std::endl;
        std::cout << "   camera_to_world_q = " << f->camera_to_world.unit_quaternion().coeffs().transpose() << std::endl;
        i++;
    }

    i = 0;
    for(LandmarkPtr lm : landmarks)
    {
        std::cout << "Landmark " << i << std::endl;
        std::cout << "   position = " << lm->position.transpose() << std::endl;
        i++;
    }
}

