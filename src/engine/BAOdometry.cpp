#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "BAOdometry.h"
#include "SE3Parameterization.h"

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
        Eigen::Map< const Sophus::SE3<T> > camera_to_world(camera_to_world_);
        Eigen::Map< const Vector3<T> > landmark_in_world(landmark_in_world_);

        bool ok = true;

        Vector3<T> landmark_in_camera = camera_to_world.inverse() * landmark_in_world;

        const T fx(myParent->myCalibration->cameras[0].calibration_matrix(0,0));
        const T fy(myParent->myCalibration->cameras[0].calibration_matrix(1,1));
        const T cx(myParent->myCalibration->cameras[0].calibration_matrix(0,2));
        const T cy(myParent->myCalibration->cameras[0].calibration_matrix(1,2));

        if( landmark_in_camera.z() < myParent->myLandmarkRadius*0.1 )
        {
            ok = false;
        }
        else
        {
            const T distance = landmark_in_camera.norm();

            const T alpha = ceres::asin( T(myParent->myLandmarkRadius) / distance );

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
    myLandmarkRadius = 1.0;
    myMaxKeyFrames = 10;
}

bool BAOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    OdometryFrame& output)
{
    bool successful_alignment = track(timestamp, circles);

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
    //output.landmarks.resize(myLocalMap.size())
    output.pose_covariance.setIdentity(); // TODO set corrent pose covariance.
    output.landmarks.clear(); // TODO: export landmarks!
    {
        std::vector<LandmarkPtr> landmarks;
        buildLocalMap(myFrames, landmarks);

        output.landmarks.resize(landmarks.size());
        for(size_t i=0; i<landmarks.size(); i++)
        {
            output.landmarks[i].position = landmarks[i]->position;
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

        current_frame->timestamp = timestamp;

        std::vector<Observation> new_observations(circles.size());

        int num_tracked_landmarks = 0;
        int num_new_landmarks = 0;

        for(size_t i=0; i<circles.size(); i++)
        {
            new_observations[i].circle = circles[i].circle;
            new_observations[i].undistorted_circle = undistortCircle(circles[i].circle);

            if( circles[i].has_previous && bool(previous_frame->observations[i].landmark) )
            {
                new_observations[i].landmark = previous_frame->observations[i].landmark;
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
                performBundleAdjustment(BA_LBA);
            }
            else
            {
                performBundleAdjustment(BA_PNP);
            }

            FramePtr last_keyframe = myFrames[myFrames.size()-2];
            if(last_keyframe->keyframe == false)
            {
                std::cerr << "Internal error!" << std::endl;
                exit(1);
            }

            const Sophus::SE3d::Tangent distance = ( last_keyframe->camera_to_world.inverse() * current_frame->camera_to_world ).log();

            current_frame->keyframe = (distance.head<3>().norm() > myLandmarkRadius*2.0) || ( distance.tail<3>().norm() > M_PI*0.2 );

            ret = true;
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

    new_frame->timestamp = timestamp;
    new_frame->camera_to_world = Sophus::SE3d(); // identity.
    new_frame->keyframe = true;

    new_frame->observations.resize(circles.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        new_frame->observations[i].circle = circles[i].circle;
        new_frame->observations[i].undistorted_circle = undistortCircle(circles[i].circle);
        new_frame->observations[i].landmark = triangulateInitialLandmark(new_frame->camera_to_world, new_frame->observations[i].undistorted_circle);
    }

    myFrames.assign({new_frame});

    performBundleAdjustment(BA_TRIANGULATION);
}

void BAOdometry::reset()
{
    myFrames.clear();
}

BAOdometry::LandmarkPtr BAOdometry::triangulateInitialLandmark(const Sophus::SE3d& camera_to_world, const cv::Vec3f& undistorted_circle)
{
    LandmarkPtr ret;

    //const cv::Vec3f undistorted = undistortCircle(circle);
    const cv::Vec3f undistorted = undistorted_circle;

    const double cx = undistorted[0];
    const double cy = undistorted[1];
    const double r = undistorted[2];

    const double IK00 = myCalibration->cameras[0].inverse_calibration_matrix(0,0);
    const double IK02 = myCalibration->cameras[0].inverse_calibration_matrix(0,2);
    const double IK11 = myCalibration->cameras[0].inverse_calibration_matrix(1,1);
    const double IK12 = myCalibration->cameras[0].inverse_calibration_matrix(1,2);

    const double los_cx = IK00*cx + IK02;
    const double los_cy = IK11*cy + IK12;
    const double los_cxminus = IK00*(cx-r) + IK02;
    const double los_cxplus = IK00*(cx+r) + IK02;
    const double los_cyminus = IK11*(cy-r) + IK12;
    const double los_cyplus = IK11*(cy+r) + IK12;

    const double alpha_xminus = ceres::atan(los_cxminus);
    const double alpha_xplus = ceres::atan(los_cxplus);
    const double alpha_yminus = ceres::atan(los_cyminus);
    const double alpha_yplus = ceres::atan(los_cyplus);

    const double los_dirx = ceres::tan( (alpha_xminus + alpha_xplus) / 2.0 );
    const double los_diry = ceres::tan( (alpha_yminus + alpha_yplus) / 2.0 );

    const double beta = ( (alpha_xplus - alpha_xminus)/2.0 + (alpha_yplus - alpha_yminus)/2.0 ) / 2.0;

    if( M_PI*0.3/180.0 < beta && beta < M_PI*150.0/180.0 )
    {
        const double distance = myLandmarkRadius/ceres::sin(beta);

        double dir[3];
        dir[0] = los_dirx;
        dir[1] = los_diry;
        dir[2] = 1.0;

        const double norm = ceres::sqrt( dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2] );

        ret.reset(new Landmark());
        ret->position.x() = distance*dir[0]/norm;
        ret->position.y() = distance*dir[1]/norm;
        ret->position.z() = distance*dir[2]/norm;

        ret->position = camera_to_world * ret->position;
    }

    return ret;
}

void BAOdometry::performBundleAdjustment(BundleAdjustmentType type)
{
    auto cost0 = new BundleAdjustment(this);

    auto cost1 = new ceres::DynamicAutoDiffCostFunction<BundleAdjustment>(cost0);

    std::vector<FramePtr>& frames = cost0->myFrames;
    size_t num_observations = 0;

    std::vector<LandmarkPtr> local_map;

    std::vector<double*> parameters;

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

    num_observations = buildLocalMap(frames, local_map);

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

    //ceres::EigenQuaternionParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization();
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

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    ceres::Solver solver;
    solver.Solve(options, &problem, &summary);
}

cv::Vec3f BAOdometry::undistortCircle(const cv::Vec3f& c)
{
    std::vector<cv::Vec2d> distorted(4);
    std::vector<cv::Vec2d> undistorted(4);

    // require newer version of OpenCV.
    //std::array< cv::Vec2d, 4 > distorted;
    //std::array< cv::Vec2d, 4 > undistorted;

    distorted[0][0] = c[0]+c[2];
    distorted[0][1] = c[1];
    distorted[1][0] = c[0]-c[2];
    distorted[1][1] = c[1];
    distorted[2][0] = c[0];
    distorted[2][1] = c[1]+c[2];
    distorted[3][0] = c[0];
    distorted[3][1] = c[1]-c[2];

    cv::undistortPoints(
        distorted,
        undistorted,
        myCalibration->cameras[0].calibration_matrix,
        myCalibration->cameras[0].distortion_coefficients,
        cv::noArray(),
        myCalibration->cameras[0].calibration_matrix);

    const cv::Vec2d center = 0.25f * ( undistorted[0] + undistorted[1] + undistorted[2] + undistorted[3] );

    const double l0 = cv::norm(center, undistorted[0]);
    const double l1 = cv::norm(center, undistorted[1]);
    const double l2 = cv::norm(center, undistorted[2]);
    const double l3 = cv::norm(center, undistorted[3]);

    cv::Vec3f ret;
    ret[0] = center[0];
    ret[1] = center[1];
    ret[2] = ( l0+l1+l2+l3 ) / 4.0;

    return ret;
}
