#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "BAOdometry.h"

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
    bool project(
        const T* camera_to_world_t,
        const T* camera_to_world_q,
        const T* landmark_in_world,
        T* projection) const
    {
        bool ok = true;

        T world_to_camera_q [4];
        world_to_camera_q[0] = camera_to_world_q[6];
        world_to_camera_q[1] = -camera_to_world_q[3];
        world_to_camera_q[2] = -camera_to_world_q[4];
        world_to_camera_q[3] = -camera_to_world_q[5];

        T tmp[3];
        tmp[0] = landmark_in_world[0] - camera_to_world_t[0];
        tmp[1] = landmark_in_world[1] - camera_to_world_t[1];
        tmp[2] = landmark_in_world[2] - camera_to_world_t[2];

        T landmark_in_camera[3];
        ceres::QuaternionRotatePoint(world_to_camera_q, tmp, landmark_in_camera);

        T fx(myParent->myCalibration->cameras[0].calibration_matrix(0,0));
        T fy(myParent->myCalibration->cameras[0].calibration_matrix(1,1));
        T cx(myParent->myCalibration->cameras[0].calibration_matrix(0,2));
        T cy(myParent->myCalibration->cameras[0].calibration_matrix(1,2));

        if( landmark_in_camera[2] < myParent->myLandmarkRadius*0.1 )
        {
            ok = false;
        }
        else
        {
            T distance = ceres::sqrt(
                landmark_in_camera[0]*landmark_in_camera[0] +
                landmark_in_camera[1]*landmark_in_camera[1] +
                landmark_in_camera[2]*landmark_in_camera[2] );

            T alpha = ceres::asin( T(myParent->myLandmarkRadius) / distance );

            T center_los[2];
            center_los[0] = landmark_in_camera[0] / landmark_in_camera[2];
            center_los[1] = landmark_in_camera[1] / landmark_in_camera[2];

            T beta[2];
            beta[0] = ceres::acos( center_los[0] );
            beta[1] = ceres::acos( center_los[1] );


            T tangentlos0[2];
            tangentlos0[0] = ceres::cos( beta[0] - alpha );
            tangentlos0[1] = center_los[1];

            T tangentlos1[2];
            tangentlos1[0] = ceres::cos( beta[0] + alpha );
            tangentlos1[1] = center_los[1];

            T tangentlos2[2];
            tangentlos2[0] = center_los[0];
            tangentlos2[1] = ceres::cos( beta[1] - alpha );

            T tangentlos3[2];
            tangentlos3[0] = center_los[0];
            tangentlos3[1] = ceres::cos( beta[1] + alpha );


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
            for(Observation& o : f->observations)
            {
                const size_t landmark_index = o.landmark->local_id;

                const T* camera_to_world_t = parameters[2*frame_index+0];
                const T* camera_to_world_q = parameters[2*frame_index+1];
                const T* landmark_in_world = parameters[2*num_frames + landmark_index];

                T projection[3];

                T* landmark_residual = residuals + 3*projection_index;


                if( project(camera_to_world_t, camera_to_world_q, landmark_in_world, projection) )
                {
                    landmark_residual[0] = (projection[0] - T(o.circle[0])) / T(mySigmaCenter);
                    landmark_residual[1] = (projection[1] - T(o.circle[1])) / T(mySigmaCenter);
                    landmark_residual[2] = (projection[2] - T(o.circle[2])) / T(mySigmaRadius);
                }
                else
                {
                    landmark_residual[0] = T(0.0);
                    landmark_residual[1] = T(0.0);
                    landmark_residual[2] = T(0.0);
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
    myMaxProjections = 50;
}

bool BAOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    OdometryFrame& output)
{
    bool successful_alignment = false;
    bool ret = true;

    successful_alignment = track(timestamp, circles);

    if(successful_alignment == false)
    {
        initialize(timestamp, circles);
    }

    if(myLastFrame)
    {
        output.timestamp = timestamp;
        output.aligned_wrt_previous = successful_alignment;
        output.camera_to_world = myLastFrame->camera_to_world;
        //output.landmarks.resize(myLocalMap.size())
        output.pose_covariance.setIdentity(); // TODO set corrent pose covariance.
        output.landmarks.clear(); // TODO: export landmarks!
        ret = true;
    }
    else
    {
        output.timestamp = timestamp;
        output.aligned_wrt_previous = false;
        output.camera_to_world = Sophus::SE3d();
        output.landmarks.clear();
        ret = false;
    }

    return ret;

    /*
    create a new frame and fill it with observations.
    update observedlandmarks array.
    if first frame
        keep as keyframe
        bundle adjustment
    else
        new_keyframe = there is new landmark || previous keyframe travalled too much
        if new_keyframe
            bundle adjustment
            keep as keyframe
        else
            perspective-n-Points
            new_keyframe_requested = travelled 
        endif
    endif
    */
}

bool BAOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles)
{
    bool ret = false;

    if(myLastFrame)
    {
        if(myKeyFrames.empty())
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        FramePtr newframe = std::make_shared<Frame>();
        newframe->timestamp = timestamp;
        newframe->camera_to_world = myLastFrame->camera_to_world;

        const Sophus::SE3d::Tangent distance = ( myLastFrame->camera_to_world.inverse() * myKeyFrames.back()->camera_to_world ).log();

        bool iskeyframe = false;

        if( iskeyframe )
        {
            myKeyFrames.push_back(newframe);
        }
        else
        {
        }

        myLastFrame = newframe;
        ret = true;
    }

    return ret;
}

void BAOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    reset();

    FramePtr new_frame = std::make_shared<Frame>();
    myKeyFrames.push_back(new_frame);

    new_frame->timestamp = timestamp;
    new_frame->camera_to_world = Sophus::SE3d(); // identity.

    myObservedLandmarks.resize(circles.size());
    for(size_t i=0; i<circles.size(); i++)
    {
        myObservedLandmarks[i] = triangulateInitialLandmark(circles[i].circle);

        if(myObservedLandmarks[i])
        {
            new_frame->observations.emplace_back();
            new_frame->observations.back().landmark = myObservedLandmarks[i];
            new_frame->observations.back().circle = circles[i].circle;
        }
    }

    performBundleAdjustment(BA_TRIANGULATION);
}

void BAOdometry::reset()
{
    myObservedLandmarks.clear();
    myKeyFrames.clear();
}

BAOdometry::LandmarkPtr BAOdometry::triangulateInitialLandmark(const cv::Vec3f& circle)
{
    LandmarkPtr ret;

    const cv::Vec3f undistorted = undistortCircle(circle);

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
    }

    return ret;
}

void BAOdometry::performBundleAdjustment(BundleAdjustmentType type)
{
    auto fn = new BundleAdjustment(this);
    auto wrapper = new ceres::DynamicAutoDiffCostFunction<BundleAdjustment>(fn);

    std::vector<FramePtr>& frames = fn->myFrames;
    size_t num_observations = 0;

    switch(type)
    {
    case BA_TRIANGULATION:
    case BA_PNP:
        //frames.push_back( myKeyFrames.back() );
        frames.push_back( myLastFrame );
        break;
    case BA_LBA:
        frames.reserve( myKeyFrames.size() );
        std::copy( myKeyFrames.begin(), myKeyFrames.end(), std::back_inserter(frames) );
        break;
    default:
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }

    myLocalMap.clear();

    for(FramePtr frame : frames)
    {
        for(Observation& o : frame->observations)
        {
            o.landmark->visited = false;
        }
    }

    for(FramePtr frame : frames)
    {
        for(Observation& o : frame->observations)
        {
            num_observations++;

            if(o.landmark->visited == false)
            {
                o.landmark->visited = true;
                myLocalMap.push_back(o.landmark);
            }
        }
    }

    for(FramePtr frame : frames)
    {
        wrapper->AddParameterBlock(3);
        wrapper->AddParameterBlock(4);
    }

    for(size_t i=0; i<myLocalMap.size(); i++)
    {
        myLocalMap[i]->local_id = i;
        wrapper->AddParameterBlock(3);
    }

    wrapper->SetNumResiduals(3*num_observations);

    // TODO check who takes ownership of this object.
    auto quaternion_parameterization = new ceres::QuaternionParameterization();

    ceres::Problem problem;

    bool first = true;
    for(FramePtr frame : frames)
    {
        frame->preBundleAdjustment();
        problem.AddParameterBlock(frame->ceres_camera_to_world_t, 3);
        problem.AddParameterBlock(frame->ceres_camera_to_world_q, 4, quaternion_parameterization);

        if(first)
        {
            problem.SetParameterBlockConstant(frame->ceres_camera_to_world_t);
            problem.SetParameterBlockConstant(frame->ceres_camera_to_world_q);
            first = false;
        }
    }

    for(size_t i=0; i<myLocalMap.size(); i++)
    {
        myLocalMap[i]->preBundleAdjustment();
        problem.AddParameterBlock( myLocalMap[i]->ceres_position, 3 );
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    ceres::Solver solver;
    solver.Solve(options, &problem, &summary);

    for(FramePtr frame : frames)
    {
        frame->postBundleAdjustment();
    }

    for(size_t i=0; i<myLocalMap.size(); i++)
    {
        myLocalMap[i]->postBundleAdjustment();
    }
}

void BAOdometry::Frame::preBundleAdjustment()
{
    std::copy( camera_to_world.data()+4, camera_to_world.data()+7, ceres_camera_to_world_t );
    std::copy( camera_to_world.data(), camera_to_world.data()+4, ceres_camera_to_world_q );
    std::swap( ceres_camera_to_world_q[0], ceres_camera_to_world_q[3] );
}

void BAOdometry::Frame::postBundleAdjustment()
{
    std::copy( ceres_camera_to_world_t, ceres_camera_to_world_t+3, camera_to_world.data()+4 );
    std::swap( ceres_camera_to_world_q[0], ceres_camera_to_world_q[3] );
    std::copy( ceres_camera_to_world_q, ceres_camera_to_world_q+4, camera_to_world.data() );
    camera_to_world.normalize();
}

void BAOdometry::Landmark::preBundleAdjustment()
{
    std::copy( position.data(), position.data()+3, ceres_position );
}

void BAOdometry::Landmark::postBundleAdjustment()
{
    std::copy( ceres_position, ceres_position+3, position.data() );
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
