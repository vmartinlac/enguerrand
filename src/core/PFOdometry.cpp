#include "OdometryHelpers.h"
#include "PFOdometry.h"

struct PFOdometry::TriangulationFunction
{

    TriangulationFunction(CalibrationDataPtr calibration, const Sophus::SE3d& camera_to_world)
    {
        myCalibration = calibration;
        myCameraToWorld = camera_to_world;
    }

    template<typename T>
    bool operator()(const T* const circle, T* landmark) const
    {
        T cx = circle[0];
        T cy = circle[1];
        T r = circle[2];

        const double IK00 = myCalibration->cameras[0].inverse_calibration_matrix(0,0);
        const double IK02 = myCalibration->cameras[0].inverse_calibration_matrix(0,2);
        const double IK11 = myCalibration->cameras[0].inverse_calibration_matrix(1,1);
        const double IK12 = myCalibration->cameras[0].inverse_calibration_matrix(1,2);

        T los_cx = IK00*cx + IK02;
        T los_cy = IK11*cy + IK12;
        T los_cxminus = IK00*(cx-r) + IK02;
        T los_cxplus = IK00*(cx+r) + IK02;
        T los_cyminus = IK11*(cy-r) + IK12;
        T los_cyplus = IK11*(cy+r) + IK12;

        T alpha_xminus = ceres::atan(los_cxminus);
        T alpha_xplus = ceres::atan(los_cxplus);
        T alpha_yminus = ceres::atan(los_cyminus);
        T alpha_yplus = ceres::atan(los_cyplus);

        T los_dirx = ceres::tan( (alpha_xminus + alpha_xplus) / 2.0 );
        T los_diry = ceres::tan( (alpha_yminus + alpha_yplus) / 2.0 );

        T beta = ( (alpha_xplus - alpha_xminus)/2.0 + (alpha_yplus - alpha_yminus)/2.0 ) / 2.0;

        if( M_PI*0.3/180.0 < beta && beta < M_PI*150.0/180.0)
        {
            T distance = myCalibration->landmark_radius / ceres::sin(beta);

            T dir[3];
            dir[0] = los_dirx;
            dir[1] = los_diry;
            dir[2] = T(1.0);

            T norm = ceres::sqrt( dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2] );

            Eigen::Matrix<T, 3, 1> landmark_in_camera;
            landmark_in_camera.x() = distance*dir[0]/norm;
            landmark_in_camera.y() = distance*dir[1]/norm;
            landmark_in_camera.z() = distance*dir[2]/norm;

            const Eigen::Matrix<T, 3, 1> landmark_in_world = myCameraToWorld.cast<T>() * landmark_in_camera;

            landmark[0] = landmark_in_world.x();
            landmark[1] = landmark_in_world.y();
            landmark[2] = landmark_in_world.z();

            return true;
        }
        else
        {
            return false;
        }
    }

    Sophus::SE3d myCameraToWorld;
    CalibrationDataPtr myCalibration;
};

PFOdometry::PFOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
    myNumParticles = 1000;
}

bool PFOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles, OdometryFrame& output)
{
    const bool successful_tracking = trackAndMap(timestamp, circles);

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    myCurrentState->save(output, successful_tracking);

    return true;
}

void PFOdometry::State::save(OdometryFrame& output, bool aligned_wrt_previous)
{
    output.timestamp = timestamp;
    output.aligned_wrt_previous = aligned_wrt_previous;
    output.camera_to_world = Sophus::SE3d(); // TODO set average pose.
    output.landmarks.clear(); // TODO set average landmark positions.
}

void PFOdometry::reset()
{
    myCurrentState.reset();
    myWorkingState.reset();
}

void PFOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    myCurrentState.reset(new State());
    myWorkingState.reset(new State());

    myCurrentState->particles.resize(myNumParticles);
    myWorkingState->particles.resize(myNumParticles);
}

bool PFOdometry::trackAndMap(double timestamp, const std::vector<TrackedCircle>& circles)
{
    bool ret = bool(myCurrentState) && bool(myWorkingState);

    if(ret)
    {
        ret = false; //TODO
    }

    return ret;
}

bool PFOdometry::triangulateLandmarkInWorldFrame(
    const cv::Vec3f& circle, 
    const Sophus::SE3d& camera_to_world,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    const cv::Vec3d undistorted_circle = OdometryHelpers::undistortCircle(circle, myCalibration);

    Eigen::Matrix<double,3,3,Eigen::RowMajor> jacobian;

    const double* ceres_variable_ptr = undistorted_circle.val;
    double* ceres_jacobian_ptr = jacobian.data();

    TriangulationFunction* fn0 = new TriangulationFunction(myCalibration, camera_to_world);

    std::unique_ptr<CeresTriangulationFunction> fn1(new CeresTriangulationFunction(fn0));

    const bool ok = fn1->Evaluate( &ceres_variable_ptr, position.data(), &ceres_jacobian_ptr );

    if(ok)
    {
        const double sigma_center = 1.5;
        const double sigma_radius = 1.5;

        Eigen::Matrix3d S0;
        S0 <<
            sigma_center*sigma_center, 0.0, 0.0,
            0.0, sigma_center*sigma_center, 0.0,
            0.0, 0.0, sigma_radius*sigma_radius;

        covariance = jacobian * S0 * jacobian.transpose();

        //std::cout << sqrt(covariance(0,0)) << std::endl;
        //std::cout << sqrt(covariance(1,1)) << std::endl;
        //std::cout << sqrt(covariance(2,2)) << std::endl;
    }

    return ok;
}

