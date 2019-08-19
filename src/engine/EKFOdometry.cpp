#include <fstream>
#include <iomanip>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/rotation.h>
#include "EKFOdometry.h"

struct EKFOdometry::TriangulationFunction
{
    EKFOdometry* mParent;

    TriangulationFunction(EKFOdometry* parent)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(const T* const circle, T* landmark) const
    {
        T cx = circle[0];
        T cy = circle[1];
        T r = circle[2];

        const double IK00 = mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,0);
        const double IK02 = mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,2);
        const double IK11 = mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,1);
        const double IK12 = mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,2);

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
            T distance = mParent->mLandmarkRadius/ceres::sin(beta);

            T dir[3];
            dir[0] = los_dirx;
            dir[1] = los_diry;
            dir[2] = T(1.0);

            T norm = ceres::sqrt( dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2] );

            landmark[0] = distance*dir[0]/norm;
            landmark[1] = distance*dir[1]/norm;
            landmark[2] = distance*dir[2]/norm;

            return true;
        }
        else
        {
            return false;
        }
    }
};

struct EKFOdometry::PredictionFunction
{
    EKFOdometry* mParent;
    double mTimestep;
    size_t mNumLandmarks;

    PredictionFunction(double dt, size_t num_landmarks, EKFOdometry* parent)
    {
        mParent = parent;
        mTimestep = dt;
        mNumLandmarks = num_landmarks;
    }

    template<typename T>
    bool operator()(T const* const* old_state_arr, T* new_state) const
    {
        const T* old_state = *old_state_arr;

        // convert linear momentum to linear velocity. Assumes mass is one.
        T linear_velocity[3];
        linear_velocity[0] = old_state[7];
        linear_velocity[1] = old_state[8];
        linear_velocity[2] = old_state[9];

        // convert angular momentum to angular velocity. Assumes inertia matrix is identity.
        T angular_velocity[3];
        angular_velocity[0] = old_state[10];
        angular_velocity[1] = old_state[11];
        angular_velocity[2] = old_state[12];

        T old_camera_to_world[4];
        old_camera_to_world[0] = old_state[6];
        old_camera_to_world[1] = old_state[3];
        old_camera_to_world[2] = old_state[4];
        old_camera_to_world[3] = old_state[5];

        T angle_axis[3];
        angle_axis[0] = mTimestep*angular_velocity[0];
        angle_axis[1] = mTimestep*angular_velocity[1];
        angle_axis[2] = mTimestep*angular_velocity[2];

        T new_camera_to_old_camera[4];
        ceres::AngleAxisToQuaternion(angle_axis, new_camera_to_old_camera);

        T new_camera_to_world[4];
        ceres::QuaternionProduct(old_camera_to_world, new_camera_to_old_camera, new_camera_to_world);
        T cte = ceres::sqrt(
            new_camera_to_world[0]*new_camera_to_world[0] +
            new_camera_to_world[1]*new_camera_to_world[1] +
            new_camera_to_world[2]*new_camera_to_world[2] +
            new_camera_to_world[3]*new_camera_to_world[3] );
        new_camera_to_world[0] /= cte;
        new_camera_to_world[1] /= cte;
        new_camera_to_world[2] /= cte;
        new_camera_to_world[3] /= cte;

        // update position.

        new_state[0] = old_state[0] + mTimestep*linear_velocity[0];
        new_state[1] = old_state[1] + mTimestep*linear_velocity[1];
        new_state[2] = old_state[2] + mTimestep*linear_velocity[2];

        // update attitude.

        new_state[3] = new_camera_to_world[1];
        new_state[4] = new_camera_to_world[2];
        new_state[5] = new_camera_to_world[3];
        new_state[6] = new_camera_to_world[0];

        // copy linear and angular momenta.

        new_state[7] = old_state[7];
        new_state[8] = old_state[8];
        new_state[9] = old_state[9];

        new_state[10] = old_state[10];
        new_state[11] = old_state[11];
        new_state[12] = old_state[12];

        // copy landmarks.

        for(size_t i=0; i<mNumLandmarks; i++)
        {
            new_state[13+3*i+0] = old_state[13+3*i+0];
            new_state[13+3*i+1] = old_state[13+3*i+1];
            new_state[13+3*i+2] = old_state[13+3*i+2];
        }

        return true;
    }
};

struct EKFOdometry::ObservationFunction
{
    EKFOdometry* mParent;
    std::vector<ObservedLandmark>& mVisibleLandmarks;

    ObservationFunction(std::vector<ObservedLandmark>& visible_landmarks, EKFOdometry* parent) :
        mVisibleLandmarks(visible_landmarks)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(T const* const* state_arr, T* prediction) const
    {
        const T* state = *state_arr;

        bool ok = true;

        T camera_to_world_t[3];
        camera_to_world_t[0] = state[0];
        camera_to_world_t[1] = state[1];
        camera_to_world_t[2] = state[2];

        T world_to_camera_q [4];
        world_to_camera_q[0] = state[6];
        world_to_camera_q[1] = -state[3];
        world_to_camera_q[2] = -state[4];
        world_to_camera_q[3] = -state[5];

        for(size_t i=0; ok && i<mVisibleLandmarks.size(); i++)
        {
            const size_t j = mVisibleLandmarks[i].landmark;

            T landmark_in_world[3];
            landmark_in_world[0] = state[13+3*j+0];
            landmark_in_world[1] = state[13+3*j+1];
            landmark_in_world[2] = state[13+3*j+2];

            T tmp[3];
            tmp[0] = landmark_in_world[0] - camera_to_world_t[0];
            tmp[1] = landmark_in_world[1] - camera_to_world_t[1];
            tmp[2] = landmark_in_world[2] - camera_to_world_t[2];

            T landmark_in_camera[3];
            ceres::QuaternionRotatePoint(world_to_camera_q, tmp, landmark_in_camera);

            T fx(mParent->mCalibration->cameras[0].calibration_matrix(0,0));
            T fy(mParent->mCalibration->cameras[0].calibration_matrix(1,1));
            T cx(mParent->mCalibration->cameras[0].calibration_matrix(0,2));
            T cy(mParent->mCalibration->cameras[0].calibration_matrix(1,2));

            if( landmark_in_camera[2] < mParent->mLandmarkRadius*0.1 )
            {
                ok = false;
            }
            else
            {
                T distance = ceres::sqrt(
                    landmark_in_camera[0]*landmark_in_camera[0] +
                    landmark_in_camera[1]*landmark_in_camera[1] +
                    landmark_in_camera[2]*landmark_in_camera[2] );

                T alpha = ceres::asin( T(mParent->mLandmarkRadius) / distance );

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

                prediction[3*i+0] = proj_x;
                prediction[3*i+1] = proj_y;
                prediction[3*i+2] = proj_radius;
            }
        }

        return ok;
    }
};

struct EKFOdometry::AugmentationFunction
{
    EKFOdometry* mParent;

    AugmentationFunction(EKFOdometry* parent)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(
        const T* const landmark_in_camera,
        const T* const camera_to_world_t,
        const T* const camera_to_world_q,
        T* landmark_in_world) const
    {
        T tmp[3];
        ceres::QuaternionRotatePoint(camera_to_world_q, landmark_in_camera, tmp);

        landmark_in_world[0] = camera_to_world_t[0] + tmp[0];
        landmark_in_world[1] = camera_to_world_t[1] + tmp[1];
        landmark_in_world[2] = camera_to_world_t[2] + tmp[2];

        return true;
    }
};

EKFOdometry::EKFOdometry(CalibrationDataPtr calibration)
{
    mLandmarkMinDistanceToCamera = 1.0;
    mLandmarkRadius = 1.0;
    mMaxLandmarks = 100; //330; // (1000-13)/3;
    mCalibration = calibration;
    mPredictionLinearMomentumSigmaRate = 1.0;
    mPredictionAngularMomentumSigmaRate = M_PI*10.0/180.0;
    mObservationPositionSigma = 1.0;
    mObservationRadiusSigma = 5.0;

    mInitialized = false;

    mStates[0].reset(new State());
    mStates[1].reset(new State());

    if(mCalibration->num_cameras == 0)
    {
        std::cerr << "Empty calibration" << std::endl;
        exit(1);
    }
}

bool EKFOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    Sophus::SE3d& camera_to_world,
    bool& aligned_wrt_previous)
{
    bool successful_tracking = false;

    if( mInitialized && circles.empty() == false )
    {
        successful_tracking = trackingPrediction(timestamp);
        //std::cout << "A " << currentState().camera_to_world.translation() << std::endl;
        //std::cout << "A " << currentState().covariance << std::endl;

        if(successful_tracking)
        {
            successful_tracking = trackingUpdate(circles);
        }

        if(successful_tracking)
        {
            successful_tracking = mappingPruneAugment(circles);
        }
    }

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    camera_to_world = currentState().camera_to_world;
    aligned_wrt_previous = successful_tracking;

    //std::cout << "TRACKING STATUS: " << aligned_wrt_previous << std::endl;

    //currentState().dump();

    /*
    std::ofstream file("trajectory.csv", std::ofstream::app);
    file
        << aligned_wrt_previous << " "
        << currentState().camera_to_world.translation().transpose().format(Eigen::FullPrecision) << " "
        << currentState().camera_to_world.unit_quaternion().coeffs().transpose().format(Eigen::FullPrecision) << " "
        << currentState().linear_momentum.transpose().format(Eigen::FullPrecision) << " "
        << currentState().angular_momentum.transpose().format(Eigen::FullPrecision) << std::endl;
    */

    return true;
}

void EKFOdometry::State::dump()
{
    std::cout << std::endl;
    std::cout << "timestamp: " << timestamp << std::endl;
    std::cout << "camera_to_world: " << std::endl;
    std::cout << camera_to_world.matrix().topRows<3>() << std::endl;
    std::cout << "linear_momentum: " << linear_momentum.transpose() << std::endl;
    std::cout << "angular_momentum: " << angular_momentum.transpose() << std::endl;
    for(size_t i=0; i<landmarks.size(); i++)
    {
        std::cout << "landmark_in_world #" << i << ": " << landmarks[i].position.transpose() << std::endl;
    }
    for(size_t i=0; i<landmarks.size(); i++)
    {
        std::cout << "landmark_in_camera #" << i << ": " << ( camera_to_world.inverse() * landmarks[i].position ).transpose() << std::endl;
    }
    std::cout << std::endl;
}

void EKFOdometry::reset()
{
    mInitialized = false;
}

bool EKFOdometry::triangulateLandmarkInWorldFrame(
    const Sophus::SE3d& camera_to_world,
    const Eigen::Matrix<double, 7, 7>& pose_covariance,
    const TrackedCircle& circle,
    NewLandmark& new_landmark)
{
    Eigen::Vector3d in_camera;
    Eigen::Matrix3d in_camera_covariance;

    Eigen::Vector3d in_world;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> jacobian_wrt_incamera;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> jacobian_wrt_camerat;
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> jacobian_wrt_cameraq;

    bool ok = true;

    if(ok)
    {
        ok = triangulateLandmarkInCameraFrame(circle, in_camera, in_camera_covariance);
    }

    if(ok)
    {
        const Eigen::Vector3d tmp0 = camera_to_world.translation();

        Eigen::Vector4d tmp1;
        tmp1.head<3>() = camera_to_world.unit_quaternion().vec();
        tmp1.w() = camera_to_world.unit_quaternion().w();

        const double* ceres_variable_ptr[3] = { in_camera.data(), tmp0.data(), tmp1.data() };
        double* ceres_value = in_world.data();
        double* ceres_jacobian_ptr[3] = { jacobian_wrt_incamera.data(), jacobian_wrt_camerat.data(), jacobian_wrt_cameraq.data() };

        std::unique_ptr<ceres::AutoDiffCostFunction<AugmentationFunction, 3, 3, 3, 4>> function(new ceres::AutoDiffCostFunction<AugmentationFunction, 3, 3, 3, 4>(new AugmentationFunction(this)));

        ok = function->Evaluate( ceres_variable_ptr, ceres_value, ceres_jacobian_ptr );
    }

    if(ok)
    {
        Eigen::Matrix<double, 10, 10> input_covariance;
        input_covariance.block<3,3>(0,0) = in_camera_covariance;
        input_covariance.block<3,7>(0,3).setZero();
        input_covariance.block<7,3>(3,0).setZero();
        input_covariance.block<7,7>(3,3) = pose_covariance;

        Eigen::Matrix<double, 10, 10> jacobian;
        jacobian.block<3,3>(0, 0) = jacobian_wrt_incamera;
        jacobian.block<3,3>(0, 3) = jacobian_wrt_camerat;
        jacobian.block<3,4>(0, 6) = jacobian_wrt_cameraq;
        jacobian.block<7,3>(3, 0).setZero();
        jacobian.block<7,7>(3, 3).setIdentity();

        const Eigen::Matrix<double, 10, 10> output_covariance = jacobian * input_covariance * jacobian.transpose();

        new_landmark.position = in_world;
        new_landmark.covariance_landmark_landmark = output_covariance.block<3,3>(0,0);
        new_landmark.covariance_landmark_camera.block<3,7>(0,0) = output_covariance.block<3,7>(0,3);
        new_landmark.covariance_landmark_camera.block<3,6>(0,3).setZero();
    }

    return ok;
}

bool EKFOdometry::triangulateLandmarkInCameraFrame(
    const TrackedCircle& tc,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    double ceres_variable[3];
    double ceres_value[3];
    double ceres_jacobian[9];
    double* ceres_variable_ptr = ceres_variable;
    double* ceres_jacobian_ptr = ceres_jacobian;

    const cv::Vec3f undistorted = undistortCircle(tc.circle);

    ceres_variable[0] = undistorted[0];
    ceres_variable[1] = undistorted[1];
    ceres_variable[2] = undistorted[2];

    std::unique_ptr<ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>> function(new ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>(new TriangulationFunction(this)));

    const bool ok = function->Evaluate( &ceres_variable_ptr, ceres_value, &ceres_jacobian_ptr );

    if(ok)
    {
        Eigen::Map< Eigen::Matrix<double,3,3,Eigen::RowMajor> > J(ceres_jacobian);

        const double sigma_center = 1.5;
        const double sigma_radius = 1.5;

        Eigen::Matrix3d S0;
        S0 <<
            sigma_center*sigma_center, 0.0, 0.0,
            0.0, sigma_center*sigma_center, 0.0,
            0.0, 0.0, sigma_radius*sigma_radius;

        covariance = J * S0 * J.transpose();
        position.x() = ceres_value[0];
        position.y() = ceres_value[1];
        position.z() = ceres_value[2];

        /*
        std::cout << sqrt(covariance(0,0)) << std::endl;
        std::cout << sqrt(covariance(1,1)) << std::endl;
        std::cout << sqrt(covariance(2,2)) << std::endl;
        */
    }

    return ok;
}

void EKFOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    State& state = currentState();

    const size_t num_circles = circles.size();

    state.timestamp = timestamp;
    state.camera_to_world = Sophus::SE3d(); // identity
    state.linear_momentum.setZero();
    state.angular_momentum.setZero();
    state.landmarks.clear();

    std::vector<Eigen::Vector3d> landmark_positions(num_circles);
    std::vector<Eigen::Matrix3d> landmark_covariances(num_circles);
    mCircleToLandmark.resize(num_circles);

    // triangulate circles into landmarks.

    size_t num_triangulated = 0;

    for(size_t i=0; i<num_circles; i++)
    {
        mCircleToLandmark[i].has_landmark = triangulateLandmarkInCameraFrame(
            circles[i],
            landmark_positions[i],
            landmark_covariances[i]);

        if( mCircleToLandmark[i].has_landmark )
        {
            num_triangulated++;
        }
    }

    // take only mMaxLandmarks landmarks (those with less variance).

    if( num_triangulated > mMaxLandmarks )
    {
        std::vector<size_t> sorted;
        sorted.reserve(num_triangulated);

        for(size_t i=0; i<num_circles; i++)
        {
            if(mCircleToLandmark[i].has_landmark)
            {
                sorted.push_back(i);
                mCircleToLandmark[i].has_landmark = false;
            }
        }

        auto pred = [&landmark_covariances] (size_t a, size_t b) -> bool
        {
            const double var_a = landmark_covariances[a].trace();
            const double var_b = landmark_covariances[b].trace();
            return var_a <= var_b;
        };

        std::sort(sorted.begin(), sorted.end(), pred);

        for(size_t i=0; i<mMaxLandmarks; i++)
        {
            mCircleToLandmark[ sorted[i] ].has_landmark = true;
        }
    }

    for(size_t i=0; i<num_circles; i++)
    {
        if( mCircleToLandmark[i].has_landmark )
        {
            mCircleToLandmark[i].landmark = state.landmarks.size();

            state.landmarks.emplace_back();
            state.landmarks.back().position = landmark_positions[i];
            state.landmarks.back().seen_count = 1;
        }
        else
        {
            mCircleToLandmark[i].landmark = 0; // static_cast<size_t>(-1);
        }
    }

    const size_t num_landmarks = state.landmarks.size();

    state.covariance.resize(13+3*num_landmarks, 13+3*num_landmarks);
    state.covariance.setZero();

    /*
    const double initial_sigma_position = 1.0e-3;
    const double initial_sigma_attitude = 1.0e-3;
    const double initial_sigma_linear_momentum = 1.0e-3;
    const double initial_sigma_angular_momentum = 1.0e-3;

    state.covariance.block<3,3>(0,0).diagonal().fill(initial_sigma_position*initial_sigma_position);
    state.covariance.block<4,4>(3,3).diagonal().fill(initial_sigma_attitude*initial_sigma_attitude);
    state.covariance.block<3,3>(7,7).diagonal().fill(initial_sigma_linear_momentum*initial_sigma_linear_momentum);
    state.covariance.block<3,3>(10,10).diagonal().fill(initial_sigma_angular_momentum*initial_sigma_angular_momentum);
    */

    for(size_t i=0; i<num_circles; i++)
    {
        if( mCircleToLandmark[i].has_landmark )
        {
            const size_t j = mCircleToLandmark[i].landmark;
            state.covariance.block<3,3>(13+3*j, 13+3*j) = landmark_covariances[i];
        }
    }

    //std::cout << state.covariance.block<3,3>(13, 13) << std::endl;
    //std::cout << "num_landmarks: " << num_landmarks << std::endl;

    mInitialized = true;
    //std::cout << state.covariance << std::endl;
}

cv::Vec3f EKFOdometry::undistortCircle(const cv::Vec3f& c)
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
        mCalibration->cameras[0].calibration_matrix,
        mCalibration->cameras[0].distortion_coefficients,
        cv::noArray(),
        mCalibration->cameras[0].calibration_matrix);

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

bool EKFOdometry::mappingPruneAugment(const std::vector<TrackedCircle>& circles)
{
    enum LandmarkStock
    {
        LMSTOCK_UNDEFINED,
        LMSTOCK_NEW_LANDMARK,
        LMSTOCK_OLD_LANDMARK
    };

    struct LandmarkSelection
    {
        LandmarkSelection()
        {
            stock = LMSTOCK_UNDEFINED;
            index = 0;
        }

        LandmarkStock stock;
        size_t index;
    };

    std::vector<NewLandmark> new_landmarks;

    State& old_state = currentState();
    State& new_state = workingState();

    // count number of landmarks seen in current frame.

    const size_t num_seen = std::count_if(
        old_state.landmarks.begin(),
        old_state.landmarks.end(),
        [] (const Landmark& lm) { return lm.seen_in_current_frame; } );

    if(num_seen > mMaxLandmarks)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    // try to triangulate new landmarks.

    if( num_seen < circles.size() && num_seen < mMaxLandmarks )
    {
        for(size_t i=0; num_seen + new_landmarks.size() < mMaxLandmarks && i < circles.size(); i++)
        {
            NewLandmark new_landmark;

            bool ok =
                circles[i].has_previous  == false ||
                mCircleToLandmark[circles[i].previous].has_landmark == false ||
                old_state.landmarks[ mCircleToLandmark[circles[i].previous].landmark ].seen_in_current_frame == false;

            if(ok)
            {
                ok = triangulateLandmarkInWorldFrame(
                    old_state.camera_to_world,
                    old_state.covariance.block<7,7>(0,0),
                    circles[i],
                    new_landmark);
            }

            if(ok)
            {
                new_landmarks.push_back(std::move(new_landmark));
            }
        }
    }

    // compute new state and new circle to landmark association.

    // Optimization problem is:
    //
    //    max num_to_add
    //
    //    such that
    //
    //    num_to_add + num_seen - num_to_remove <= mMaxLandmarks
    //    0 <= num_to_add <= new_landmarks.size()
    //    0 <= num_to_remove <= old_state.landmarks.size()

    size_t num_to_add = 0;
    size_t num_to_remove = 0;

    if( num_seen < mMaxLandmarks )
    {
        if( num_seen + new_landmarks.size() <= mMaxLandmarks )
        {
            num_to_add = new_landmarks.size();
        }
        else
        {
            num_to_add = mMaxLandmarks - num_seen;
        }

        if(old_state.landmarks.size() + num_to_add > mMaxLandmarks)
        {
            num_to_remove = old_state.landmarks.size() + num_to_add - mMaxLandmarks;
        }
    }

    const size_t new_num_landmarks = old_state.landmarks.size() + num_to_add - num_to_remove;

    if( new_num_landmarks > mMaxLandmarks || num_to_add > new_landmarks.size() )
    {
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }

    // compute new state.

    std::vector<LandmarkSelection> landmark_selection(new_num_landmarks);
    new_state.landmarks.resize(new_num_landmarks);
    new_state.covariance.resize(13+3*new_num_landmarks, 13+3*new_num_landmarks);
    size_t landmark_counter = 0;

    // take from old landmarks.

    for(size_t i=0; i<old_state.landmarks.size(); i++)
    {
        const bool take = ( old_state.landmarks[i].seen_in_current_frame || landmark_counter < old_state.landmarks.size()-num_to_remove );

        if(take)
        {
            landmark_selection[landmark_counter].stock = LMSTOCK_OLD_LANDMARK;
            landmark_selection[landmark_counter].index = i;
            landmark_counter++;
        }
    }

    // take new landmarks.

    for(size_t i=0; i<num_to_add; i++)
    {
        landmark_selection[landmark_counter].stock = LMSTOCK_NEW_LANDMARK;
        landmark_selection[landmark_counter].index = i;
        landmark_counter++;
    }

    if(landmark_counter != new_num_landmarks)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    new_state.covariance.block<13,13>(0,0) = old_state.covariance.block<13,13>(0,0);

    for(size_t i=0; i<new_num_landmarks; i++)
    {
        const auto& lms = landmark_selection[i];

        if( lms.stock == LMSTOCK_OLD_LANDMARK )
        {
            new_state.landmarks[i] = old_state.landmarks[lms.index];

            new_state.covariance.block<3,3>(13+3*i, 13+3*i) = old_state.covariance.block<3,3>(13+3*lms.index, 13+3*lms.index);
            new_state.covariance.block<13,3>(0, 13+3*i) = old_state.covariance.block<13,3>(0, 13+3*lms.index);
            new_state.covariance.block<3,13>(13+3*i, 0) = old_state.covariance.block<3,13>(13+3*lms.index, 0);

            for(size_t j=0; j<i; j++)
            {
                const auto& other_lms = landmark_selection[j];

                if(other_lms.stock == LMSTOCK_OLD_LANDMARK)
                {
                    new_state.covariance.block<3,3>(13+3*i, 13+3*j) = old_state.covariance.block<3,3>(13+3*lms.index, 13+3*other_lms.index);
                    new_state.covariance.block<3,3>(13+3*j, 13+3*i) = old_state.covariance.block<3,3>(13+3*other_lms.index, 13+3*lms.index);
                }
                else
                {
                    new_state.covariance.block<3,3>(13+3*i, 13+3*j).setZero();
                    new_state.covariance.block<3,3>(13+3*j, 13+3*i).setZero();
                }
            }
        }
        else if( lms.stock == LMSTOCK_NEW_LANDMARK )
        {
            new_state.landmarks[i].position = new_landmarks[lms.index].position;
            new_state.landmarks[i].seen_count = 1;
            new_state.landmarks[i].seen_in_current_frame = true;

            new_state.covariance.block<3,3>(13+3*i, 13+3*i) = new_landmarks[lms.index].covariance_landmark_landmark;
            new_state.covariance.block<13,3>(0, 13+3*i) = new_landmarks[lms.index].covariance_landmark_camera.transpose();
            new_state.covariance.block<3,13>(13+3*i, 0) = new_landmarks[lms.index].covariance_landmark_camera;

            for(size_t j=0; j<i; j++)
            {
                new_state.covariance.block<3,3>(13+3*i, 13+3*j).setZero();
                new_state.covariance.block<3,3>(13+3*j, 13+3*i).setZero();
            }
        }
        else
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }
    }

    switchStates();

    return true;
}

bool EKFOdometry::trackingUpdate(const std::vector<TrackedCircle>& circles)
{
    State& old_state = currentState();
    State& new_state = workingState();

    std::vector<ObservedLandmark> observed_landmarks;
    observed_landmarks.reserve(old_state.landmarks.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        if(circles[i].has_previous && mCircleToLandmark[circles[i].previous].has_landmark)
        {
            const size_t landmark = mCircleToLandmark[circles[i].previous].landmark;

            const Eigen::Vector3d landmark_in_camera = old_state.camera_to_world.inverse() * old_state.landmarks[landmark].position;

            if( landmark_in_camera.z() > mLandmarkMinDistanceToCamera )
            {
                observed_landmarks.emplace_back();
                observed_landmarks.back().landmark = landmark;
                observed_landmarks.back().undistorted_circle = undistortCircle( circles[i].circle );
            }
        }
    }

    bool ok = true;

    if( observed_landmarks.empty() )
    {
        // if no landmark seen, consider we are lost.
        ok = false;
    }
    else
    {
        const size_t observation_dim = 3*observed_landmarks.size();

        Eigen::VectorXd old_state_vector = old_state.toVector();

        Eigen::VectorXd predicted_observation(observation_dim);

        Eigen::VectorXd sensed_observation(observation_dim);

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(observation_dim, old_state.getDimension());

        Eigen::MatrixXd sensing_covariance = Eigen::MatrixXd::Zero(observation_dim, observation_dim);

        for(size_t i=0; i<observed_landmarks.size(); i++)
        {
            sensed_observation[3*i+0] = observed_landmarks[i].undistorted_circle[0];
            sensed_observation[3*i+1] = observed_landmarks[i].undistorted_circle[1];
            sensed_observation[3*i+2] = observed_landmarks[i].undistorted_circle[2];

            sensing_covariance(3*i+0, 3*i+0) = mObservationPositionSigma*mObservationPositionSigma;
            sensing_covariance(3*i+1, 3*i+1) = mObservationPositionSigma*mObservationPositionSigma;
            sensing_covariance(3*i+2, 3*i+2) = mObservationRadiusSigma*mObservationRadiusSigma;
        }

        std::unique_ptr< ceres::DynamicAutoDiffCostFunction<ObservationFunction> > function(new ceres::DynamicAutoDiffCostFunction<ObservationFunction>(new ObservationFunction(observed_landmarks, this)));
        function->AddParameterBlock(old_state.getDimension());
        function->SetNumResiduals(observation_dim);
        const double* ceres_old_state = old_state_vector.data();
        double* ceres_jacobian = jacobian.data();
        ok = function->Evaluate(&ceres_old_state, predicted_observation.data(), &ceres_jacobian);

        if(ok)
        {
            /*
            //for(size_t i=0; i<observed_landmarks.size(); i++)
            {
                const int i = 0;
                std::cout << sensed_observation[3*i+0] << " -> " << predicted_observation[3*i+0] << std::endl;
                std::cout << sensed_observation[3*i+1] << " -> " << predicted_observation[3*i+1] << std::endl;
                std::cout << sensed_observation[3*i+2] << " -> " << predicted_observation[3*i+2] << std::endl;
                std::cout << std::endl;
                //break;
            }
            */

            const Eigen::VectorXd residual = sensed_observation - predicted_observation;

            const Eigen::MatrixXd residual_covariance = jacobian * old_state.covariance * jacobian.transpose() + sensing_covariance;

            Eigen::LDLT<Eigen::MatrixXd> solver;
            solver.compute(residual_covariance);

            const Eigen::VectorXd new_state_vector = old_state_vector + old_state.covariance * jacobian.transpose() * solver.solve(residual);

            const Eigen::MatrixXd new_covariance = old_state.covariance - old_state.covariance*jacobian.transpose()*solver.solve(jacobian * old_state.covariance);

            new_state.camera_to_world.translation() = new_state_vector.segment<3>(0);

            new_state.camera_to_world.setQuaternion(Eigen::Quaterniond( new_state_vector(6), new_state_vector(3), new_state_vector(4), new_state_vector(5) ));

            new_state.linear_momentum = new_state_vector.segment<3>(7);

            new_state.angular_momentum = new_state_vector.segment<3>(10);

            new_state.landmarks = std::move(old_state.landmarks);

            for(size_t i=0; i<new_state.landmarks.size(); i++)
            {
                new_state.landmarks[i].position = new_state_vector.segment<3>(13+3*i);
                new_state.landmarks[i].seen_in_current_frame = false;
                //new_state.landmarks[i].seen_count = old_state.landmarks[i].seen_count;
            }

            for(size_t i=0; i<observed_landmarks.size(); i++)
            {
                new_state.landmarks[ observed_landmarks[i].landmark ].seen_count++;
                new_state.landmarks[ observed_landmarks[i].landmark ].seen_in_current_frame = true;
            }

            new_state.covariance = new_covariance;

            new_state.timestamp = old_state.timestamp;
        }
    }

    if(ok)
    {
        switchStates();
    }

    return ok;
}

bool EKFOdometry::trackingPrediction(double timestamp)
{
    State& old_state = currentState();
    State& new_state = workingState();

    const double timestep = timestamp - old_state.timestamp;

    const size_t dim = 13 + 3*old_state.landmarks.size();

    Eigen::VectorXd initial_state = old_state.toVector();
    Eigen::VectorXd predicted_state(old_state.getDimension());

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(dim, dim);

    std::unique_ptr< ceres::DynamicAutoDiffCostFunction<PredictionFunction> > function(new ceres::DynamicAutoDiffCostFunction<PredictionFunction>(new PredictionFunction(timestep, old_state.landmarks.size(), this)));
    function->AddParameterBlock(dim);
    function->SetNumResiduals(dim);
    const double* ceres_params = initial_state.data();
    double* ceres_derivative = jacobian.data();
    bool ok = function->Evaluate(&ceres_params, predicted_state.data(), &ceres_derivative);

    if(ok)
    {
        new_state.timestamp = timestamp;

        new_state.camera_to_world.translation() = predicted_state.segment<3>(0);

        new_state.camera_to_world.setQuaternion(Eigen::Quaterniond( predicted_state(6), predicted_state(3), predicted_state(4), predicted_state(5) ));

        new_state.linear_momentum = old_state.linear_momentum;
        new_state.angular_momentum = old_state.angular_momentum;
        new_state.landmarks = std::move(old_state.landmarks);

        for(Landmark& lm : new_state.landmarks)
        {
            lm.seen_in_current_frame = false;
        }

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> noise(dim, dim);
        noise.setZero();

        noise(7,7) = mPredictionLinearMomentumSigmaRate * mPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(8,8) = mPredictionLinearMomentumSigmaRate * mPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(9,9) = mPredictionLinearMomentumSigmaRate * mPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(10,10) = mPredictionAngularMomentumSigmaRate * mPredictionAngularMomentumSigmaRate * timestep * timestep;
        noise(11,11) = mPredictionAngularMomentumSigmaRate * mPredictionAngularMomentumSigmaRate * timestep * timestep;
        noise(12,12) = mPredictionAngularMomentumSigmaRate * mPredictionAngularMomentumSigmaRate * timestep * timestep;

        new_state.covariance = jacobian * (old_state.covariance + noise) * jacobian.transpose();

        switchStates();
    }

    return ok;
}

EKFOdometry::Landmark::Landmark()
{
    seen_count = 0;
}

EKFOdometry::CircleToLandmark::CircleToLandmark()
{
    has_landmark = false;
    landmark = 0;
}

EKFOdometry::State::State()
{
    timestamp = 0.0;
}

size_t EKFOdometry::State::getDimension()
{
    return 13 + 3*landmarks.size();
}

Eigen::VectorXd EKFOdometry::State::toVector()
{
    Eigen::VectorXd ret(getDimension());

    ret.segment<7>(0) = poseToVector(camera_to_world);
    ret.segment<3>(7) = linear_momentum;
    ret.segment<3>(10) = angular_momentum;

    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.segment<3>(13+3*i) = landmarks[i].position;
    }

    return ret;
}

Eigen::Matrix<double, 7, 1> EKFOdometry::poseToVector(const Sophus::SE3d& pose)
{
    Eigen::Matrix<double,7,1> ret;

    ret.head<3>() = pose.translation();
    ret.segment<3>(3) = pose.unit_quaternion().vec();
    ret(6) = pose.unit_quaternion().w();

    return ret;
}

Sophus::SE3d EKFOdometry::vectorToPose(const Eigen::Matrix<double, 7, 1>& vector)
{
    Sophus::SE3d ret;

    ret.translation() = vector.head<3>();
    ret.setQuaternion( Eigen::Quaterniond( vector(6), vector(3), vector(4), vector(5) ) );

    return ret;
}
