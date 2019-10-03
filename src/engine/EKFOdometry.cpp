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

        const double IK00 = mParent->myCalibration->cameras[0].inverse_calibration_matrix(0,0);
        const double IK02 = mParent->myCalibration->cameras[0].inverse_calibration_matrix(0,2);
        const double IK11 = mParent->myCalibration->cameras[0].inverse_calibration_matrix(1,1);
        const double IK12 = mParent->myCalibration->cameras[0].inverse_calibration_matrix(1,2);

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
            T distance = mParent->myLandmarkRadius/ceres::sin(beta);

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
        size_t offset = 0;

        Eigen::Map< Sophus::SE3<T> > old_pose( old_state_arr[0] + offset );
        Eigen::Map< Sophus::SE3<T> > new_pose( new_state + offset );
        offset += Sophus::SE3<T>::num_parameters;

        Eigen::Map< typename Sophus::SE3<T>::Tangent > old_momentum( old_state_arr[0] + offset );
        Eigen::Map< typename Sophus::SE3<T>::Tangent > new_momentum( new_state + offset );
        offset += Sophus::SE3<T>::DoF;

        for(size_t i=0; i<mNumLandmarks; i++)
        {
            Eigen::Map< Eigen::Matrix<T,3,1> > old_landmark( old_state_arr[0] + offset );
            Eigen::Map< Eigen::Matrix<T,3,1> > new_landmark( new_state + offset );
            offset += 3;

            new_landmark = old_landmark;
        }

        new_momentum = old_momentum;

        // convert momentum to velocity. Assumes mass is one and inertia matrix is identity.
        const typename Sophus::SE3<T>::Tangent velocity = old_momentum;

        new_pose = old_pose * Sophus::SE3<T>::exp(mTimestep * velocity);

        return true;
    }
};

/*
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

            T fx(mParent->myCalibration->cameras[0].calibration_matrix(0,0));
            T fy(mParent->myCalibration->cameras[0].calibration_matrix(1,1));
            T cx(mParent->myCalibration->cameras[0].calibration_matrix(0,2));
            T cy(mParent->myCalibration->cameras[0].calibration_matrix(1,2));

            if( landmark_in_camera[2] < mParent->myLandmarkRadius*0.1 )
            {
                ok = false;
            }
            else
            {
                T distance = ceres::sqrt(
                    landmark_in_camera[0]*landmark_in_camera[0] +
                    landmark_in_camera[1]*landmark_in_camera[1] +
                    landmark_in_camera[2]*landmark_in_camera[2] );

                T alpha = ceres::asin( T(mParent->myLandmarkRadius) / distance );

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

                prediction[3*i+0] = proj_x;
                prediction[3*i+1] = proj_y;
                prediction[3*i+2] = proj_radius;
            }
        }

        return ok;
    }
};
*/

/*
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
*/

EKFOdometry::EKFOdometry(CalibrationDataPtr calibration)
{
    myLandmarkMinDistanceToCamera = 1.0;
    myLandmarkRadius = 1.0;
    myMaxLandmarks = 100; //330; // (1000-13)/3;
    myCalibration = calibration;
    myPredictionLinearMomentumSigmaRate = 1.0;
    myPredictionAngularMomentumSigmaRate = M_PI*10.0/180.0;
    myObservationPositionSigma = 1.0;
    myObservationRadiusSigma = 5.0;

    myInitialized = false;

    myStates[0].reset(new State());
    myStates[1].reset(new State());

    if(myCalibration->cameras.empty())
    {
        std::cerr << "Empty calibration" << std::endl;
        exit(1);
    }
}

bool EKFOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles, OdometryFrame& output)
{
    bool successful_tracking = myInitialized;

    if(successful_tracking)
    {
        successful_tracking = trackingPrediction(timestamp);
    }

    if(successful_tracking)
    {
        successful_tracking = trackingUpdate(circles);
    }

    if(successful_tracking)
    {
        successful_tracking = mappingPruneAugment(circles);
    }

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    output.timestamp = timestamp;
    output.aligned_wrt_previous = successful_tracking;
    output.camera_to_world = currentState().camera_to_world;
    //output.pose_covariance.setIdentity(); // TODO set corrent pose covariance.
    output.landmarks.clear();
    for(Landmark& lm : currentState().landmarks)
    {
        if(lm.seen_in_current_frame)
        {
            output.landmarks.emplace_back();
            output.landmarks.back().position = lm.position;
        }
    }

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
    std::cout << "timestamp = " << timestamp << std::endl;
    std::cout << "camera_to_world_t =  " << camera_to_world.translation().transpose() << std::endl;
    std::cout << "camera_to_world_q = " << camera_to_world.unit_quaternion().coeffs().transpose() << std::endl;
    std::cout << "momentum_translation = " << momentum.head<3>().transpose() << std::endl;
    std::cout << "momentum_rotation = " << momentum.tail<3>().transpose() << std::endl;
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
    myInitialized = false;
}

/*
bool EKFOdometry::triangulateLandmark(
    const cv::Vec3f& circle,
    const Sophus::SE3d& camera_to_world,
    TriangulatedLandmark& new_landmark)
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

        std::unique_ptr<CeresAugmentationFunction> function;
        function.reset(new CeresAugmentationFunction(new AugmentationFunction(this)));

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
*/

bool EKFOdometry::triangulateLandmarkInCameraFrame(
    const cv::Vec3f& circle,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    const cv::Vec3d undistorted_circle = undistortCircle(circle);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> jacobian;

    const double* ceres_variable_ptr = undistorted_circle.val;
    double* ceres_jacobian_ptr = jacobian.data();

    std::unique_ptr<CeresTriangulationFunction> function;
    function.reset(new CeresTriangulationFunction(new TriangulationFunction(this)));

    const bool ok = function->Evaluate( &ceres_variable_ptr, position.data(), &ceres_jacobian_ptr );

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

void EKFOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    State& state = currentState();

    const size_t num_circles = circles.size();

    state.timestamp = timestamp;
    state.camera_to_world = Sophus::SE3d(); // identity
    state.momentum.setZero();
    state.landmarks.clear();

    std::vector<Eigen::Vector3d> landmark_positions(num_circles);
    std::vector<Eigen::Matrix3d> landmark_covariances(num_circles);
    myCircleToLandmark.resize(num_circles);

    // triangulate circles into landmarks.


    for(size_t i=0; i<num_circles; i++)
    {
        myCircleToLandmark[i].has_landmark = triangulateLandmarkInCameraFrame(
            circles[i].circle,
            landmark_positions[i],
            landmark_covariances[i]);

        if( myCircleToLandmark[i].has_landmark )
        {
            myCircleToLandmark[i].landmark = state.landmarks.size();

            state.landmarks.emplace_back();
            state.landmarks.back().position = landmark_positions[i];
            state.landmarks.back().seen_count = 1;
        }
        else
        {
            myCircleToLandmark[i].landmark = 0; // static_cast<size_t>(-1);
        }
    }

    const size_t num_landmarks = state.landmarks.size();
    const size_t dim = state.getDimension();

    state.covariance.resize(dim, dim);
    state.covariance.setZero();

    for(size_t i=0; i<num_circles; i++)
    {
        if( myCircleToLandmark[i].has_landmark )
        {
            const size_t j = myCircleToLandmark[i].landmark;
            state.covariance.block<3,3>(13+3*j, 13+3*j) = landmark_covariances[i];
        }
    }

    //std::cout << state.covariance.block<3,3>(13, 13) << std::endl;
    //std::cout << "num_landmarks: " << num_landmarks << std::endl;

    myInitialized = true;
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

bool EKFOdometry::mappingPruneAugment(const std::vector<TrackedCircle>& circles)
{
    /*
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

    if(num_seen > myMaxLandmarks)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    // try to triangulate new landmarks.

    if( num_seen < circles.size() && num_seen < myMaxLandmarks )
    {
        for(size_t i=0; num_seen + new_landmarks.size() < myMaxLandmarks && i < circles.size(); i++)
        {
            NewLandmark new_landmark;

            bool ok =
                circles[i].has_previous  == false ||
                myCircleToLandmark[circles[i].previous].has_landmark == false ||
                old_state.landmarks[ myCircleToLandmark[circles[i].previous].landmark ].seen_in_current_frame == false;

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
    //    num_to_add + num_seen - num_to_remove <= myMaxLandmarks
    //    0 <= num_to_add <= new_landmarks.size()
    //    0 <= num_to_remove <= old_state.landmarks.size()

    size_t num_to_add = 0;
    size_t num_to_remove = 0;

    if( num_seen < myMaxLandmarks )
    {
        if( num_seen + new_landmarks.size() <= myMaxLandmarks )
        {
            num_to_add = new_landmarks.size();
        }
        else
        {
            num_to_add = myMaxLandmarks - num_seen;
        }

        if(old_state.landmarks.size() + num_to_add > myMaxLandmarks)
        {
            num_to_remove = old_state.landmarks.size() + num_to_add - myMaxLandmarks;
        }
    }

    const size_t new_num_landmarks = old_state.landmarks.size() + num_to_add - num_to_remove;

    if( new_num_landmarks > myMaxLandmarks || num_to_add > new_landmarks.size() )
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
    */
    return false;
}

bool EKFOdometry::trackingUpdate(const std::vector<TrackedCircle>& circles)
{
    /*
    State& old_state = currentState();
    State& new_state = workingState();

    std::vector<ObservedLandmark> observed_landmarks;
    observed_landmarks.reserve(old_state.landmarks.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        if(circles[i].has_previous && myCircleToLandmark[circles[i].previous].has_landmark)
        {
            const size_t landmark = myCircleToLandmark[circles[i].previous].landmark;

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

            sensing_covariance(3*i+0, 3*i+0) = myObservationPositionSigma*myObservationPositionSigma;
            sensing_covariance(3*i+1, 3*i+1) = myObservationPositionSigma*myObservationPositionSigma;
            sensing_covariance(3*i+2, 3*i+2) = myObservationRadiusSigma*myObservationRadiusSigma;
        }

        std::unique_ptr<CeresObservationFunction> function;
        function.reset(new CeresObservationFunction(new ObservationFunction(observed_landmarks, this)));

        function->AddParameterBlock(old_state.getDimension());
        function->SetNumResiduals(observation_dim);
        const double* ceres_old_state = old_state_vector.data();
        double* ceres_jacobian = jacobian.data();
        ok = function->Evaluate(&ceres_old_state, predicted_observation.data(), &ceres_jacobian);

        if(ok)
        {
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
    */
    return false;
}

bool EKFOdometry::trackingPrediction(double timestamp)
{
    State& old_state = currentState();
    State& new_state = workingState();

    const double timestep = timestamp - old_state.timestamp;

    const size_t dim = old_state.getDimension();

    Eigen::VectorXd initial_state = old_state.toVector();
    Eigen::VectorXd predicted_state(dim);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(dim, dim);

    std::unique_ptr<CeresPredictionFunction> function;
    function.reset(new CeresPredictionFunction(new PredictionFunction(timestep, old_state.landmarks.size(), this)));

    function->AddParameterBlock(dim);
    function->SetNumResiduals(dim);

    const double* ceres_params = initial_state.data();
    double* ceres_derivative = jacobian.data();
    bool ok = function->Evaluate(&ceres_params, predicted_state.data(), &ceres_derivative);

    if(ok)
    {
        // TODO TODO TODO TODO
        exit(1);
        /*
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
        */
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

size_t EKFOdometry::State::getDimension() const
{
    return
        Sophus::SE3d::num_parameters +
        Sophus::SE3d::DoF +
        3*landmarks.size();
}

Eigen::VectorXd EKFOdometry::State::toVector() const
{
    Eigen::VectorXd ret(getDimension());

    Eigen::Map<Sophus::SE3d>( ret.data() ) = camera_to_world;

    Eigen::Map<Sophus::SE3d::Tangent>( ret.data() + Sophus::SE3d::num_parameters ) = momentum;

    for(size_t i=0; i<landmarks.size(); i++)
    {
        Eigen::Map<Eigen::Vector3d>( ret.data() + Sophus::SE3d::num_parameters + Sophus::SE3d::DoF ) = landmarks[i].position;
    }

    return ret;
}

void EKFOdometry::switchStates()
{
    std::swap(myStates[0], myStates[1]);
}

EKFOdometry::State& EKFOdometry::currentState()
{
    return *myStates[0];
}

EKFOdometry::State& EKFOdometry::workingState()
{
    return *myStates[1];
}
