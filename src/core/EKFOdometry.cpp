#include <fstream>
#include <iomanip>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/rotation.h>
#include "EKFOdometry.h"
#include "OdometryHelpers.h"
#include "CoreConstants.h"

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
            T distance = CORE_LANDMARK_RADIUS / ceres::sin(beta);

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

        const Eigen::Map< const Sophus::SE3<T> > old_pose( old_state_arr[0] + offset );
        Eigen::Map< Sophus::SE3<T> > new_pose( new_state + offset );
        offset += Sophus::SE3<T>::num_parameters;

        const Eigen::Map< const typename Sophus::SE3<T>::Tangent > old_momentum( old_state_arr[0] + offset );
        Eigen::Map< typename Sophus::SE3<T>::Tangent > new_momentum( new_state + offset );
        offset += Sophus::SE3<T>::DoF;

        for(size_t i=0; i<mNumLandmarks; i++)
        {
            const Eigen::Map< const Eigen::Matrix<T,3,1> > old_landmark( old_state_arr[0] + offset );
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

struct EKFOdometry::ObservationFunction
{
    EKFOdometry* mParent;
    const std::vector<ObservedLandmark>& mVisibleLandmarks;

    ObservationFunction(const std::vector<ObservedLandmark>& visible_landmarks, EKFOdometry* parent) :
        mVisibleLandmarks(visible_landmarks)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(T const* const* state_arr, T* prediction) const
    {
        bool ok = true;

        const Eigen::Map< const Sophus::SE3<T> > camera_to_world(state_arr[0]);

        for(size_t i=0; ok && i<mVisibleLandmarks.size(); i++)
        {
            const size_t j = mVisibleLandmarks[i].landmark;

            const Eigen::Map< const Eigen::Matrix<T,3,1> > landmark_in_world( state_arr[0] + Sophus::SE3<T>::num_parameters + Sophus::SE3<T>::DoF + 3*j );

            const Eigen::Matrix<T,3,1> landmark_in_camera = camera_to_world.inverse() * landmark_in_world;

            const T fx(mParent->myCalibration->cameras[0].calibration_matrix(0,0));
            const T fy(mParent->myCalibration->cameras[0].calibration_matrix(1,1));
            const T cx(mParent->myCalibration->cameras[0].calibration_matrix(0,2));
            const T cy(mParent->myCalibration->cameras[0].calibration_matrix(1,2));

            if( landmark_in_camera.z() < CORE_LANDMARK_RADIUS*0.1 )
            {
                ok = false;
            }
            else
            {
                const T distance = landmark_in_camera.norm();

                const T alpha = ceres::asin( T(CORE_LANDMARK_RADIUS) / distance );

                T center_los[2];
                center_los[0] = landmark_in_camera.x() / landmark_in_camera.z();
                center_los[1] = landmark_in_camera.y() / landmark_in_camera.z();

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

struct EKFOdometry::AugmentationFunction
{
    EKFOdometry* mParent;

    AugmentationFunction(EKFOdometry* parent)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(
        const T* const camera_to_world_,
        const T* const landmark_in_camera_,
        T* landmark_in_world_) const
    {
        const Eigen::Map< const Sophus::SE3<T> > camera_to_world(camera_to_world_);

        const Eigen::Map< const Eigen::Matrix<T,3,1> > landmark_in_camera(landmark_in_camera_);

        Eigen::Map< Eigen::Matrix<T,3,1> > landmark_in_world(landmark_in_world_);

        landmark_in_world = camera_to_world * landmark_in_camera;

        return true;
    }
};

EKFOdometry::EKFOdometry(CalibrationDataPtr calibration)
{
    myLandmarkMinDistanceToCamera = CORE_LANDMARK_RADIUS;
    myMaxLandmarks = 100; //330; // (1000-13)/3;
    myCalibration = calibration;
    myPredictionLinearMomentumSigmaRate = 1.0;
    myPredictionAngularMomentumSigmaRate = M_PI*10.0/180.0;
    myObservationPositionSigma = 1.5;
    myObservationRadiusSigma = 3.0;

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
    bool successful_tracking = currentState().valid;

    if(successful_tracking)
    {
        successful_tracking = trackingPrediction(timestamp, circles);
    }

    if(successful_tracking)
    {
        successful_tracking = trackingUpdate();
    }

    if(successful_tracking)
    {
        successful_tracking = mappingPruneAugment();
    }

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    output.timestamp = currentState().timestamp;
    output.aligned_wrt_previous = successful_tracking;
    output.camera_to_world = currentState().camera_to_world;
    //output.pose_covariance.setIdentity(); // TODO set current pose covariance.
    output.landmarks.clear();
    for(Landmark& lm : currentState().landmarks)
    {
        if(lm.currently_seen)
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

/*
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
*/

void EKFOdometry::reset()
{
    currentState().valid = false;
}

bool EKFOdometry::triangulateLandmarkInWorldFrame(
    const cv::Vec3f& circle,
    const Sophus::SE3d& camera_to_world,
    const SE3SE3CovarianceMatrix& camera_to_world_covariance,
    Eigen::Vector3d& landmark_in_world,
    Eigen::Matrix3d& covariance_landmark_landmark,
    Vector3SE3CovarianceMatrix& covariance_landmark_camera)
{
    Eigen::Vector3d in_camera;
    Eigen::Matrix3d in_camera_covariance;

    Eigen::Vector3d in_world;
    Eigen::Matrix<double, 3, Sophus::SE3d::num_parameters, Eigen::RowMajor> jacobian_wrt_camera;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> jacobian_wrt_incamera;

    bool ok = true;

    if(ok)
    {
        ok = triangulateLandmarkInCameraFrame(circle, in_camera, in_camera_covariance);
    }

    if(ok)
    {
        const double* ceres_variable_arr[2] = { camera_to_world.data(), in_camera.data() };
        double* ceres_jacobian_arr[2] = { jacobian_wrt_camera.data(), jacobian_wrt_incamera.data() };

        std::unique_ptr<CeresAugmentationFunction> function;
        function.reset(new CeresAugmentationFunction(new AugmentationFunction(this)));

        ok = function->Evaluate( ceres_variable_arr, in_world.data(), ceres_jacobian_arr );
    }

    if(ok)
    {
        constexpr size_t K = Sophus::SE3d::num_parameters;

        Eigen::Matrix<double, K+3, K+3> input_covariance;
        input_covariance.setZero();
        input_covariance.block<K,K>(0,0) = camera_to_world_covariance;
        input_covariance.block<3,3>(K,K) = in_camera_covariance;

        // jacobian of (landmark_in_camera, camera_to_world) -> (landmark_in_world, camera_to_world).
        Eigen::Matrix<double, K+3, K+3> jacobian;

        jacobian.block<3,3>(0,0) = jacobian_wrt_incamera;
        jacobian.block<3,K>(0,3) = jacobian_wrt_camera;
        jacobian.block<K,3>(3,0).setZero();
        jacobian.block<K,K>(3,3).setIdentity();

        const Eigen::Matrix<double, K+3, K+3> output_covariance = jacobian * input_covariance * jacobian.transpose();

        landmark_in_world = in_world;
        covariance_landmark_landmark = output_covariance.block<3,3>(0,0);
        covariance_landmark_camera = output_covariance.block<3,K>(0,3);
    }

    return ok;
}

bool EKFOdometry::triangulateLandmarkInCameraFrame(
    const cv::Vec3f& circle,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    const cv::Vec3d undistorted_circle = OdometryHelpers::undistortCircle(circle, myCalibration);
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
    std::vector<Eigen::Vector3d> landmark_positions(num_circles);
    std::vector<Eigen::Matrix3d> landmark_covariances(num_circles);

    state.valid = true;
    state.timestamp = timestamp;
    state.camera_to_world = Sophus::SE3d(); // identity
    state.momentum.setZero();
    state.landmarks.clear();
    state.observations.assign(num_circles, Observation());

    // triangulate circles into landmarks.

    for(size_t i=0; i<num_circles; i++)
    {
        state.observations[i].circle = circles[i].circle;

        state.observations[i].has_landmark = (state.landmarks.size() < myMaxLandmarks) && triangulateLandmarkInCameraFrame(
            circles[i].circle,
            landmark_positions[i],
            landmark_covariances[i]);

        if( state.observations[i].has_landmark )
        {
            state.observations[i].landmark = state.landmarks.size();

            state.landmarks.emplace_back();
            Landmark& lm = state.landmarks.back();
            lm.position = landmark_positions[i];
            lm.seen_count = 1;
            lm.currently_seen = true;
            lm.current_observation = i;
            lm.updated = true;
        }
        else
        {
            state.observations[i].landmark = 0; // static_cast<size_t>(-1);
        }
    }

    const size_t num_landmarks = state.landmarks.size();
    const size_t dim = state.getDimension();

    state.covariance.resize(dim, dim);
    state.covariance.setZero();

    for(size_t i=0; i<num_circles; i++)
    {
        if( state.observations[i].has_landmark )
        {
            const size_t j = state.observations[i].landmark;
            state.covariance.block<3,3>(13+3*j, 13+3*j) = landmark_covariances[i];
        }
    }

    //std::cout << state.covariance.block<3,3>(13, 13) << std::endl;
    //std::cout << "num_landmarks: " << num_landmarks << std::endl;
    //std::cout << state.covariance << std::endl;
}

bool EKFOdometry::mappingPruneAugment()
{
    constexpr int se3_num_parameters = Sophus::SE3d::num_parameters;
    constexpr int se3_dof = Sophus::SE3d::DoF;
    constexpr int pose_and_momentum = se3_num_parameters + se3_dof;

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

    struct NewLandmark
    {
        size_t observation;
        Eigen::Vector3d position;
        Eigen::Matrix3d covariance_landmark_landmark;
        Vector3SE3CovarianceMatrix covariance_landmark_camera;
    };

    State& old_state = currentState();
    State& new_state = workingState();

    if(old_state.valid == false)
    {
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }

    // take landmarks from three pools in following order until maximum number of landmarks is reached.:
    // 1. seen existing landmarks.
    // 2. new triangulated landmarks.
    // 3. unseen existing landmarks.

    std::vector<LandmarkSelection> landmark_selection;
    std::vector<NewLandmark> new_landmarks;

    // take from seen existing landmarks.

    for(size_t i=0; landmark_selection.size() < myMaxLandmarks && i<old_state.landmarks.size(); i++)
    {
        if(old_state.landmarks[i].currently_seen)
        {
            landmark_selection.emplace_back();
            landmark_selection.back().stock = LMSTOCK_OLD_LANDMARK;
            landmark_selection.back().index = i;
        }
    }

    // take from new landmarks.

    for(size_t i=0; landmark_selection.size() < myMaxLandmarks && i < old_state.observations.size(); i++)
    {
        NewLandmark new_landmark;

        bool ok = (old_state.observations[i].has_landmark == false);

        if(ok)
        {
            ok = triangulateLandmarkInWorldFrame(
                old_state.observations[i].circle,
                old_state.camera_to_world,
                old_state.covariance.block<se3_num_parameters,se3_num_parameters>(0,0),
                new_landmark.position,
                new_landmark.covariance_landmark_landmark,
                new_landmark.covariance_landmark_camera);
        }

        if(ok)
        {
            landmark_selection.emplace_back();
            landmark_selection.back().stock = LMSTOCK_NEW_LANDMARK;
            landmark_selection.back().index = new_landmarks.size();

            new_landmark.observation = i;
            new_landmarks.push_back(std::move(new_landmark));
        }
    }

    // take from unseen existing landmarks.

    for(size_t i=0; landmark_selection.size() < myMaxLandmarks && i<old_state.landmarks.size(); i++)
    {
        if(old_state.landmarks[i].currently_seen == false)
        {
            landmark_selection.emplace_back();
            landmark_selection.back().stock = LMSTOCK_OLD_LANDMARK;
            landmark_selection.back().index = i;
        }
    }

    const size_t new_num_landmarks = landmark_selection.size();

    // compute new state.

    new_state.valid = true;
    new_state.camera_to_world = old_state.camera_to_world;
    new_state.momentum = old_state.momentum;
    new_state.landmarks.assign(new_num_landmarks, Landmark());
    new_state.observations.assign(old_state.observations.size(), Observation());
    new_state.covariance.resize(se3_num_parameters + se3_dof + 3*new_num_landmarks, se3_num_parameters + se3_dof + 3*new_num_landmarks);

    for(size_t i=0; i<old_state.observations.size(); i++)
    {
        old_state.observations[i].circle = new_state.observations[i].circle;
        old_state.observations[i].has_landmark = false;
        old_state.observations[i].landmark = 0;
    }

    new_state.covariance.block<se3_num_parameters+se3_dof,se3_num_parameters+se3_dof>(0,0) =
        old_state.covariance.block<se3_num_parameters+se3_dof,se3_num_parameters+se3_dof>(0,0);

    for(size_t i=0; i<new_num_landmarks; i++)
    {
        const auto& lms = landmark_selection[i];

        if( lms.stock == LMSTOCK_OLD_LANDMARK )
        {
            new_state.landmarks[i].position = old_state.landmarks[lms.index].position;
            new_state.landmarks[i].seen_count = old_state.landmarks[lms.index].seen_count;
            new_state.landmarks[i].currently_seen = old_state.landmarks[lms.index].currently_seen;
            new_state.landmarks[i].current_observation = old_state.landmarks[lms.index].current_observation;
            new_state.landmarks[i].updated = old_state.landmarks[lms.index].updated;

            if(old_state.landmarks[lms.index].currently_seen)
            {
                const size_t observation_index = old_state.landmarks[lms.index].current_observation;
                new_state.observations[observation_index].has_landmark = true;
                new_state.observations[observation_index].landmark = i;
            }

            new_state.covariance.block<3,3>(pose_and_momentum+3*i, pose_and_momentum+3*i) =
                old_state.covariance.block<3,3>(pose_and_momentum+3*lms.index, pose_and_momentum+3*lms.index);

            new_state.covariance.block<pose_and_momentum,3>(0, pose_and_momentum+3*i) =
                old_state.covariance.block<pose_and_momentum,3>(0, pose_and_momentum+3*lms.index);

            new_state.covariance.block<3,pose_and_momentum>(pose_and_momentum+3*i, 0) =
                old_state.covariance.block<3,pose_and_momentum>(pose_and_momentum+3*lms.index, 0);

            for(size_t j=0; j<i; j++)
            {
                const auto& olms = landmark_selection[j];

                if(olms.stock == LMSTOCK_OLD_LANDMARK)
                {
                    new_state.covariance.block<3,3>(pose_and_momentum+3*i, pose_and_momentum+3*j) =
                        old_state.covariance.block<3,3>(pose_and_momentum+3*lms.index, pose_and_momentum+3*olms.index);

                    new_state.covariance.block<3,3>(pose_and_momentum+3*j, pose_and_momentum+3*i) =
                        old_state.covariance.block<3,3>(pose_and_momentum+3*olms.index, pose_and_momentum+3*lms.index);
                }
                else
                {
                    new_state.covariance.block<3,3>(pose_and_momentum+3*i, pose_and_momentum+3*j).setZero();
                    new_state.covariance.block<3,3>(pose_and_momentum+3*j, pose_and_momentum+3*i).setZero();
                }
            }
        }
        else if( lms.stock == LMSTOCK_NEW_LANDMARK )
        {
            const size_t observation_index = new_landmarks[lms.index].observation;
            new_state.landmarks[i].position = new_landmarks[lms.index].position;
            new_state.landmarks[i].seen_count = 1;
            new_state.landmarks[i].currently_seen = true;
            new_state.landmarks[i].current_observation = observation_index;
            new_state.landmarks[i].updated = true;

            new_state.observations[observation_index].has_landmark = true;
            new_state.observations[observation_index].landmark = i;

            new_state.covariance.block<3,3>(pose_and_momentum+3*i, pose_and_momentum+3*i) =
                new_landmarks[lms.index].covariance_landmark_landmark;

            new_state.covariance.block<se3_num_parameters,3>(0, pose_and_momentum+3*i) =
                new_landmarks[lms.index].covariance_landmark_camera.transpose();
            new_state.covariance.block<se3_dof,3>(se3_num_parameters, pose_and_momentum+3*i).setZero();

            new_state.covariance.block<3,se3_num_parameters>(pose_and_momentum+3*i,0) =
                new_landmarks[lms.index].covariance_landmark_camera;
            new_state.covariance.block<3,se3_dof>(pose_and_momentum+3*i, se3_num_parameters).setZero();

            for(size_t j=0; j<i; j++)
            {
                new_state.covariance.block<3,3>(pose_and_momentum+3*i, pose_and_momentum+3*j).setZero();
                new_state.covariance.block<3,3>(pose_and_momentum+3*j, pose_and_momentum+3*i).setZero();
            }
        }
        else
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }
    }

    // commit new state.

    switchStates();

    return true;
}

bool EKFOdometry::trackingUpdate()
{
    State& old_state = currentState();
    State& new_state = workingState();

    if(old_state.valid == false)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    const int num_observations = old_state.observations.size();
    const int num_landmarks = old_state.landmarks.size();

    std::vector<ObservedLandmark> observed_landmarks;
    observed_landmarks.reserve(num_landmarks);

    for(size_t i=0; i<num_observations; i++)
    {
        if( old_state.observations[i].has_landmark )
        {
            const size_t landmark = old_state.observations[i].landmark;

            if(old_state.landmarks[landmark].currently_seen == false || old_state.landmarks[landmark].current_observation != i)
            {
                std::cerr << "Internal error!" << std::endl;
                exit(1);
            }

            const Eigen::Vector3d landmark_in_camera = old_state.camera_to_world.inverse() * old_state.landmarks[landmark].position;

            if( landmark_in_camera.z() > myLandmarkMinDistanceToCamera )
            {
                observed_landmarks.emplace_back();
                observed_landmarks.back().landmark = landmark;
                observed_landmarks.back().undistorted_circle = OdometryHelpers::undistortCircle( old_state.observations[i].circle, myCalibration );
            }
        }
    }

    // if not enough landmarks are seen, we consider we are lost.
    bool ok = ( observed_landmarks.size() >= 3 );

    if(ok)
    {
        const size_t observation_dim = 3*observed_landmarks.size();

        Eigen::VectorXd old_state_vector = old_state.toVector();

        Eigen::VectorXd predicted_observation(observation_dim);

        Eigen::VectorXd sensed_observation(observation_dim);

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(observation_dim, old_state.getDimension());

        Eigen::MatrixXd sensing_covariance = Eigen::MatrixXd::Zero(observation_dim, observation_dim);

        for(size_t i=0; i<observed_landmarks.size(); i++)
        {
            sensed_observation(3*i+0) = observed_landmarks[i].undistorted_circle[0];
            sensed_observation(3*i+1) = observed_landmarks[i].undistorted_circle[1];
            sensed_observation(3*i+2) = observed_landmarks[i].undistorted_circle[2];

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

            //

            size_t offset = 0;

            new_state.camera_to_world = Eigen::Map<const Sophus::SE3d>(new_state_vector.data() + offset);
            offset += Sophus::SE3d::num_parameters;

            new_state.momentum = Eigen::Map<const Sophus::SE3d::Tangent>(new_state_vector.data() + offset);
            offset += Sophus::SE3d::DoF;

            new_state.landmarks.swap(old_state.landmarks);
            for(ObservedLandmark& olm : observed_landmarks)
            {
                new_state.landmarks[olm.landmark].updated = true;
            }

            new_state.observations.swap(old_state.observations);

            new_state.covariance = new_covariance;

            new_state.timestamp = old_state.timestamp;

            new_state.valid = true;
        }
    }

    new_state.valid = ok;

    switchStates();

    return ok;
}

bool EKFOdometry::trackingPrediction(double timestamp, const std::vector<TrackedCircle>& circles)
{
    State& old_state = currentState();
    State& new_state = workingState();

    if(old_state.valid == false)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    const double timestep = timestamp - old_state.timestamp;

    const size_t num_circles = circles.size();

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
        new_state.valid = true;
        new_state.timestamp = timestamp;

        size_t offset = 0;

        new_state.camera_to_world = Eigen::Map<Sophus::SE3d>( predicted_state.data() + offset );
        offset += Sophus::SE3d::num_parameters;

        new_state.momentum = Eigen::Map<Sophus::SE3d::Tangent>( predicted_state.data() + offset );
        offset += Sophus::SE3d::DoF;

        new_state.landmarks.assign(old_state.landmarks.size(), Landmark());
        new_state.observations.assign(num_circles, Observation());

        for(size_t i=0; i<new_state.landmarks.size(); i++)
        {
            Landmark& old_lm = old_state.landmarks[i];
            Landmark& new_lm = new_state.landmarks[i];

            new_lm.position = old_lm.position;
            new_lm.seen_count = old_lm.seen_count;
            new_lm.currently_seen = false;
            new_lm.current_observation = 0;
            new_lm.updated = false;
        }

        for(size_t i=0; i<num_circles; i++)
        {
            Observation& obs = new_state.observations[i];
            obs.circle = circles[i].circle;

            if(circles[i].has_previous)
            {
                obs.has_landmark = old_state.observations[circles[i].previous].has_landmark;
                if(obs.has_landmark)
                {
                    obs.landmark = old_state.observations[circles[i].previous].landmark;

                    new_state.landmarks[obs.landmark].seen_count++;
                    new_state.landmarks[obs.landmark].currently_seen = true;
                    new_state.landmarks[obs.landmark].current_observation = i;
                }
                else
                {
                    obs.landmark = 0;
                }
            }
            else
            {
                obs.has_landmark = false;
                obs.landmark = 0;
            }
        }

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> noise(dim, dim);
        noise.setZero();

        noise(7,7) = myPredictionLinearMomentumSigmaRate * myPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(8,8) = myPredictionLinearMomentumSigmaRate * myPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(9,9) = myPredictionLinearMomentumSigmaRate * myPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(10,10) = myPredictionAngularMomentumSigmaRate * myPredictionAngularMomentumSigmaRate * timestep * timestep;
        noise(11,11) = myPredictionAngularMomentumSigmaRate * myPredictionAngularMomentumSigmaRate * timestep * timestep;
        noise(12,12) = myPredictionAngularMomentumSigmaRate * myPredictionAngularMomentumSigmaRate * timestep * timestep;

        new_state.covariance = jacobian * (old_state.covariance + noise) * jacobian.transpose();

        switchStates();
    }

    new_state.valid = ok;

    return ok;
}

size_t EKFOdometry::State::getDimension() const
{
    return Sophus::SE3d::num_parameters + Sophus::SE3d::DoF + 3*landmarks.size();
}

Eigen::VectorXd EKFOdometry::State::toVector() const
{
    Eigen::VectorXd ret(getDimension());

    Eigen::Map<Sophus::SE3d>( ret.data() ) = camera_to_world;

    Eigen::Map<Sophus::SE3d::Tangent>( ret.data() + Sophus::SE3d::num_parameters ) = momentum;

    for(size_t i=0; i<landmarks.size(); i++)
    {
        Eigen::Map<Eigen::Vector3d>( ret.data() + Sophus::SE3d::num_parameters + Sophus::SE3d::DoF + 3*i) = landmarks[i].position;
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
