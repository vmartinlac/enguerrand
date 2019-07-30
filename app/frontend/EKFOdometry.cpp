#include <fstream>
#include <iomanip>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
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

        T invK[] =
        {
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,0)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,1)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,2)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,0)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,1)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,2)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(2,0)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(2,1)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(2,2)),
        };

        T dir[3] =
        {
            cx*invK[0] + cy*invK[1] + invK[2],
            cx*invK[3] + cy*invK[4] + invK[5],
            cx*invK[6] + cy*invK[7] + invK[8]
        };

        T norm = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
        dir[0] /= norm;
        dir[1] /= norm;
        dir[2] /= norm;

        T tdir[3] =
        {
            (cx+r)*invK[0] + cy*invK[1] + invK[2],
            (cx+r)*invK[3] + cy*invK[4] + invK[5],
            (cx+r)*invK[6] + cy*invK[7] + invK[8]
        };

        T tnorm = sqrt(tdir[0]*tdir[0] + tdir[1]*tdir[1] + tdir[2]*tdir[2]);
        tdir[0] /= tnorm;
        tdir[1] /= tnorm;
        tdir[2] /= tnorm;

        T cosalpha = dir[0]*tdir[0] + dir[1]*tdir[1] + dir[2]*tdir[2];

        constexpr double threshold = std::cos(M_PI*0.3/180.0);

        if( M_PI*0.1 < cosalpha && cosalpha < threshold )
        {
            T sinalpha = sqrt( T(1.0) - cosalpha*cosalpha );

            T distance = T(mParent->mLandmarkRadius) / sinalpha;

            landmark[0] = distance*dir[0];
            landmark[1] = distance*dir[1];
            landmark[2] = distance*dir[2];

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

            if( landmark_in_camera[2] < 1.0e-4 )
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

                T angleaxis0[3];
                angleaxis0[0] = alpha;
                angleaxis0[1] = T(0.0);
                angleaxis0[2] = T(0.0);

                T angleaxis1[3];
                angleaxis1[0] = -alpha;
                angleaxis1[1] = T(0.0);
                angleaxis1[2] = T(0.0);

                T angleaxis2[3];
                angleaxis2[0] = T(0.0);
                angleaxis2[1] = alpha;
                angleaxis2[2] = T(0.0);

                T angleaxis3[3];
                angleaxis3[0] = T(0.0);
                angleaxis3[1] = -alpha;
                angleaxis3[2] = T(0.0);

                T tangentlos0[3];
                ceres::AngleAxisRotatePoint(angleaxis0, landmark_in_camera, tangentlos0);

                T tangentlos1[3];
                ceres::AngleAxisRotatePoint(angleaxis1, landmark_in_camera, tangentlos1);

                T tangentlos2[3];
                ceres::AngleAxisRotatePoint(angleaxis2, landmark_in_camera, tangentlos2);

                T tangentlos3[3];
                ceres::AngleAxisRotatePoint(angleaxis3, landmark_in_camera, tangentlos3);

                T tanpoint0[2];
                tanpoint0[0] = fx * tangentlos0[0]/tangentlos0[2] + cx;
                tanpoint0[1] = fy * tangentlos0[1]/tangentlos0[2] + cy;

                T tanpoint1[2];
                tanpoint1[0] = fx * tangentlos1[0]/tangentlos1[2] + cx;
                tanpoint1[1] = fy * tangentlos1[1]/tangentlos1[2] + cy;

                T tanpoint2[2];
                tanpoint2[0] = fx * tangentlos2[0]/tangentlos2[2] + cx;
                tanpoint2[1] = fy * tangentlos2[1]/tangentlos2[2] + cy;

                T tanpoint3[2];
                tanpoint3[0] = fx * tangentlos3[0]/tangentlos3[2] + cx;
                tanpoint3[1] = fy * tangentlos3[1]/tangentlos3[2] + cy;

                T proj_x = ( tanpoint0[0] + tanpoint1[0] + tanpoint2[0] + tanpoint3[0] ) / 4.0;
                T proj_y = ( tanpoint0[1] + tanpoint1[1] + tanpoint2[1] + tanpoint3[1] ) / 4.0;
                T proj_radius = ( ceres::abs(tanpoint1[1] -tanpoint0[1]) + ceres::abs(tanpoint3[0] - tanpoint2[0]) ) / 4.0;

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
    mMaxLandmarks = 330;
    mCalibration = calibration;
    mPredictionLinearMomentumSigmaRate = 1.0;
    mPredictionAngularMomentumSigmaRate = M_PI*10.0/180.0;
    mObservationRadiusSigma = 1.0;
    mObservationPositionSigma = 1.0;

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

        if(successful_tracking)
        {
            successful_tracking = trackingUpdate(circles);
        }

        if(successful_tracking)
        {
            successful_tracking = mappingAugment(circles);
        }
    }

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    camera_to_world = currentState().camera_to_world;
    aligned_wrt_previous = successful_tracking;

    //std::cout << "TRACKING STATUS: " << aligned_wrt_previous << std::endl;
    //newState().dump();
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
    std::cout << "timestamp: " << timestamp << std::endl;
    std::cout << "num landmarks: " << landmarks.size() << std::endl;
    std::cout << "camera_to_world: " << std::endl;
    std::cout << camera_to_world.matrix() << std::endl;
    std::cout << "linear_momentum: " << linear_momentum.transpose() << std::endl;
    std::cout << "angular_momentum: " << angular_momentum.transpose() << std::endl;
    std::cout << std::endl;
}

void EKFOdometry::reset()
{
    mInitialized = false;
}

bool EKFOdometry::triangulateLandmark(
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
        mCircleToLandmark[i].has_landmark = triangulateLandmark(
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
    std::array< cv::Vec2d, 5 > distorted;
    std::array< cv::Vec2d, 5 > undistorted;

    distorted[0][0] = c[0];
    distorted[0][1] = c[1];
    distorted[1][0] = c[0]+c[2];
    distorted[1][1] = c[1];
    distorted[2][0] = c[0]-c[2];
    distorted[2][1] = c[1];
    distorted[3][0] = c[0];
    distorted[3][1] = c[1]+c[2];
    distorted[4][0] = c[0];
    distorted[4][1] = c[1]-c[2];

    cv::undistortPoints(
        distorted,
        undistorted,
        mCalibration->cameras[0].calibration_matrix,
        mCalibration->cameras[0].distortion_coefficients,
        cv::noArray(),
        mCalibration->cameras[0].calibration_matrix);

    Eigen::Matrix<double, 5, 2, Eigen::RowMajor> eig;

    eig <<
        undistorted[0][0], undistorted[0][1],
        undistorted[1][0], undistorted[1][1],
        undistorted[2][0], undistorted[2][1],
        undistorted[3][0], undistorted[3][1],
        undistorted[4][0], undistorted[4][1];

    eig.row(1) -= eig.topRows<1>();
    eig.row(2) -= eig.topRows<1>();
    eig.row(3) -= eig.topRows<1>();
    eig.row(4) -= eig.topRows<1>();

    cv::Vec3f ret;
    ret[0] = undistorted[0][0];
    ret[1] = undistorted[0][1];
    ret[2] = ( eig.row(1).norm() + eig.row(2).norm() + eig.row(3).norm() + eig.row(4).norm() ) / 4.0;

    return ret;
}

bool EKFOdometry::mappingAugment(const std::vector<TrackedCircle>& circles)
{
    /*
    State& old_state = currentState();
    State& new_state = workingState();

    std::unique_ptr<ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>> f1
        (new ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>(new TriangulationFunction(this)));

    std::unique_ptr<ceres::AutoDiffCostFunction<AugmentationFunction, 3, 3, 4, 3>> f2
        (new ceres::AutoDiffCostFunction<AugmentationFunction, 3, 3, 4, 3>(new AugmentationFunction(this)));
    */

    /*
    const bool ok = function->Evaluate( &ceres_variable_ptr, ceres_value, &ceres_jacobian_ptr );
    */

    // TODO

    std::vector<CircleToLandmark> new_circle_to_landmark(circles.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        if( circles[i].has_previous && mCircleToLandmark[circles[i].previous].has_landmark)
        {
            new_circle_to_landmark[i].has_landmark = true;
            new_circle_to_landmark[i].landmark = mCircleToLandmark[circles[i].previous].landmark;
        }
        else
        {
            new_circle_to_landmark[i].has_landmark = false;
            new_circle_to_landmark[i].landmark = 0;
        }
    }

    mCircleToLandmark = std::move(new_circle_to_landmark);

    //new_state = std::move(old_state);

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
            for(size_t i=0; i<observed_landmarks.size(); i++)
            {
                std::cout << sensed_observation[3*i+0] << " -> " << predicted_observation[3*i+0] << std::endl;
                std::cout << sensed_observation[3*i+1] << " -> " << predicted_observation[3*i+1] << std::endl;
                std::cout << sensed_observation[3*i+2] << " -> " << predicted_observation[3*i+2] << std::endl;
                std::cout << std::endl;
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
            }

            for(size_t i=0; i<observed_landmarks.size(); i++)
            {
                new_state.landmarks[ observed_landmarks[i].landmark ].seen_count++;
            }

            new_state.covariance = new_covariance;
        }

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

    ret.segment<3>(0) = camera_to_world.translation();
    ret.segment<3>(3) = camera_to_world.unit_quaternion().vec();
    ret(6) = camera_to_world.unit_quaternion().w();
    ret.segment<3>(7) = linear_momentum;
    ret.segment<3>(10) = angular_momentum;

    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.segment<3>(13+3*i) = landmarks[i].position;
    }

    return ret;
}

