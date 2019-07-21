#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
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

        if( M_PI*0.1 < cosalpha && cosalpha < threshold)
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

    PredictionFunction(EKFOdometry* parent)
    {
        mParent = parent;
        mTimestep = 1.0;
    }

    void setTimestep(double dt)
    {
        mTimestep = dt;
    }

    template<typename T>
    bool operator()(const T* const old_state, T* new_state) const
    {
        return false;
    }
};

struct EKFOdometry::ObservationFunction
{
    EKFOdometry* mParent;

    ObservationFunction(EKFOdometry* parent)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(const T* const old_state, T* new_state) const
    {
        return false;
    }
};

EKFOdometry::EKFOdometry(CalibrationDataPtr calibration)
{
    mLandmarkRadius = 1.0;
    mMaxLandmarks = 330;
    mCalibration = calibration;
    mInitialized = false;
    mStateOffset = 0;

    mTriangulationFunction = new TriangulationFunction(this);
    mPredictionFunction = new PredictionFunction(this);
    mObservationFunction = new ObservationFunction(this);

    mTriangulationCostFunction.reset(new ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>(mTriangulationFunction));
    //mPredictionCostFunction.reset(new ceres::AutoDiffCostFunction<PredictionFunction, 13, 13>(mPredictionFunction));
    //mObservationCostFunction.reset(new ceres::AutoDiffCostFunction<ObservationFunction, 13, 13>(mObservationFunction));

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
    const int new_state_offset = (mStateOffset+1)%2;

    State& old_state = mState[mStateOffset];
    State& new_state = mState[new_state_offset];

    mStateOffset = new_state_offset;

    bool successful_tracking = false;

    if(mInitialized)
    {
        successful_tracking = track(timestamp, circles, old_state, new_state);
    }

    if(successful_tracking == false)
    {
        initialize(timestamp, circles, new_state);
    }

    camera_to_world = new_state.camera_to_world;
    aligned_wrt_previous = successful_tracking;

    return true;
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

    cv::Vec3f undistorted = undistortCircle(tc.circle);

    ceres_variable[0] = undistorted[0];
    ceres_variable[1] = undistorted[1];
    ceres_variable[2] = undistorted[2];

    const bool ok = mTriangulationCostFunction->Evaluate( &ceres_variable_ptr, ceres_value, &ceres_jacobian_ptr );

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

void EKFOdometry::initialize(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    State& new_state)
{
    const size_t num_circles = circles.size();

    new_state.timestamp = timestamp;
    new_state.camera_to_world = Sophus::SE3d(); // identity
    new_state.linear_momentum.setZero();
    new_state.angular_momentum.setZero();
    new_state.landmarks.clear();

    std::vector<Eigen::Vector3d> landmark_positions(num_circles);
    std::vector<Eigen::Matrix3d> landmark_covariances(num_circles);
    mCirclesToLandmark.resize(num_circles);

    // TODO: enforce maximum number of landmarks.

    for(size_t i=0; i<num_circles; i++)
    {
        mCirclesToLandmark[i].has_landmark = triangulateLandmark(
            circles[i],
            landmark_positions[i],
            landmark_covariances[i]);

        if( mCirclesToLandmark[i].has_landmark )
        {
            mCirclesToLandmark[i].landmark = new_state.landmarks.size();

            new_state.landmarks.emplace_back();
            new_state.landmarks.back().position = landmark_positions[i];
        }
        else
        {
            mCirclesToLandmark[i].landmark = 0; // static_cast<size_t>(-1);
        }
    }

    const size_t num_landmarks = new_state.landmarks.size();

    new_state.covariance.resize(13+3*num_landmarks, 13+3*num_landmarks);
    new_state.covariance.setZero();

    /*
    const double initial_sigma_position = 1.0e-3;
    const double initial_sigma_attitude = 1.0e-3;
    const double initial_sigma_linear_momentum = 1.0e-3;
    const double initial_sigma_angular_momentum = 1.0e-3;

    new_state.covariance.block<3,3>(0,0).diagonal().fill(initial_sigma_position*initial_sigma_position);
    new_state.covariance.block<4,4>(3,3).diagonal().fill(initial_sigma_attitude*initial_sigma_attitude);
    new_state.covariance.block<3,3>(7,7).diagonal().fill(initial_sigma_linear_momentum*initial_sigma_linear_momentum);
    new_state.covariance.block<3,3>(10,10).diagonal().fill(initial_sigma_angular_momentum*initial_sigma_angular_momentum);
    */

    for(size_t i=0; i<num_circles; i++)
    {
        if( mCirclesToLandmark[i].has_landmark )
        {
            const size_t j = mCirclesToLandmark[i].landmark;
            new_state.covariance.block<3,3>(13+3*j, 13+3*j) = landmark_covariances[i];
        }
    }

    //std::cout << "num_landmarks: " << num_landmarks << std::endl;

    //mInitialized = true;
    //std::cout << new_state.covariance << std::endl;
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

bool EKFOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    State& old_state,
    State& new_state)
{
    const double dt = timestamp - old_state.timestamp;

    new_state.timestamp = timestamp;

    // predict.

    return true;
}

