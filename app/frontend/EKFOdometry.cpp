#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include "EKFOdometry.h"

struct EKFOdometry::TriangulationFunctor
{
    template<typename T>
    bool operator()(const T* const parameters, T* residuals)
    {
        return false;
    }
};

struct EKFOdometry::PredictionFunctor
{
};

struct EKFOdometry::UpdateFunctor
{
};

EKFOdometry::EKFOdometry(CalibrationDataPtr calibration)
{
    mLandmarkRadius = 1.0;
    mCalibration = calibration;
    mInitialized = false;
    mStateOffset = 0;

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

    if(mInitialized)
    {
        // TODO
    }
    else
    {
        initialize(timestamp, circles, new_state);
        aligned_wrt_previous = false;
    }

    camera_to_world = new_state.camera_to_world;

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
    cv::Matx<double, 5, 2> distorted_points;
    cv::Matx<double, 5, 3> los;

    distorted_points <<
        tc.circle[0], tc.circle[1];
        tc.circle[0]+tc.circle[2], tc.circle[1],
        tc.circle[0]-tc.circle[2], tc.circle[1],
        tc.circle[0], tc.circle[1]+tc.circle[2],
        tc.circle[0], tc.circle[1]-tc.circle[2],

    cv::undistortPoints(
        distorted_points,
        los,
        mCalibration->cameras[0].calibration_matrix,
        mCalibration->cameras[0].distortion_coefficients );

    Eigen::Matrix<double,5,3> loseig;
    cv::cv2eigen(los, loseig);

    loseig.row(0).normalize();
    loseig.row(1).normalize();
    loseig.row(2).normalize();
    loseig.row(3).normalize();
    loseig.row(4).normalize();

    const Eigen::Vector3d direction = loseig.topRows<1>().transpose();

    const Eigen::Array<double,4,1> cosalpha = ( loseig.bottomRows<4>() * direction ).array();

    const Eigen::Array<double,4,1> sinalpha = (1.0-cosalpha.square()).sqrt();

    const double meansinalpha = sinalpha.sum() / 4.0;

    const double distance = mLandmarkRadius / meansinalpha;

    position = distance * direction;

    // TODO: compute covariance.

    return true;
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

    std::vector<Eigen::Vector3d> landmark_positions(num_circles);
    std::vector<Eigen::Matrix3d> landmark_covariances(num_circles);
    mCirclesToLandmark.resize(num_circles);
    new_state.landmarks.clear();


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

    new_state.covariance.resize(12+3*num_landmarks, 12+3*num_landmarks);
    new_state.covariance.setZero();

    // TODO: set cov(camera, camera).

    for(size_t i=0; i<num_circles; i++)
    {
        if( mCirclesToLandmark[i].has_landmark )
        {
            const size_t j = mCirclesToLandmark[i].landmark;
            new_state.covariance.block<3,3>(12+3*j, 12+3*j) = landmark_covariances[i];
            // TODO: set cov(landmark, camera).
        }
    }

    mInitialized = true;
}

cv::Vec3f EKFOdometry::undistortCircle(const cv::Vec3f& c)
{
    cv::Matx<double, 5, 2> distorted;
    cv::Matx<double, 5, 2> undistorted;

    distorted <<
        c[0], c[1];
        c[0]+c[2], c[1],
        c[0]-c[2], c[1],
        c[0], c[1]+c[2],
        c[0], c[1]-c[2],

    cv::undistortPoints(
        distorted,
        undistorted,
        mCalibration->cameras[0].calibration_matrix,
        mCalibration->cameras[0].distortion_coefficients,
        cv::noArray(),
        mCalibration->cameras[0].calibration_matrix);

    Eigen::Matrix<double, 5, 2> eig;
    cv::cv2eigen(undistorted, eig);

    eig.row(1) -= eig.topRows<1>();
    eig.row(2) -= eig.topRows<1>();
    eig.row(3) -= eig.topRows<1>();
    eig.row(4) -= eig.topRows<1>();

    cv::Vec3f ret;
    ret[0] = undistorted(0,0);
    ret[1] = undistorted(0,1);
    ret[2] = ( eig.row(1).norm() + eig.row(2).norm() + eig.row(3).norm() + eig.row(4).norm() ) / 4.0;

    return ret;
}

