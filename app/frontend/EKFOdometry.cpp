#include "EKFOdometry.h"

EKFOdometry::EKFOdometry()
{
    mInitialized = false;
    mStateOffset = 0;
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
    }

    camera_to_world = new_state.camera_to_world;
    aligned_wrt_previous = new_state.aligned_wrt_previous;

    return true;
}

void EKFOdometry::reset()
{
    mInitialized = false;
}

void EKFOdometry::estimateLandmarkPositionInCameraFrame(
    const TrackedCircle& tc,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    // TODO;
}

void EKFOdometry::initialize(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    State& new_state)
{
    const size_t num_landmarks = circles.size();

    new_state.timestamp = timestamp;

    new_state.aligned_wrt_previous = false;

    new_state.camera_to_world = Sophus::SE3d(); // identity
    new_state.linear_momentum.setZero();
    new_state.angular_momentum.setZero();

    new_state.landmarks.resize(circles.size());

    new_state.covariance.resize(12+3*num_landmarks, 12+3*num_landmarks);
    new_state.covariance.setZero();

    // TODO: set cov(camera,camera).

    for(size_t i=0; i<num_landmarks; i++)
    {
        Eigen::Matrix3d landmark_covariance;
        estimateLandmarkPositionInCameraFrame(
            circles[i],
            new_state.landmarks[i].position_in_world,
            landmark_covariance);

        new_state.covariance.block<3,3>(12+3*i, 12+3*i) = landmark_covariance;
        // TODO: set cov(landmark, camera).
    }

    mInitialized = true;
}

