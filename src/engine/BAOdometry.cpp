#include <ceres/ceres.h>
#include "BAOdometry.h"

struct BAOdometry::LandmarkTriangulation
{
    template<typename T>
    bool operator()(const T* const parameters, T* residuals)
    {
        return false;
    }
};

BAOdometry::BAOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
    myLandmarkRadius = 1.0;
    myMaxKeyFrames = 10;
    myMaxProjections = 50;
    clear();
}

bool BAOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    Sophus::SE3d& camera_to_world,
    bool& aligned_wrt_previous)
{
    return false;
}

void BAOdometry::reset()
{
    clear();
}

void BAOdometry::clear()
{
    const size_t max_landmarks = myMaxKeyFrames*myMaxProjections;
    myLandmarks.resize(max_landmarks);
    myAvailableLandmarks.resize(max_landmarks);
    for(size_t i=0; i<max_landmarks; i++)
    {
        myLandmarks[i].reference_count = 0;
        myAvailableLandmarks[i] = i;
    }
    myKeyFrames.clear();
}

