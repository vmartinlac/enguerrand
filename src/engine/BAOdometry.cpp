#include <ceres/ceres.h>
#include "BAOdometry.h"

struct BAOdometry::BundleAdjustment
{
    BundleAdjustment()
    {
        ;
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
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
}

bool BAOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    Sophus::SE3d& camera_to_world,
    bool& aligned_wrt_previous)
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
        aligned_wrt_previous = successful_alignment;
        camera_to_world = myLastFrame->camera_to_world;
        ret = true;
    }
    else
    {
        aligned_wrt_previous = false;
        camera_to_world = Sophus::SE3d();
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

    triangulationAdjustment();
}

void BAOdometry::reset()
{
    myObservedLandmarks.clear();
    myKeyFrames.clear();
}

BAOdometry::LandmarkPtr BAOdometry::triangulateInitialLandmark(const cv::Vec3f& circle)
{
    // TODO
    return LandmarkPtr();
}

void BAOdometry::triangulationAdjustment()
{
    // TODO
}

void BAOdometry::localBundleAdjustment()
{
    // TODO
}

void BAOdometry::PnPAdjustment()
{
    // TODO
}

