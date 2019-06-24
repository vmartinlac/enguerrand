#include "VideoPort.h"
#include "EdgeCirclesPort.h"
#include "PoseLandmarksPort.h"
#include "EKFSLAM.h"

EKFSLAM::EKFSLAM()
{
    initialize();
}

const char* EKFSLAM::getName() const
{
    return "EKFSLAM";
}

size_t EKFSLAM::getNumPorts() const
{
    return 3;
}

bool EKFSLAM::initialize()
{
    mFirst = true;
    mMaxLocalMapSize = 0;
    mLocalMapSize = 0;
    mMu.resize(0);
    mSigma.resize(0,0);
    return true;
}

void EKFSLAM::finalize()
{
    mMaxLocalMapSize = 0;
    mLocalMapSize = 0;
    mMu.resize(0);
    mSigma.resize(0,0);
}

void EKFSLAM::compute(PipelinePort** ports)
{
    VideoPort* video = static_cast<VideoPort*>(ports[0]);
    EdgeCirclesPort* circles = static_cast<EdgeCirclesPort*>(ports[1]);
    PoseLandmarksPort* output = static_cast<PoseLandmarksPort*>(ports[2]);

    if(video->frame.isValid())
    {
        const double t1 = video->frame.getTimestamp();
    }
}

void EKFSLAM::predict()
{
    if(mFirst)
    {
        ;
    }
    else
    {
        ;
    }
}

void EKFSLAM::update()
{
}

