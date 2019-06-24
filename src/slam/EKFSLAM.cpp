#include "VideoPort.h"
#include "EdgeCirclesPort.h"
#include "PoseLandmarksPort.h"
#include "EKFSLAM.h"

EKFSLAM::EKFSLAM()
{
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
    return true;
}

void EKFSLAM::finalize()
{
}

void EKFSLAM::compute(PipelinePort** ports)
{
    VideoPort* video = static_cast<VideoPort*>(ports[0]);
    EdgeCirclesPort* circles = static_cast<EdgeCirclesPort*>(ports[1]);
    PoseLandmarksPort* output = static_cast<PoseLandmarksPort*>(ports[2]);
}

