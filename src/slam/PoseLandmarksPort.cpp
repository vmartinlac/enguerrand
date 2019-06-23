#include "PoseLandmarksPort.h"

PoseLandmarksPort::PoseLandmarksPort()
{
    reset();
}

void PoseLandmarksPort::reset()
{
    available = false;
    registered_wrt_previous_frame = false;
    object_to_world = Sophus::SE3d();
    landmarks.clear();
}

