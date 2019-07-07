#include "EdgeCirclesPort.h"

EdgeCirclesPort::EdgeCirclesPort()
{
    available = false;
}

void EdgeCirclesPort::reset()
{
    available = false;
    flags = cv::Mat1b();
    normals = cv::Mat2f();
    circles.clear();
}

