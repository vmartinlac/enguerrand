
#pragma once

#include <opencv2/videoio.hpp>
#include "VideoSource.h"

class RealsenseVideoSource : public AsynchronousVideoSource
{
public:

};

typedef std::shared_ptr<RealsenseVideoSource> RealsenseVideoSourcePtr;

