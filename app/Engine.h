#pragma once

#include <tbb/flow_graph.h>
#include <thread>
#include <mutex>
#include "VideoSource.h"
#include "OdometryCode.h"

class Engine
{
public:

    bool exec(VideoSourcePtr video, OdometryCodePtr odometry_code);
};
