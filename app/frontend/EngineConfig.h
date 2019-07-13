
#pragma once

#include "Histogram.h"
#include "VideoSource.h"
#include "OdometryCode.h"

class EngineConfig
{
public:

    HistogramPtr balls_reference_histogram;
    VideoSourcePtr video;
    OdometryCodePtr odometry_code;
};

using EngineConfigPtr = std::shared_ptr<EngineConfig>;

