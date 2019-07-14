
#pragma once

#include "Histogram.h"
#include "VideoSource.h"
#include "OdometryCode.h"

class EngineConfig
{
public:

    HistogramPtr balls_histogram;
    VideoSourcePtr video_input;
    OdometryCodePtr odometry_code;

public:

    bool loadFromFile(const std::string& path);

    void clear();
};

using EngineConfigPtr = std::shared_ptr<EngineConfig>;

