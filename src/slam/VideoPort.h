
#pragma once

#include <opencv2/core.hpp>
#include <array>
#include "VideoFrame.h"
#include "PipelinePort.h"

class VideoPort : public PipelinePort
{
public:

    VideoPort();

    void reset() override;

public:

    VideoFrame frame;
};

