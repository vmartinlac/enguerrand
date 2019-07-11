
#pragma once

#include <memory>
#include "VideoFrame.h"
#include "EdgeCirclesData.h"

class EngineFrame
{
public:

    VideoFrame video_frame;
    EdgeCirclesData edge_circles_data;
};

typedef std::shared_ptr<EngineFrame> EngineFramePtr;
