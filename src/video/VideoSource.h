
#pragma once

#include "VideoFrame.h"

class VideoSource
{
public:

    VideoSource();

    virtual void trigger() = 0;

    virtual void read(VideoFrame& frame) = 0;

    virtual int getNumViews() = 0;
};

typedef std::shared_ptr<VideoSource> VideoSourcePtr;

