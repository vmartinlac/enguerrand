
#pragma once

#include "VideoFrame.h"

class VideoSource
{
public:

    VideoSource();

    virtual bool open() = 0;

    virtual void close() = 0;

    virtual void trigger() = 0;

    virtual void read(VideoFrame& frame) = 0;

    virtual int getNumViews() = 0;
};

typedef std::shared_ptr<VideoSource> VideoSourcePtr;

