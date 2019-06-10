
#pragma once

#include <opencv2/videoio.hpp>
#include "VideoSource.h"

class GenICamVideoSource : public VideoSource
{
public:

    GenICamVideoSource();

    void trigger() override;

    void read(VideoFrame& frame) override;

    int getNumViews() override;
};

typedef std::shared_ptr<GenICamVideoSource> GenICamVideoSourcePtr;

