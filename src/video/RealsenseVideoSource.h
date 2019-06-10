
#pragma once

#include <opencv2/videoio.hpp>
#include "VideoSource.h"

class RealsenseVideoSource : public VideoSource
{
public:

    RealsenseVideoSource();

    void trigger() override;

    void read(VideoFrame& frame) override;

    int getNumViews() override;
};

typedef std::shared_ptr<RealsenseVideoSource> RealsenseVideoSourcePtr;

