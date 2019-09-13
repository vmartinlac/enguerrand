
#pragma once

#include <opencv2/videoio.hpp>
#include "VideoSource.h"

class ImageVideoSource : public SynchronousVideoSource
{
public:

    ImageVideoSource();

    bool load(const std::string& filename);

    bool open() override;

    void close() override;

    void read(VideoFrame& frame) override;

    int getNumViews() override;

protected:

    cv::Mat mImage;
};

typedef std::shared_ptr<ImageVideoSource> ImageVideoSourcePtr;

