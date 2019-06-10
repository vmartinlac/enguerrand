#pragma once

#include <opencv2/videoio.hpp>
#include "VideoSource.h"

class OpenCVVideoSource : public VideoSource
{
public:

    OpenCVVideoSource(cv::Ptr<cv::VideoCapture> cap);

    cv::Ptr<cv::VideoCapture> getVideoCapture();

    void trigger() override;

    void read(VideoFrame& frame) override;

    int getNumViews() override;
};

typedef std::shared_ptr<OpenCVVideoSource> OpenCVVideoSourcePtr;

