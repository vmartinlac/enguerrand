#pragma once

#include <opencv2/videoio.hpp>
#include "VideoSource.h"

class FileVideoSource : public SynchronousVideoSource
{
public:

    FileVideoSource();

    void setFileName(const std::string& filename);

    bool open() override;

    void close() override;

    void read(VideoFrame& frame) override;

    int getNumViews() override;

protected:

    std::string mPath;
    cv::VideoCapture mCap;
    int mNextFrameId;
};

typedef std::shared_ptr<FileVideoSource> FileVideoSourcePtr;

