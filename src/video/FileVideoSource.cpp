#include <iostream>
#include "FileVideoSource.h"

FileVideoSource::FileVideoSource()
{
    mNextFrameId = 0;
}

void FileVideoSource::setFileName(const std::string& path)
{
    mPath = path;
}

void FileVideoSource::trigger()
{
    if(mCap.isOpened())
    {
        mCap.grab();
    }
}

void FileVideoSource::read(VideoFrame& frame)
{
    frame.setInvalid();

    cv::Mat image;
    mCap.read(image);

    if(image.data)
    {
        const double timestamp = 1.0e-3 * mCap.get(cv::CAP_PROP_POS_MSEC);
        frame.setValid(mNextFrameId, timestamp, std::move(image));
        mNextFrameId++;
    }
}

bool FileVideoSource::open()
{
    mNextFrameId = 0;
    return mCap.open(mPath);
}

void FileVideoSource::close()
{
    mCap.release();
}

int FileVideoSource::getNumViews()
{
    return 1;
}

