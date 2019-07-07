#include "VideoFrame.h"

VideoFrame::VideoFrame()
{
    mIsValid = false;
}

VideoFrame::VideoFrame(const VideoFrame& other)
{
    mIsValid = other.mIsValid;
    mId = other.mId;
    mTimestamp = other.mTimestamp;;
    mNumViews = other.mNumViews;
    mViews = other.mViews;
}

void VideoFrame::setValid(int id, double timestamp, cv::Mat&& image)
{
    mIsValid = true;
    mId = id;
    mTimestamp = timestamp;
    mNumViews = 1;
    mViews[0] = image;
}

void VideoFrame::setValid(int id, double timestamp, cv::Mat&& image0, cv::Mat&& image1)
{
    mIsValid = true;
    mId = id;
    mTimestamp = timestamp;
    mNumViews = 2;
    mViews[0] = image0;
    mViews[1] = image1;
}

void VideoFrame::setInvalid()
{
    mIsValid = false;
}

bool VideoFrame::isValid()
{
    return mIsValid;
}

int VideoFrame::getId()
{
    if( mIsValid == false)
    {
        throw std::runtime_error("internal error");
    }
    return mId;
}

double VideoFrame::getTimestamp()
{
    if( mIsValid == false)
    {
        throw std::runtime_error("internal error");
    }
    return mTimestamp;
}

int VideoFrame::getNumViews()
{
    if( mIsValid == false)
    {
        throw std::runtime_error("internal error");
    }
    return mNumViews;
}

cv::Mat VideoFrame::getView(int index)
{
    if( mIsValid == false)
    {
        throw std::runtime_error("internal error");
    }
    return mViews[index];
}

