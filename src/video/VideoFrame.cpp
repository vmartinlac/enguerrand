#include "VideoFrame.h"

VideoFrame::VideoFrame()
{
    mIsValid = false;
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
    return mId;
}

double VideoFrame::getTimestamp()
{
    return mTimestamp;
}

int VideoFrame::getNumViews()
{
    return mNumViews;
}

cv::Mat VideoFrame::getView(int index)
{
    return mViews[index];
}

