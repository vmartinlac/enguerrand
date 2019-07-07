
#pragma once

#include <memory>
#include <array>
#include <opencv2/core.hpp>

class VideoFrame
{
public:

    VideoFrame();

    VideoFrame(const VideoFrame& o);

    void setValid(int id, double timestamp, cv::Mat&& image);

    void setValid(int id, double timestamp, cv::Mat&& image0, cv::Mat&& image1);

    void setInvalid();

    bool isValid();

    int getId();

    double getTimestamp();

    int getNumViews();

    cv::Mat getView(int index=0);

protected:

    bool mIsValid;
    int mId;
    double mTimestamp;
    int mNumViews;
    std::array<cv::Mat,2> mViews;
};

typedef std::shared_ptr<VideoFrame> VideoFramePtr;

