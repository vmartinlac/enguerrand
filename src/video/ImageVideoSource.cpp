#include <opencv2/imgcodecs.hpp>
#include "ImageVideoSource.h"

ImageVideoSource::ImageVideoSource()
{
}

bool ImageVideoSource::load(const std::string& path)
{
    mImage = cv::imread(path);
    return bool(mImage.data);
}

void ImageVideoSource::read(VideoFrame& frame)
{
    frame.setInvalid();

    if( mImage.data )
    {
        frame.setValid(0, 0.0, std::move(mImage));
        mImage = cv::Mat();
    }
}

bool ImageVideoSource::open()
{
    return bool(mImage.data);
}

void ImageVideoSource::close()
{
    mImage = cv::Mat();
}

int ImageVideoSource::getNumViews()
{
    return 1;
}

