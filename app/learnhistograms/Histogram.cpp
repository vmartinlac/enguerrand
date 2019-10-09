#include "Histogram.h"

Histogram::Histogram()
{
    myBins = 0;
}

bool Histogram::build(size_t bins, const cv::Mat3b& image, const cv::Vec3f& circle, double gamma)
{
    const size_t N = bins*bins*bins;

    myHistogram.assign(N, 0);

    myBins = bins;

    myCount = 0;

    const cv::Rect ROI =
        cv::Rect( cv::Point(0,0), image.size() ) &
        cv::Rect( circle[0]-circle[2]-1, circle[1]-circle[2]-1, 2*circle[2]+1, 2*circle[2]+1 );

    for(int di=0; di<ROI.height; di++)
    {
        for(int dj=0; dj<ROI.width; dj++)
        {
            const int j = ROI.x + dj;
            const int i = ROI.y + di;

            const double radius = std::hypot( j+0.5-circle[0], i+0.5-circle[1] );

            if(radius < gamma * circle[2])
            {
                cv::Vec3i tmp = image(i,j);
                tmp *= (int) bins;
                tmp /= 256;

                const size_t index = myBins*myBins*tmp[0] + myBins*tmp[1] + tmp[2];

                myHistogram[index]++;
                myCount++;
            }
        }
    }

    return (myCount > 0);
}

size_t Histogram::getBins() const
{
    return myBins;
}

size_t Histogram::getCount() const
{
    return myCount;
}

const std::vector<uint32_t>& Histogram::refVector() const
{
    return myHistogram;
}

std::vector<uint32_t>& Histogram::refVector()
{
    return myHistogram;
}

