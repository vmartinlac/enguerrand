#include <iostream>
#include <QFile>
#include <QDataStream>
#include "Histogram.h"

void BigHistogramBuilder::init(size_t bins)
{
    mBins = bins;
    mHistogram.assign(bins*bins*bins, 0);
    mCount = 0;
}

void BigHistogramBuilder::add(const cv::Mat3b& image, double gamma)
{
    const double center_x = 0.5*image.cols;
    const double center_y = 0.5*image.rows;
    const double radius = std::min(image.cols, image.rows)*0.5*gamma;

    for(int i=0; i<image.rows; i++)
    {
        for(int j=0; j<image.cols; j++)
        {
            const double my_radius = std::hypot(j+0.5 - center_x, i+0.5 - center_y);

            if(my_radius < radius)
            {
                cv::Vec3i pix = image(i,j);

                pix *= (int) mBins;
                pix /= 256;

                const size_t index = mBins*mBins*pix[0] + mBins*pix[1] + pix[2];

                mHistogram[index]++;
                mCount++;
            }
        }
    }
}

bool BigHistogramBuilder::build(Histogram& hist)
{
    bool ret = false;

    if(mCount > 0)
    {
        const size_t N = mBins*mBins*mBins;

        std::vector<float> values(N);

        for(size_t i=0; i<N; i++)
        {
            values[i] = float(mHistogram[i]) / float(mCount);
        }

        hist.set(mBins, std::move(values));

        ret = true;
    }

    return ret;
}

void Histogram::set(size_t bins, std::vector<float>&& histogram)
{
    mBins = bins;
    mHistogram = std::move(histogram);
}

bool Histogram::save(const std::string& path) const
{
    bool ret = false;

    QFile file( path.c_str() );

    if( file.open(QIODevice::WriteOnly) )
    {
        QDataStream stream(&file);
        stream << (quint64) mBins;

        const size_t N = mBins*mBins*mBins;

        for(size_t i=0; i<N; i++)
        {
            stream << mHistogram[i];
        }

        file.close();

        ret = true;
    }

    return ret;
}

bool Histogram::load(const std::string& path)
{
    bool ret = false;

    QFile file( path.c_str() );

    if( file.open(QIODevice::ReadOnly) )
    {
        QDataStream stream(&file);

        quint64 tmp;
        stream >> tmp;
        mBins = tmp;

        const size_t N = mBins*mBins*mBins;

        mHistogram.resize(N);

        for(size_t i=0; i<N; i++)
        {
            stream >> mHistogram[i];
        }

        file.close();

        ret = true;
    }

    return ret;
}

Histogram::Histogram()
{
    mBins = 0;
}

size_t Histogram::getBins() const
{
    return mBins;
}

double Histogram::computeIntersectionWith(const Histogram& other)
{
    if( mBins != other.mBins )
    {
        std::cerr << "Internal error: compared two histograms with different number of bins!" << std::endl;
        exit(1);
    }

    const size_t N = mBins*mBins*mBins;

    double s = 0.0;

    for(size_t i=0; i<N; i++)
    {
        s += std::min(mHistogram[i], other.mHistogram[i]);
    }

    return s;
}

bool Histogram::build(size_t bins, const cv::Mat3b& image, const cv::Vec3f& circle, double gamma)
{
    bool ret = false;

    const size_t N = bins*bins*bins;

    mHistogram.assign(N, 0.0f);

    mBins = bins;

    size_t count = 0;

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

                const size_t index = mBins*mBins*tmp[0] + mBins*tmp[1] + tmp[2];

                mHistogram[index] += 1.0f;
                count++;
            }
        }
    }

    if(count > 0)
    {
        for(size_t i=0; i<N; i++)
        {
            mHistogram[i] /= float(count);
        }

        ret = true;
    }

    return ret;
}

