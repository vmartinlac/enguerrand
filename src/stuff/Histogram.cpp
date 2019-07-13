#include <QFile>
#include <QDataStream>
#include "Histogram.h"

void HistogramBuilder::init(size_t bins)
{
    mBins = bins;
    mHistogram.assign(bins*bins*bins, 0);
    mCount = 0;
}

void HistogramBuilder::add(const cv::Mat3b& image)
{
    const double center_x = 0.5*image.cols;
    const double center_y = 0.5*image.rows;
    const double radius = std::min(image.cols, image.rows)*0.5*0.9;

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

void HistogramBuilder::build(Histogram& hist)
{
    const size_t N = mBins*mBins*mBins;

    std::vector<float> values(N);

    for(size_t i=0; i<N; i++)
    {
        values[i] = float(mHistogram[i]) / float(mCount);
    }

    hist.set(mBins, std::move(values));
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

        for(size_t i=0; i<N; i++)
        {
            stream << mHistogram[i];
        }

        file.close();

        ret = true;
    }

    return ret;
}

