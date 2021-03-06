#include <iostream>
#include <QFile>
#include <QDataStream>
#include "ObservationValidatorSimple.h"

const double ObservationValidatorSimple::myRadiusRatio = 0.95;
const double ObservationValidatorSimple::myDistanceThreshold = 0.03;
const size_t ObservationValidatorSimple::myNumHistogramBins = 16;

const uint32_t ObservationValidatorSimple::Histogram::mSignature = 0x11AA11BB;

ObservationValidatorSimple::ObservationValidatorSimple()
{
}

bool ObservationValidatorSimple::validate(const cv::Mat3b& image, const cv::Vec3f& circle)
{
    Histogram hist;
    bool ret = false;

    if( hist.build(myNumHistogramBins, image, circle, myRadiusRatio) )
    {
        ret = ( myReferenceHistogram.computeIntersectionWith(hist) > myDistanceThreshold );
    }

    return ret;
}

void ObservationValidatorSimple::prepareTraining()
{
    myBigHistogramBuilder.init(myNumHistogramBins);
}

void ObservationValidatorSimple::addSample(const cv::Mat3b& image, const cv::Vec3f& circle)
{
    myBigHistogramBuilder.add(image, circle, myRadiusRatio);
}

bool ObservationValidatorSimple::train()
{
    Histogram new_reference;

    const bool ret = myBigHistogramBuilder.build(new_reference);

    myBigHistogramBuilder.init(0);

    if(ret)
    {
        myReferenceHistogram = std::move(new_reference);
    }

    return ret;
}

bool ObservationValidatorSimple::load(const std::string& path)
{
    Histogram new_histogram;

    const bool ret = new_histogram.load(path);

    if(ret)
    {
        myReferenceHistogram = std::move(new_histogram);
    }

    return ret;
}

bool ObservationValidatorSimple::save(const std::string& path)
{
    return myReferenceHistogram.save(path);
}

void ObservationValidatorSimple::BigHistogramBuilder::init(size_t bins)
{
    mBins = bins;
    mHistogram.assign(bins*bins*bins, 0);
    mCount = 0;
}

void ObservationValidatorSimple::BigHistogramBuilder::add(const cv::Mat3b& image, const cv::Vec3f& circle, double gamma)
{
    const double center_x = circle[0];
    const double center_y = circle[1];
    const double radius = circle[2]*gamma;

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

bool ObservationValidatorSimple::BigHistogramBuilder::build(Histogram& hist)
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

void ObservationValidatorSimple::Histogram::set(size_t bins, std::vector<float>&& histogram)
{
    mBins = bins;
    mHistogram = std::move(histogram);
}

bool ObservationValidatorSimple::Histogram::save(const std::string& path) const
{
    bool ret = false;

    QFile file( path.c_str() );

    if( file.open(QIODevice::WriteOnly) )
    {
        QDataStream stream(&file);

        stream << (quint32) mSignature;

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

bool ObservationValidatorSimple::Histogram::load(const std::string& path)
{
    bool ret = false;

    QFile file( path.c_str() );

    if( file.open(QIODevice::ReadOnly) )
    {
        QDataStream stream(&file);

        quint32 tmp0;
        stream >> tmp0;

        if(tmp0 == mSignature)
        {
            quint64 tmp1;
            stream >> tmp1;
            mBins = tmp1;

            const size_t N = mBins*mBins*mBins;

            mHistogram.resize(N);

            for(size_t i=0; i<N; i++)
            {
                stream >> mHistogram[i];
            }

            ret = true;
        }

        file.close();
    }

    return ret;
}

ObservationValidatorSimple::Histogram::Histogram()
{
    mBins = 0;
}

size_t ObservationValidatorSimple::Histogram::getBins() const
{
    return mBins;
}

double ObservationValidatorSimple::Histogram::computeIntersectionWith(const Histogram& other)
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

bool ObservationValidatorSimple::Histogram::build(size_t bins, const cv::Mat3b& image, const cv::Vec3f& circle, double gamma)
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

