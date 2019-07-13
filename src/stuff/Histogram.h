
#pragma once

#include <memory>
#include <opencv2/core.hpp>

class Histogram
{
public:

    Histogram();

    void set(size_t bins, std::vector<float>&& histogram);

    bool build(size_t bins, const cv::Mat3b& image, const cv::Vec3f& circle, double gamma);

    bool load(const std::string& path);

    bool save(const std::string& path) const;

    size_t getBins() const;

    double computeIntersectionWith(const Histogram& other);

protected:

    size_t mBins;
    std::vector<float> mHistogram;
};

using HistogramPtr = std::shared_ptr<Histogram>;

class BigHistogramBuilder
{
public:

    void init(size_t bins);

    void add(const cv::Mat3b& image, double gamma);

    bool build(Histogram& hist);

protected:

    size_t mBins;
    std::vector<size_t> mHistogram;
    size_t mCount;
};

