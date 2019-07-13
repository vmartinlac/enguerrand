
#pragma once

#include <opencv2/core.hpp>

class Histogram
{
public:

    void set(size_t bins, std::vector<float>&& histogram);

    bool load(const std::string& path);

    bool save(const std::string& path) const;

protected:

    size_t mBins;
    std::vector<float> mHistogram;
};

class HistogramBuilder
{
public:

    void init(size_t bins);

    void add(const cv::Mat3b& image);

    void build(Histogram& hist);

protected:

    size_t mBins;
    std::vector<size_t> mHistogram;
    size_t mCount;
};

