
#pragma once

#include <vector>
#include <opencv2/core.hpp>

class Histogram
{
public:

    Histogram();

    bool build(size_t bins, const cv::Mat3b& image, const cv::Vec3f& circle, double gamma);

    size_t getBins() const;

    std::vector<uint32_t>& refVector();

    size_t getCount() const;

protected:

    size_t myBins;
    size_t myCount;
    std::vector<uint32_t> myHistogram;
};
