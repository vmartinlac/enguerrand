
#pragma once

#include <opencv2/core.hpp>

class VariantGenerator
{
public:

    VariantGenerator() = default;
    virtual ~VariantGenerator() = default;

    bool operator()(const cv::Mat3b& original, cv::Mat3b& variant);

protected:

    virtual bool generateVariant(const cv::Mat3b& original, cv::Mat3b& variant) = 0;
};

