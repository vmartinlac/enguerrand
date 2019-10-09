
#pragma once

#include <memory>
#include <opencv2/ml.hpp>
#include "Histogram.h"

class HistogramValidator
{
public:

    HistogramValidator();

    bool load(const std::string& path);

    void save(const std::string& path);

    void addSample(const cv::Mat3b& image, const cv::Vec3f& circle);

    void clearSamples();

    bool train();

    bool validate(const cv::Mat3b& image, const cv::Vec3f& circle);

protected:

    double myAlpha;
    size_t myDimension;
    size_t myNumHistogramBins;
    cv::Ptr<cv::ml::SVM> mySVM;
};

using HistogramValidatorPtr = std::shared_ptr<HistogramValidator>;

