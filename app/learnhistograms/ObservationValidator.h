
#pragma once

#include <memory>
#include <opencv2/ml.hpp>
#include "Histogram.h"

class ObservationValidator
{
public:

    ObservationValidator();

    bool load(const std::string& path);

    void save(const std::string& path);

    bool addSample(const cv::Mat3b& image, const cv::Vec3f& circle);

    void clearSamples();

    size_t getNumSamples() const;

    bool train();

    bool validate(const cv::Mat3b& image, const cv::Vec3f& circle);

protected:

    static const double myTransformationExponent;
    static const double mySVMNu;
    static const double myRadiusRatio;
    static const size_t myNumHistogramBins;
    cv::Ptr<cv::ml::SVM> mySVM;
    std::vector<Histogram> myTrainingSet;
};

using ObservationValidatorPtr = std::shared_ptr<ObservationValidator>;

