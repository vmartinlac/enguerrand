
#pragma once

#include <memory>
#include <opencv2/ml.hpp>
#include "ObservationValidator.h"

class ObservationValidatorSVM : public ObservationValidator
{
public:

    ObservationValidatorSVM();

    bool load(const std::string& path) override;

    bool save(const std::string& path) override;

    void prepareTraining() override;

    void addSample(const cv::Mat3b& image, const cv::Vec3f& circle) override;

    bool train() override;

    bool validate(const cv::Mat3b& image, const cv::Vec3f& circle) override;

protected:

    class Histogram
    {
    public:

        Histogram();

        bool build(size_t bins, const cv::Mat3b& image, const cv::Vec3f& circle, double gamma);

        size_t getBins() const;

        std::vector<uint32_t>& refVector();

        const std::vector<uint32_t>& refVector() const;

        size_t getCount() const;

    protected:

        size_t myBins;
        size_t myCount;
        std::vector<uint32_t> myHistogram;
    };

protected:

    static const double myTransformationExponent;
    static const double mySVMNu;
    static const double myRadiusRatio;
    static const size_t myNumHistogramBins;
    cv::Ptr<cv::ml::SVM> mySVM;
    std::vector<Histogram> myTrainingSet;
};

using ObservationValidatorSVMPtr = std::shared_ptr<ObservationValidatorSVM>;

