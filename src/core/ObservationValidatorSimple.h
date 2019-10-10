
#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include "ObservationValidator.h"

class ObservationValidatorSimple : public ObservationValidator
{
public:

    ObservationValidatorSimple();

    bool validate(const cv::Mat3b& image, const cv::Vec3f& circle) override;

    void prepareTraining() override;

    void addSample(const cv::Mat3b& image, const cv::Vec3f& circle) override;

    bool train() override;

    bool load(const std::string& path) override;

    bool save(const std::string& path) override;

protected:

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
        static const uint32_t mSignature;
    };

    class BigHistogramBuilder
    {
    public:

        void init(size_t bins);

        void add(const cv::Mat3b& image, const cv::Vec3f& circle, double gamma);

        bool build(Histogram& hist);

    protected:

        size_t mBins;
        std::vector<size_t> mHistogram;
        size_t mCount;
    };

protected:

    BigHistogramBuilder myBigHistogramBuilder;
    Histogram myReferenceHistogram;
    static const double myDistanceThreshold;
    static const double myRadiusRatio;
    static const size_t myNumHistogramBins;
};

using ObservationValidatorSimplePtr = std::shared_ptr<ObservationValidatorSimple>;
