
#pragma once

#include <memory>
#include <opencv2/core.hpp>

class ObservationValidator
{
public:

    ObservationValidator();

    virtual ~ObservationValidator();

    virtual bool validate(const cv::Mat3b& image, const cv::Vec3f& circle) = 0;

    virtual bool load(const std::string& path) = 0;

    virtual bool save(const std::string& path) = 0;

    virtual void prepareTraining() = 0;

    virtual void addSample(const cv::Mat3b& image, const cv::Vec3f& circle) = 0;

    virtual bool train() = 0;
};

using ObservationValidatorPtr = std::shared_ptr<ObservationValidator>;
