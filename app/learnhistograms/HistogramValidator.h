
#pragma once

#include <memory>
#include "Histogram.h"

enum HistogramValidationResult
{
    HISTOGRAM_VALIDATION_ACCEPTED,
    HISTOGRAM_VALIDATION_REJECTED,
    HISTOGRAM_VALIDATION_FAILURE
};

class HistogramValidator;

using HistogramValidatorPtr = std::shared_ptr<HistogramValidator>;

class HistogramValidator
{
public:

    static HistogramValidatorPtr createHistogramValidatorSVM();

    HistogramValidator();

    virtual void load() = 0;

    virtual void save() = 0;

    virtual bool addToTrainingSet(const Histogram& histogram) = 0;

    virtual void clearTrainingSet() = 0;

    virtual void train() = 0;

    virtual HistogramValidationResult validate(const Histogram& histogram) = 0;
};

