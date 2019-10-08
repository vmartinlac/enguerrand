
#pragma once

#include <vector>
#include <svm.h>
#include "HistogramValidator.h"

class HistogramValidatorSVM : public HistogramValidator
{
public:

    HistogramValidatorSVM();

    void load() override;

    void save() override;

    bool addToTrainingSet(const Histogram& histogram) override;

    void clearTrainingSet() override;

    void train() override;

    HistogramValidationResult validate(const Histogram& histogram) override;

protected:

    std::vector< std::vector<svm_node> > myTrainingNodes;
    std::unique_ptr<svm_model> myModel;
    double myAlpha;
};
