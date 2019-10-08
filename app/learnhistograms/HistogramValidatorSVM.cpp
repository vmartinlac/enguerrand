#include <iostream>
#include "HistogramValidatorSVM.h"

HistogramValidatorPtr HistogramValidator::createHistogramValidatorSVM()
{
    return std::make_shared<HistogramValidatorSVM>();
}

HistogramValidatorSVM::HistogramValidatorSVM()
{
    myAlpha = 0.25;
}

void HistogramValidatorSVM::load()
{
}

void HistogramValidatorSVM::save()
{
}

bool HistogramValidatorSVM::addToTrainingSet(const Histogram& histogram)
{
    std::vector<svm_node> nodes;

    bool ok = (histogram.getCount() > 0);

    if(ok)
    {
        auto& vector = histogram.refVector();

        for(size_t i=0; ok && i<vector.size(); i++)
        {
            if( vector[i] > 0 )
            {
                nodes.emplace_back();
                nodes.back().index = i;
                nodes.back().value = std::pow(double(vector[i]) / double(histogram.getCount()), myAlpha);
            }
        }

        nodes.emplace_back();
        nodes.back().index = -1;
        nodes.back().value = 0.0;

        myTrainingNodes.push_back(std::move(nodes));
    }

    return ok;
}

void HistogramValidatorSVM::clearTrainingSet()
{
    myTrainingNodes.clear();
}

void HistogramValidatorSVM::train()
{
    // TODO!
    /*
    std::cout << myTrainingNodes.size() << std::endl;
    std::cout << "Hello" << std::endl;
    */
}

HistogramValidationResult HistogramValidatorSVM::validate(const Histogram& histogram)
{
    return HISTOGRAM_VALIDATION_FAILURE;
}

