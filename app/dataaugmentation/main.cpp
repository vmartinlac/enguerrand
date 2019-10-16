#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include "DataDirIterator.h"

class VariantGenerator
{
public:

    bool operator()(const cv::Mat3b& original, cv::Mat3b& variant);

protected:

    virtual bool generateVariant(const cv::Mat3b& original, cv::Mat3b& variant) = 0;
};

int main(int num_args, char** args)
{
    std::vector< std::unique_ptr<VariantGenerator> > generators;
    //generators.emplace_back(new 

    return 0;
}

