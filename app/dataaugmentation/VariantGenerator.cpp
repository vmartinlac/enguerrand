#include "VariantGenerator.h"

bool VariantGenerator::operator()(const cv::Mat3b& original, cv::Mat3b& variant)
{
    if(variant.size() != original.size())
    {
        variant.create( variant.size() );
    }

    return generateVariant(original, variant);
}

