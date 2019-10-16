#include "VariantGenerator.h"

bool VariantGenerator::operator()(const cv::Mat3b& original, cv::Mat3b& variant)
{
    return generateVariant(original, variant);
}

