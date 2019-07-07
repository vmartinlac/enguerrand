
#pragma once

#include <array>
#include "EdgeCirclesData.h"

class EdgeDetectionCPU
{
public:

    EdgeDetectionCPU();

    void detect(
        const cv::Mat3b& input_image,
        EdgeCirclesData& ecdata);

protected:

    std::array<cv::Vec2i,8> mNeighbors;
};
