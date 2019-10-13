
#pragma once

#include <array>
#include "EdgeDetection.h"

class EdgeDetectionCPU : public EdgeDetection
{
public:

    EdgeDetectionCPU();

    void detect(
        const cv::Mat3b& image,
        cv::Mat1b& edges,
        cv::Mat2f& normals) override;

protected:

    std::array<cv::Vec2i,8> mNeighbors;
};

using EdgeDetectionCPUPtr = std::shared_ptr<EdgeDetectionCPU>;

