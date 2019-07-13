
#pragma once

#include <array>
#include <memory>

class EdgeDetectionCPU
{
public:

    EdgeDetectionCPU();

    void detect(
        const cv::Mat3b& image,
        cv::Mat1b& edges,
        cv::Mat2f& normals);

protected:

    std::array<cv::Vec2i,8> mNeighbors;
};

using EdgeDetectionCPUPtr = std::shared_ptr<EdgeDetectionCPU>;

