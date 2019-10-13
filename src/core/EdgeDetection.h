
#pragma once

#include <memory>
#include <opencv2/core.hpp>

class EdgeDetection;

using EdgeDetectionPtr = std::shared_ptr<EdgeDetection>;

class EdgeDetection
{
public:

    EdgeDetection() = default;

    virtual ~EdgeDetection() = default;

    virtual void detect(
        const cv::Mat3b& image,
        cv::Mat1b& edges,
        cv::Mat2f& normals) = 0;

    static EdgeDetectionPtr createEdgeDetectionAny();

    static EdgeDetectionPtr createEdgeDetectionCPU();

    static EdgeDetectionPtr createEdgeDetectionGPU();
};


