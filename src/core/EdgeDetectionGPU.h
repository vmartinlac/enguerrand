#pragma once

#ifndef WITH_CUDA
#error "EdgeDetectionGPU requires CUDA!"
#endif

#include "EdgeDetection.h"

class EdgeDetectionGPU : public EdgeDetection
{
public:

    EdgeDetectionGPU();
    virtual ~EdgeDetectionGPU();

    void detect(
        const cv::Mat3b& image,
        cv::Mat1b& edges,
        cv::Mat2f& normals) override;

protected:
};


