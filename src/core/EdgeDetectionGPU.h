#pragma once

#ifndef WITH_CUDA
#error "EdgeDetectionGPU requires CUDA!"
#endif

#include <opencv2/cudafilters.hpp>
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

    cv::Ptr<cv::cuda::Filter> myGaussianFilter;
    cv::Ptr<cv::cuda::Filter> mySobelXFilter;
    cv::Ptr<cv::cuda::Filter> mySobelYFilter;
};


