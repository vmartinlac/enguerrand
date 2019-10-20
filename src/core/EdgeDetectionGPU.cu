#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include "EdgeDetectionGPU.h"

#define EDGEDETECTIONGPU_NONZERO_GRADIENT 1
#define EDGEDETECTIONGPU_MAXIMUM_ALONG_GRADIENT 2
#define EDGEDETECTIONGPU_EDGE 4

__global__ void EdgeDetectionGPU_convertToGray(cv::cuda::PtrStepSz<uchar3> input, cv::cuda::PtrStepSz<uint8_t> output)
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x < input.cols && y < input.rows)
    {
        // copy blue channel.
        output(y,x) = input(y,x).x;
    }
}

__global__ void EdgeDetectionGPU_step0(
    cv::cuda::PtrStepSz<float> sobelx,
    cv::cuda::PtrStepSz<float> sobely,
    cv::cuda::PtrStepSz<float> gradient_norm,
    cv::cuda::PtrStepSz<uint8_t> edges,
    cv::cuda::PtrStepSz<float2> normals)
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    constexpr int margin = 3;

    if(x < sobelx.cols && y < sobelx.rows)
    {
        uint8_t& edge = edges(y,x);
        float2& normal = normals(y,x);

        edge = 0;
        normal.x = 0.0f;
        normal.y = 0.0f;

        if( margin <= x && x < sobelx.cols-margin && margin <= y && y < sobelx.rows-margin)
        {
            const float dx = sobelx(y,x);
            const float dy = sobely(y,x);

            const float alpha = hypot(dx, dy);
            gradient_norm(y,x) = alpha;

            if(alpha > 1.0e-5)
            {
                edge = EDGEDETECTIONGPU_NONZERO_GRADIENT;
                normal.x = dx/alpha;
                normal.y = dy/alpha;
            }
        }
    }
}

__global__ void EdgeDetectionGPU_step1(
    cv::cuda::PtrStepSz<float> gradient_norm,
    cv::cuda::PtrStepSz<uint8_t> edges,
    cv::cuda::PtrStepSz<float2> normals)
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x < edges.cols && y < edges.rows)
    {
        uint8_t& edge = edges(y,x);

        if( edge & EDGEDETECTIONGPU_NONZERO_GRADIENT )
        {
            const float2 N = normals(y,x);

            int dx = 0;
            int dy = 0;

            if( N.x <= -M_SQRT1_2)
            {
                dx = -1;
            }
            else if(N.x >= M_SQRT1_2)
            {
                dx = 1;
            }

            if(N.y <= -M_SQRT1_2)
            {
                dy = -1;
            }
            else if(N.y >= M_SQRT1_2)
            {
                dy = 1;
            }

            if( dx != 0 || dy != 0 )
            {
                const float my_norm = gradient_norm(y,x);
                const float norm_a = gradient_norm(y+dy, x+dx);
                const float norm_b = gradient_norm(y-dy, x-dx);

                const bool ismaximum = ( my_norm >= norm_a && my_norm >= norm_b );

                if( ismaximum )
                {
                    edge |= EDGEDETECTIONGPU_MAXIMUM_ALONG_GRADIENT;
                }
            }
        }
    }
}

__global__ void EdgeDetectionGPU_step2(
    cv::cuda::PtrStepSz<float> gradient_norm,
    cv::cuda::PtrStepSz<uint8_t> edges,
    cv::cuda::PtrStepSz<float2> normals)
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    constexpr float high_threshold = 760.0f;
    constexpr float low_threshold = 0.7f*high_threshold; //200.0f;

    if(x < edges.cols && y < edges.rows)
    {
        uint8_t& edge = edges(y,x);

        if( edge & EDGEDETECTIONGPU_MAXIMUM_ALONG_GRADIENT )
        {
            const float this_value = gradient_norm.ptr(y)[x];

            if( this_value >= high_threshold )
            {
                edge |= EDGEDETECTIONGPU_EDGE;
            }
            else if( this_value >= low_threshold )
            {
                if(gradient_norm(y+1,x+1) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;
                if(gradient_norm(y+1,x+0) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;
                if(gradient_norm(y+1,x-1) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;

                if(gradient_norm(y+0,x-1) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;
                if(gradient_norm(y+0,x+1) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;

                if(gradient_norm(y-1,x+1) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;
                if(gradient_norm(y-1,x+0) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;
                if(gradient_norm(y-1,x-1) > high_threshold) edge |= EDGEDETECTIONGPU_EDGE;
            }
        }
    }
}

__global__ void EdgeDetectionGPU_step3(cv::cuda::PtrStepSz<uint8_t> edges)
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x < edges.cols && y < edges.rows)
    {
        uint8_t& edge = edges(y,x);
        
        if( edge & EDGEDETECTIONGPU_EDGE )
        {
            edge = 255;
        }
        else
        {
            edge = 0;
        }
    }
}

EdgeDetectionGPU::EdgeDetectionGPU()
{
    myGaussianFilter = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(), 3.0);
    mySobelXFilter = cv::cuda::createSobelFilter(CV_8UC1, CV_32FC1, 1, 0, 5);
    mySobelYFilter = cv::cuda::createSobelFilter(CV_8UC1, CV_32FC1, 0, 1, 5);
}

EdgeDetectionGPU::~EdgeDetectionGPU()
{
}

void EdgeDetectionGPU::detect(const cv::Mat3b& image, cv::Mat1b& edges, cv::Mat2f& normals)
{
    const dim3 block_size(16, 16);

    const dim3 grid_size(
        (image.cols+1)/block_size.x,
        (image.rows+1)/block_size.y );

    cv::cuda::Stream stream;

    cv::cuda::GpuMat d_image;
    cv::cuda::GpuMat d_gray(image.size(), CV_8UC1);
    cv::cuda::GpuMat d_blurred(image.size(), CV_8UC1);
    cv::cuda::GpuMat d_sobel_x(image.size(), CV_32FC1);
    cv::cuda::GpuMat d_sobel_y(image.size(), CV_32FC1);
    cv::cuda::GpuMat d_gradient_norm(image.size(), CV_32FC1);
    cv::cuda::GpuMat d_edges(image.size(), CV_8UC1);
    cv::cuda::GpuMat d_normals(image.size(), CV_32FC2);

    // upload image.
    d_image.upload(image, stream);

    // convert to one-channel.
    EdgeDetectionGPU_convertToGray<<<grid_size, block_size, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(d_image, d_gray);

    // blur.
    myGaussianFilter->apply(d_gray, d_blurred, stream);

    // x and y sobel filter.
    mySobelXFilter->apply(d_blurred, d_sobel_x, stream);
    mySobelYFilter->apply(d_blurred, d_sobel_y, stream);

    // Norm of gradient and non-maxima suppression.
    EdgeDetectionGPU_step0<<<grid_size, block_size, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(d_sobel_x, d_sobel_y, d_gradient_norm, d_edges, d_normals);
    EdgeDetectionGPU_step1<<<grid_size, block_size, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(d_gradient_norm, d_edges, d_normals);
    EdgeDetectionGPU_step2<<<grid_size, block_size, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(d_gradient_norm, d_edges, d_normals);
    EdgeDetectionGPU_step3<<<grid_size, block_size, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(d_edges);

    // retrieve edges and normals.
    d_edges.download(edges, stream);
    d_normals.download(normals, stream);

    stream.waitForCompletion();
}

