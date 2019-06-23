#include "EdgeDetectionGPU.h"

#include "EdgeCirclesPort.h"
#include "EdgeDetectionGPU.h"
#include "VideoPort.h"

EdgeDetectionGPU::EdgeDetectionGPU()
{
}

size_t EdgeDetectionGPU::getNumPorts() const
{
    return 2;
}

bool EdgeDetectionGPU::initialize()
{
    return true;
}

void EdgeDetectionGPU::finalize()
{
}

void EdgeDetectionGPU::pullGPU(PipelinePort** ports)
{
    VideoPort* video = static_cast<VideoPort*>(ports[0]);
    EdgeCirclesPort* edge = static_cast<EdgeCirclesPort*>(ports[1]);
}

void EdgeDetectionGPU::pushGPU(PipelinePort** ports)
{
    VideoPort* video = static_cast<VideoPort*>(ports[0]);
    EdgeCirclesPort* edge = static_cast<EdgeCirclesPort*>(ports[1]);

    edge->circles.clear();
}

/*

#include <iostream>
#include <queue>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "TrackerGPUImpl.h"

__global__ void computeSobelNorm(
    cv::cuda::PtrStepSz<float3> sobel_x,
    cv::cuda::PtrStepSz<float3> sobel_y,
    cv::cuda::PtrStepSz<float3> sobel_norm,
    cv::cuda::PtrStepSz<float> max_sobel_norm,
    cv::cuda::PtrStepSz<uint8_t> channel)
{
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if( x < sobel_x.cols && y < sobel_x.rows )
    {
        const float blue_x = sobel_x.ptr(y)[x].x;
        const float green_x = sobel_x.ptr(y)[x].y;
        const float red_x = sobel_x.ptr(y)[x].z;

        const float blue_y = sobel_y.ptr(y)[x].x;
        const float green_y = sobel_y.ptr(y)[x].y;
        const float red_y = sobel_y.ptr(y)[x].z;

        const float blue_norm = hypot(blue_x, blue_y);
        const float green_norm = hypot(green_x, green_y);
        const float red_norm = hypot(red_x, red_y);

        sobel_norm.ptr(y)[x].x = blue_norm;
        sobel_norm.ptr(y)[x].y = green_norm;
        sobel_norm.ptr(y)[x].z = red_norm;


        uint8_t c = 0;
        float max = blue_norm;

        if(green_norm > max)
        {
            max = green_norm;
            c = 1;
        }

        if(red_norm > max)
        {
            max = red_norm;
            c = 2;
        }

        channel.ptr(y)[x] = c;
        max_sobel_norm.ptr(y)[x] = max;
    }
}

__global__ void nonMaximaSuppression()
{
}

TrackerGPUImpl::TrackerGPUImpl()
{
    mNeighbors[0] = cv::Vec2i(-1,-1);
    mNeighbors[1] = cv::Vec2i(-1,0);
    mNeighbors[2] = cv::Vec2i(-1,1);
    mNeighbors[3] = cv::Vec2i(0,-1);
    mNeighbors[4] = cv::Vec2i(0,1);
    mNeighbors[5] = cv::Vec2i(1,-1);
    mNeighbors[6] = cv::Vec2i(1,0);
    mNeighbors[7] = cv::Vec2i(1,1);
}

void TrackerGPUImpl::track(const cv::Mat& input_image, std::vector<TrackedLandmark>& result)
{
    std::cout << "Detecting edges..." << std::endl;
    detectEdges(input_image);
}

void TrackerGPUImpl::detectEdges(const cv::Mat& input_image)
{
    if( input_image.type() != CV_8UC3 ) throw "Internal error: incorrect image format.";

    const cv::Size image_size = input_image.size();

    cv::cuda::GpuMat d_input;
    cv::cuda::GpuMat d_sobel_x;
    cv::cuda::GpuMat d_sobel_y;
    cv::cuda::GpuMat d_sobel_norm(image_size, CV_32FC3);
    cv::cuda::GpuMat d_max_sobel_norm(image_size, CV_32FC1);
    cv::cuda::GpuMat d_channel( image_size, CV_8UC1 );
    cv::cuda::GpuMat d_flags( image_size, CV_8UC1 );


    cv::Mat3f sobel_x;
    cv::Mat3f sobel_y;
    cv::Mat3f sobel_norm(image_size);
    cv::Mat1f max_sobel_norm(image_size);
    cv::Mat1b channel(image_size);
    mFlags.create(image_size);

    //mFlags.create( image_size );

    // compute sobel x-derivative and y-derivative.

    static cv::Ptr<cv::cuda::Filter> filterx;
    static cv::Ptr<cv::cuda::Filter> filtery;
    if( filterx.get() == nullptr ) filterx = cv::cuda::createSobelFilter(CV_8UC3, CV_32FC3, 1, 0, 5);
    if( filtery.get() == nullptr ) filtery = cv::cuda::createSobelFilter(CV_8UC3, CV_32FC3, 0, 1, 5);

    cv::cuda::Event e1;
    cv::cuda::Stream s1;
    d_input.upload(input_image, s1);
    e1.record(s1);

    cv::cuda::Stream s2x;
    cv::cuda::Stream s2y;
    cv::cuda::Event e2x;
    cv::cuda::Event e2y;

    s2x.waitEvent(e1);
    s2y.waitEvent(e1);

    filterx->apply(d_input, d_sobel_x, s2x);
    filtery->apply(d_input, d_sobel_y, s2y);

    d_sobel_x.download(sobel_x, s2x);
    d_sobel_y.download(sobel_y, s2y);

    e2x.record(s2x);
    e2y.record(s2y);

    cv::cuda::Stream s3;

    s3.waitEvent(e2x);
    s3.waitEvent(e2y);

    dim3 blocks( (image_size.width+15)/16, (image_size.height+15)/16 );
    dim3 threads( 16, 16 );

    computeSobelNorm<<< blocks, threads, 0, cv::cuda::StreamAccessor::getStream(s3)  >>>(d_sobel_x, d_sobel_y, d_sobel_norm, d_max_sobel_norm, d_channel);

    d_sobel_norm.download(sobel_norm, s3);
    d_max_sobel_norm.download(max_sobel_norm, s3);
    d_channel.download(channel, s3);

    s3.waitForCompletion();

    //cv::Sobel(input_image, sobel_x, CV_32F, 1, 0, 5);
    //cv::Sobel(input_image, sobel_y, CV_32F, 0, 1, 5);

    // non-maximum suppression.

    const int margin = 3;

    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            uint8_t flag = 0;

            if( margin <= i && i+margin < image_size.height && margin <= j && j+margin < image_size.width )
            {
                const int c = channel(i,j);
                const float gradient_x = sobel_x(i,j)[c];
                const float gradient_y = sobel_y(i,j)[c];
                const float gradient_norm = sobel_norm(i,j)[c];

                const double epsilon = 1.0e-5;

                if( gradient_norm > epsilon )
                {
                    int dx = 0;
                    int dy = 0;

                    if(gradient_x/gradient_norm <= -M_SQRT1_2)
                    {
                        dx = -1;
                    }
                    else if(gradient_x/gradient_norm >= M_SQRT1_2)
                    {
                        dx = 1;
                    }

                    if(gradient_y/gradient_norm <= -M_SQRT1_2)
                    {
                        dy = -1;
                    }
                    else if(gradient_y/gradient_norm >= M_SQRT1_2)
                    {
                        dy = 1;
                    }

                    if( dx != 0 || dy != 0 )
                    {
                        const bool ismax = ( gradient_norm >= sobel_norm(i+dy,j+dx)[c] ) && ( gradient_norm >= sobel_norm(i-dy,j-dx)[c] );

                        if( ismax )
                        {
                            flag = FLAG_EDGE;
                        }
                    }
                }
            }

            mFlags(i,j) = flag;
        }
    }

    //cv::imshow("rien", mFlags*255);
    //cv::waitKey(0);

    {
        // TODO: use dichotomy to compute quantiles.

        std::vector<float> values(image_size.width*image_size.height);
        std::copy(max_sobel_norm.begin(), max_sobel_norm.end(), values.begin());

        std::sort(values.begin(), values.end());

        int k1 = static_cast<int>( values.size()*0.97 );
        k1 = std::min<int>(values.size(), std::max(0,k1));
        const float high_threshold = values[k1];

        int k2 = static_cast<int>( values.size()*0.92 );
        k2 = std::min<int>(values.size(), std::max(0,k2));
        const float low_threshold = values[k2];

        for(int i=0; i<image_size.height; i++)
        {
            for(int j=0; j<image_size.width; j++)
            {
                if( mFlags(i,j) == FLAG_EDGE )
                {
                    const float this_value = max_sobel_norm(i,j);

                    if( this_value < high_threshold )
                    {
                        mFlags(i,j) = 0;

                        if( this_value >= low_threshold )
                        {
                            for(const cv::Vec2i& delta : mNeighbors)
                            {
                                const cv::Vec2i that_point(i+delta[0], j+delta[1]);

                                if( 0 <= that_point[0] && that_point[0] < image_size.height && 0 <= that_point[1] && that_point[1] < image_size.width )
                                {
                                    const float that_value = max_sobel_norm(that_point[0], that_point[1]);

                                    if(that_value > high_threshold)
                                    {
                                        mFlags(i,j) = FLAG_EDGE;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    cv::imshow("rien", mFlags*255);
    cv::waitKey(1);
}
*/

