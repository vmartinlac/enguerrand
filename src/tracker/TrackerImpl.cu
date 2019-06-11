#include <iostream>
#include <queue>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "TrackerImpl.h"

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
        if( x == 1 && y == 1 ) printf("%d\n", sizeof(sobel_norm.ptr(y)[x]));


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

TrackerImpl::TrackerImpl()
{
    mNeighbors[0] = cv::Vec2i(-1,-1);
    mNeighbors[1] = cv::Vec2i(-1,0);
    mNeighbors[2] = cv::Vec2i(-1,1);
    mNeighbors[3] = cv::Vec2i(0,-1);
    mNeighbors[4] = cv::Vec2i(0,1);
    mNeighbors[5] = cv::Vec2i(1,-1);
    mNeighbors[6] = cv::Vec2i(1,0);
    mNeighbors[7] = cv::Vec2i(1,1);

    mLineFitter.setUseOpenCV(false);
    mCircleFitter.setUseOpenCV(false);
}

void TrackerImpl::track(const cv::Mat& input_image, std::vector<TrackedLandmark>& result)
{
    mCircleFitter.setMinMaxRadius( 5.0f, float(input_image.cols)*0.5f );

    std::cout << "Detecting edges..." << std::endl;
    detectEdges(input_image);
    std::cout << "Working on edges..." << std::endl;
    findCircles();
}

void TrackerImpl::findCircles()
{
    const cv::Size image_size = mFlags.size();

    std::vector<cv::Vec2i> pixels_to_process;
    
    pixels_to_process.reserve(image_size.width*image_size.height);

    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            if(mFlags(i,j) & FLAG_EDGE)
            {
                pixels_to_process.push_back(cv::Vec2i(i,j));
            }
        }
    }

    while(pixels_to_process.empty() == false)
    {
        std::uniform_int_distribution<int> distrib(0, pixels_to_process.size()-1);

        const int selected_index = distrib(mEngine);

        const cv::Vec2i seed = pixels_to_process[selected_index];
        pixels_to_process[selected_index] = pixels_to_process.back();
        pixels_to_process.pop_back();

        if( (mFlags(seed) & FLAG_NO_SEED) == 0 )
        {
            findCircle(seed);
        }
    }
}

void TrackerImpl::findCircle(const cv::Vec2i& seed)
{
    std::vector<cv::Vec2i> patch;
    cv::Vec3f line;
    cv::Vec3f circle;
    bool ok = true;

    auto pred_radius = [this,seed] (const cv::Vec2i& neighbor)
    {
        const float dist = std::hypot(
            float(seed[0]-neighbor[0]),
            float(seed[1]-neighbor[1]));

        const float radius = 5.0f;

        return (dist < radius);
    };

    auto fit_line = [this] (const std::vector<cv::Vec2i>& patch, cv::Vec3f& line) -> bool
    {
        std::vector<cv::Vec2f> points(patch.size());
        std::transform(patch.begin(), patch.end(), points.begin(), [] (const cv::Vec2i& pt) { return static_cast<cv::Vec2f>(pt); } );

        return mLineFitter.fit(points, false, line);
    };

    auto fit_circle = [this] (const std::vector<cv::Vec2i>& patch, cv::Vec3f& circle) -> bool
    {
        std::vector<cv::Vec2f> points(patch.size());
        std::transform(patch.begin(), patch.end(), points.begin(), [] (const cv::Vec2i& pt) { return static_cast<cv::Vec2f>(pt); } );

        return mCircleFitter.fit(points, false, circle);
    };

    auto lies_on_line = [] (const cv::Vec3f& line, const cv::Vec2i& pt) -> bool
    {
        // asserts that (line[0], line[1]) vector is normalized.
        const float dist = std::fabs( line[0]*float(pt[0]) + line[1]*float(pt[1]) + line[2] );
        return (dist < 5.0f);
    };

    auto lies_on_circle = [] (const cv::Vec3f& circle, const cv::Vec2i& pt) -> bool
    {
        const float dist = std::hypot(float(pt[0]) - circle[0], float(pt[1]) - circle[1]);
        return std::fabs(dist - circle[2]) < 5.0f;
    };

    if(ok)
    {
        patch.push_back(seed);
        growPatch(patch, pred_radius);
    }

    if(ok)
    {
        ok = fit_line(patch, line);
    }

    if(ok)
    {
        for(const cv::Vec2i& pt : patch)
        {
            ok = ok && lies_on_line(line, pt);
        }
    }

    if(ok)
    {
        ok = growPrimitive(patch, fit_line, lies_on_line, line);
    }

    if(ok)
    {
        for(const cv::Vec2i& pt : patch)
        {
            mFlags(pt) |= FLAG_NO_SEED;
        }
    }

    /*
    if(ok)
    {
        ok = growPrimitive(patch, fit_circle, lies_on_circle, circle);
    }

    if(ok)
    {
        debugShowPatch("result", patch);
    }
    */
}

void TrackerImpl::debugShowPatch(const std::string& name, const std::vector<cv::Vec2i>& patch)
{
    cv::Mat3b image(mFlags.size());

    for(int i=0; i<image.rows; i++)
    {
        for(int j=0; j<image.cols; j++)
        {
            if(mFlags(i,j) & FLAG_EDGE)
            {
                image(i,j) = cv::Vec3b(128,128,128);
            }
            else
            {
                image(i,j) = cv::Vec3b(0,0,0);
            }
        }
    }

    for(const cv::Vec2i& pt : patch)
    {
        image(pt) = cv::Vec3b(0,0,255);
    }

    cv::imshow(name, image);
    cv::waitKey(0);
}

template<typename PrimitiveType, typename EstimatorType, typename ClassifierType>
bool TrackerImpl::growPrimitive<PrimitiveType,EstimatorType>(
    std::vector<cv::Vec2i>& patch,
    const EstimatorType& estimator,
    const ClassifierType& classifier,
    PrimitiveType& result)
{
    bool go_on = true;
    bool ret = true;

    auto growth_pred = [&result, &classifier] (const cv::Vec2i& pt) -> bool
    {
        return classifier(result, pt);
    };

    int num_iterations = 0;

    while(go_on)
    {
        go_on = ret = estimator(patch, result);

        if(go_on)
        {
            const size_t prev_size = patch.size();
            growPatch(patch, growth_pred);
            go_on = (patch.size() > prev_size);
        }

        num_iterations++;
    }

    return ret;
}

template<typename T>
void TrackerImpl::growPatch<T>(std::vector<cv::Vec2i>& patch, const T& pred)
{
    // TODO: set these variable as class members to avoid many memory allocations.
    std::vector<cv::Vec2i> visited;
    std::queue<cv::Vec2i> queue;

    for(cv::Vec2i& pt : patch)
    {
        mFlags(pt) |= FLAG_VISITED;
        visited.push_back(pt);
        queue.push(pt);
    }

    while( queue.empty() == false )
    {
        const cv::Vec2i point = queue.front();
        queue.pop();

        for(int k=0; k<mNeighbors.size(); k++)
        {
            const cv::Vec2i neighbor = point + mNeighbors[k];

            if( 0 <= neighbor[0] && neighbor[0] < mFlags.rows && 0 <= neighbor[1] && neighbor[1] < mFlags.cols )
            {
                const uint8_t f = mFlags(neighbor);

                if( (f & FLAG_EDGE) != 0 && (f & FLAG_VISITED) == 0 )
                {
                    if( pred(neighbor) )
                    {
                        mFlags(neighbor) |= FLAG_VISITED;
                        visited.push_back(neighbor);
                        queue.push(neighbor);
                        patch.push_back(neighbor);
                    }
                }
            }
        }
    }

    for( const cv::Vec2i& pt : visited )
    {
        mFlags(pt) &= ~FLAG_VISITED;
    }
}

void TrackerImpl::detectEdges(const cv::Mat& input_image)
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

    /*
    cv::Sobel(input_image, sobel_x, CV_32F, 1, 0, 5);
    cv::Sobel(input_image, sobel_y, CV_32F, 0, 1, 5);
    */

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

    //cv::imshow("rien", mFlags*255);
    //cv::waitKey(1);
}

