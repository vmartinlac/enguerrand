#include <iostream>
#include <queue>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "TrackerImpl.h"

__global__ void mykernel(cv::cuda::PtrStep<float> tmp)
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
    mCircleFitter.setUseOpenCV(true);
}

void TrackerImpl::track(const cv::Mat& input_image, std::vector<TrackedLandmark>& result)
{
    mCircleFitter.setMinMaxRadius( 5.0f, float(input_image.cols)*0.5f );

    detectEdges(input_image);
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
        return (dist < 2.0f);
    };

    auto lies_on_circle = [] (const cv::Vec3f& circle, const cv::Vec2i& pt) -> bool
    {
        const float dist = std::hypot(float(pt[0]) - circle[0], float(pt[1]) - circle[1]);
        return std::fabs(dist - circle[2]) < 2.0f;
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
        std::vector<cv::Vec2i>::iterator it = patch.begin();

        while(it != patch.end())
        {
            if( lies_on_line(line, *it) )
            {
                it++;
            }
            else
            {
                it = patch.erase(it);
            }
        }

        ok = (patch.size() >= 5);
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

    if(ok)
    {
        ok = growPrimitive(patch, fit_circle, lies_on_circle, circle);
    }

    if(ok)
    {
        debugShowPatch("result", patch);
    }
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

    cv::Mat3f sobel_x;
    cv::Mat3f sobel_y;
    cv::Mat3f sobel_norm(image_size);
    cv::Mat1f max_sobel_norm(image_size);
    cv::Mat1b channel( image_size );

    mFlags.create( image_size );

    // compute sobel x-derivative and y-derivative.

    cv::Sobel(input_image, sobel_x, CV_32F, 1, 0, 5);
    cv::Sobel(input_image, sobel_y, CV_32F, 0, 1, 5);

    // compute norm of gradient.


    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            sobel_norm(i,j)[0] = std::hypot(sobel_x(i,j)[0], sobel_y(i,j)[0]);
            sobel_norm(i,j)[1] = std::hypot(sobel_x(i,j)[1], sobel_y(i,j)[1]);
            sobel_norm(i,j)[2] = std::hypot(sobel_x(i,j)[2], sobel_y(i,j)[2]);
            max_sobel_norm(i,j) = std::max(sobel_norm(i,j)[0], std::max( sobel_norm(i,j)[1], sobel_norm(i,j)[2] ));
        }
    }

    // for each pixel, compute the channel on which its gradient has the largest margnitude.

    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            const cv::Vec3f col = sobel_norm(i,j);

            int c = 0;
            float value = col[0];

            if( col[1] > value )
            {
                c = 1;
                value = col[1];
            }

            if( col[2] > value )
            {
                c = 2;
                value = col[2];
            }

            channel(i,j) = c;
        }
    }

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
    //cv::waitKey(0);
}

