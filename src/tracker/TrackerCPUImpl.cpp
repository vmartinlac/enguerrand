#include <iostream>
#include <queue>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "LineFitter.h"
#include "CircleFitter.h"
#include "TrackerCPUImpl.h"

TrackerCPUImpl::TrackerCPUImpl()
{
    mNeighbors[0] = cv::Vec2i(-1,-1);
    mNeighbors[1] = cv::Vec2i(-1,0);
    mNeighbors[2] = cv::Vec2i(-1,1);
    mNeighbors[3] = cv::Vec2i(0,-1);
    mNeighbors[4] = cv::Vec2i(0,1);
    mNeighbors[5] = cv::Vec2i(1,-1);
    mNeighbors[6] = cv::Vec2i(1,0);
    mNeighbors[7] = cv::Vec2i(1,1);

    mMinRadius = 5.0f;
    mMaxRadius = 600.0f;
}

void TrackerCPUImpl::track(const cv::Mat& input_image, std::vector<TrackedLandmark>& result)
{
    //std::vector<cv::Vec2f> centers;
    std::vector<cv::Vec3f> circles;

    detectEdges(input_image);
    //detectCirclesCentersWithHough(centers);
    detectCirclesWithRANSAC(circles);

    cv::Mat tmp = input_image.clone();
    for(cv::Vec3f& c : circles)
    {
        cv::circle(tmp, cv::Point2f(c[0], c[1]), c[2], cv::Scalar(255, 255, 255));
    }
    cv::imshow("rien", tmp);
    cv::waitKey(0);
}

void TrackerCPUImpl::detectEdges(const cv::Mat& input_image)
{
    if( input_image.type() != CV_8UC3 ) throw "Internal error: incorrect image format.";

    const cv::Size image_size = input_image.size();

    cv::Mat1b gray;
    cv::Mat1b blurred;
    cv::Mat1f sobel_x;
    cv::Mat1f sobel_y;
    cv::Mat1f edgeness(image_size);

    mFlags.create( image_size );
    mNormals.create( image_size );

    // convert to gray.

    {
        gray.create(image_size);
        const int from_to[2] = {0, 0};
        cv::mixChannels(&input_image, 1, &gray, 1, from_to, 1);
    }
    cv::GaussianBlur(gray, gray, cv::Size(), 3.0);
    //cv::imshow("rien", gray);
    //cv::waitKey(0);

    // compute sobel x-derivative and y-derivative.

    cv::Sobel(gray, sobel_x, CV_32F, 1, 0, 5);
    cv::Sobel(gray, sobel_y, CV_32F, 0, 1, 5);

    // compute norm of gradient.

    const int margin = 3;

    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            mFlags(i,j) = 0;
            mNormals(i,j)[0] = 0.0f;
            mNormals(i,j)[1] = 0.0f;

            if( margin <= i && i+margin < image_size.height && margin <= j && j+margin < image_size.width )
            {
                const float gradient_norm = std::hypot(sobel_x(i,j), sobel_y(i,j));

                edgeness(i,j) = gradient_norm;

                const float epsilon = 1.0e-5;

                if( gradient_norm > epsilon )
                {
                    mFlags(i,j) = FLAG_NONZERO_GRADIENT;
                    mNormals(i,j)[0] = sobel_x(i,j) / gradient_norm;
                    mNormals(i,j)[1] = sobel_y(i,j) / gradient_norm;
                }
            }
        }
    }

    // non-maximum suppression.

    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            if( mFlags(i,j) & FLAG_NONZERO_GRADIENT )
            {
                const cv::Vec2f N = mNormals(i,j);

                int dx = 0;
                int dy = 0;

                if( N[0] <= -M_SQRT1_2)
                {
                    dx = -1;
                }
                else if(N[0] >= M_SQRT1_2)
                {
                    dx = 1;
                }

                if(N[1] <= -M_SQRT1_2)
                {
                    dy = -1;
                }
                else if(N[1] >= M_SQRT1_2)
                {
                    dy = 1;
                }

                if( dx != 0 || dy != 0 )
                {
                    const bool ismaximum = ( edgeness(i,j) >= edgeness(i+dy,j+dx) && edgeness(i,j) >= edgeness(i-dy,j-dx) );

                    if( ismaximum )
                    {
                        mFlags(i,j) |= FLAG_MAXIMUM_ALONG_GRADIENT;
                    }
                }
            }
        }
    }

    {
        const float high_threshold = 760.0f;
        const float low_threshold = 0.7f*high_threshold; //200.0f;

        for(int i=0; i<image_size.height; i++)
        {
            for(int j=0; j<image_size.width; j++)
            {
                if( mFlags(i,j) & FLAG_MAXIMUM_ALONG_GRADIENT )
                {
                    const float this_value = edgeness(i,j);

                    if( this_value >= high_threshold )
                    {
                        mFlags(i,j) |= FLAG_EDGE;
                    }
                    else if( this_value >= low_threshold )
                    {
                        for(const cv::Vec2i& delta : mNeighbors)
                        {
                            const cv::Vec2i that_point(i+delta[0], j+delta[1]);

                            if( 0 <= that_point[0] && that_point[0] < image_size.height && 0 <= that_point[1] && that_point[1] < image_size.width )
                            {
                                const float that_value = edgeness(that_point[0], that_point[1]);

                                if(that_value > high_threshold)
                                {
                                    mFlags(i,j) |= FLAG_EDGE;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

#if 1
    {
        cv::Mat3b tmp = input_image.clone();
        for(int i=0; i<image_size.height; i++)
        {
            for(int j=0; j<image_size.width; j++)
            {
                if(mFlags(i,j) & FLAG_EDGE)
                {
                    tmp(i,j) = cv::Vec3b(0,255,0);
                }
                else
                {
                    cv::Vec3b& pix = tmp(i,j);
                    int gray = (pix[0]+pix[1]+pix[2])/3;
                    pix = cv::Vec3b(gray,gray,gray);
                }
            }
        }
        cv::imshow("rien", tmp);
        cv::waitKey(0);
    }
#endif
}

void TrackerCPUImpl::detectCirclesCentersWithHough(std::vector<cv::Vec2f>& centers)
{
    const float histogram_to_image = 4.0f;
    const int histogram_threshold = 20;
    const float maxima_radius = 4.0f;
    const int num_mean_shift_iterations = 2;

    const float margin = mMaxRadius + 5.0f;
    const int histogram_width = static_cast<int>( std::ceil(mFlags.cols + 2*static_cast<int>(margin)) / histogram_to_image );
    const int histogram_height = static_cast<int>( std::ceil(mFlags.rows + 2*static_cast<int>(margin)) / histogram_to_image );

    cv::Mat1i histogram(histogram_height, histogram_width);
    std::fill(histogram.begin(), histogram.end(), 0);

    centers.clear();

    for(int i=0; i<mFlags.rows; i++)
    {
        for(int j=0; j<mFlags.cols; j++)
        {
            if(mFlags(i,j) & FLAG_EDGE)
            {
                const cv::Point2i pt_from(
                    static_cast<int>( (margin + static_cast<float>(j) + 0.5 - mNormals(i,j)[0]*mMinRadius) / histogram_to_image ),
                    static_cast<int>( (margin + static_cast<float>(i) + 0.5 - mNormals(i,j)[1]*mMinRadius) / histogram_to_image ) );
                    
                const cv::Point2i pt_to(
                    static_cast<int>( (margin + static_cast<float>(j) + 0.5 - mNormals(i,j)[0]*mMaxRadius) / histogram_to_image ),
                    static_cast<int>( (margin + static_cast<float>(i) + 0.5 - mNormals(i,j)[1]*mMaxRadius) / histogram_to_image ) );
                    
                cv::LineIterator it(histogram, pt_from, pt_to);

                for(int k=0; k<it.count; k++)
                {
                    histogram(it.pos())++;
                    it++;
                }
            }
        }
    }

#if 1
    cv::imshow("rien", histogram*65535/(*std::max_element(histogram.begin(), histogram.end())));
    cv::waitKey(0);
#endif

    for(int i=0; i<histogram_height; i++)
    {
        for(int j=0; j<histogram_width; j++)
        {
            if(histogram(i,j) >= histogram_threshold)
            {
                bool ismax = true;
                for(const cv::Vec2i& delta : mNeighbors)
                {
                    const cv::Vec2i neigh = cv::Vec2i(i,j) + delta;

                    if( 0 <= neigh[0] && neigh[0] < histogram_height && 0 <= neigh[1] && neigh[1] < histogram_width )
                    {
                        ismax = ismax && (histogram(i,j) >= histogram(neigh));
                    }
                }

                if(ismax)
                {
                    cv::Vec2f center(i+0.5f, j+0.5f);

                    // one iteration of mean shift to refine center.

                    const int N = static_cast<int>(std::ceil(maxima_radius)) + 1;

                    for(int iter=0; iter<num_mean_shift_iterations; iter++)
                    {
                        cv::Vec2f accum(0.0f, 0.0f);
                        int mass = 0;

                        const int ii = static_cast<int>(center[0]);
                        const int jj = static_cast<int>(center[1]);

                        for(int di=-N; di<=N; di++)
                        {
                            for(int dj=-N; dj<=N; dj++)
                            {
                                const cv::Vec2i other(ii+di,jj+dj);

                                if( std::hypot(di,dj) < maxima_radius && 0 <= other[0] && other[0] < histogram_height && 0 <= other[1] && other[1] < histogram_width )
                                {
                                    accum[0] += static_cast<float>(histogram(other)) * (ii + 0.5f);
                                    accum[1] += static_cast<float>(histogram(other)) * (jj + 0.5f);
                                    mass += histogram(other);
                                }
                            }
                        }

                        if( mass > 0 )
                        {
                            center = accum * (1.0f/static_cast<float>(mass));
                        }
                    }

                    centers.emplace_back(
                        histogram_to_image*center[0] - margin,
                        histogram_to_image*center[1] - margin);

                    // set neighborhood to zero.

                    for(int di=-N; di<=N; di++)
                    {
                        for(int dj=-N; dj<=N; dj++)
                        {
                            const cv::Vec2i other(i+di,j+dj);

                            if( std::hypot(di,dj) < maxima_radius && 0 <= other[0] && other[0] < histogram_height && 0 <= other[1] && other[1] < histogram_width )
                            {
                                histogram(other) = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    std::cout << centers.size() << std::endl;
}

void TrackerCPUImpl::detectCirclesWithRANSAC(std::vector<cv::Vec3f>& circles)
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

    circles.clear();

    while(pixels_to_process.empty() == false)
    {
        std::uniform_int_distribution<int> distrib(0, pixels_to_process.size()-1);

        const int selected_index = distrib(mEngine);

        const cv::Vec2i seed = pixels_to_process[selected_index];
        pixels_to_process[selected_index] = pixels_to_process.back();
        pixels_to_process.pop_back();

        if( (mFlags(seed) & FLAG_NO_SEED) == 0 )
        {
            cv::Vec3f circle;

            if( findCircle(seed, circle) )
            {
                circles.push_back(circle);
            }
        }
    }
}

bool TrackerCPUImpl::findCircle(const cv::Vec2i& seed, cv::Vec3f& circle)
{
    const cv::Vec2f from(
        seed[1] + 0.5f - 2.0f*mNormals(seed)[0]*mMinRadius,
        seed[0] + 0.5f - 2.0f*mNormals(seed)[1]*mMinRadius );

    const cv::Vec2f to(
        seed[1] + 0.5f - 2.0f*mNormals(seed)[0]*mMaxRadius,
        seed[0] + 0.5f - 2.0f*mNormals(seed)[1]*mMaxRadius );

    const cv::Point2i from_int(
        static_cast<int>(from[0]),
        static_cast<int>(from[1]) );

    const cv::Point2i to_int(
        static_cast<int>(to[0]),
        static_cast<int>(to[1]) );

    cv::LineIterator line(mFlags, from_int, to_int);

    bool has_hit = false;
    cv::Point2i opposite;
    bool ret = false;

    for(int k=0; has_hit == false && k<line.count; k++)
    {
        has_hit = findEdgeInNeighborhood(line.pos(), 1, opposite);
        line++;
    }

    if( has_hit )
    {

        ret = true;

        if(ret)
        {
            const float dot_product = mNormals(seed).dot(mNormals(opposite));
            constexpr float threshold = -cos(10.0*M_PI/180.0);

            ret = (dot_product < threshold);
        }

        if(ret)
        {
            const float radius = 0.5f * std::hypot( opposite.x - seed[1], opposite.y - seed[0] );

            circle[0] = seed[1] + 0.5f - radius*mNormals(seed)[0];
            circle[1] = seed[0] + 0.5f - radius*mNormals(seed)[1];
            circle[2] = radius;
        }

        if(ret)
        {
            // TODO: check circle.
        }
    }

    return ret;
}

bool TrackerCPUImpl::findEdgeInNeighborhood(const cv::Point& center, int half_size, cv::Point& edge)
{
    bool ret = false;
    double best_dist = 0.0f;

    for(int di=-half_size; di<=half_size; di++)
    {
        for(int dj=-half_size; dj<=half_size; dj++)
        {
            const cv::Point2i other(center.x+dj, center.y+di);

            if( 0 <= other.x && other.x < mFlags.cols && 0 <= other.y && other.y < mFlags.rows )
            {
                if( mFlags(other) & FLAG_EDGE )
                {
                    const double dist = std::hypot( other.x - center.x, other.y - center.y );

                    if(ret == false || dist < best_dist)
                    {
                        best_dist = dist;
                        edge = other;
                        ret = true;
                    }
                }
            }
        }
    }

    return ret;
}

void TrackerCPUImpl::findCircleGrowingRegion(const cv::Vec2i& seed)
{
    std::vector<cv::Vec2i> patch;
    bool ok = true;

    auto pred_neighborood = [this,seed] (const cv::Vec2i& neighbor) -> bool
    {
        const float dist = std::hypot(
            float(seed[0]-neighbor[0]),
            float(seed[1]-neighbor[1]));

        const float radius = 15.0f;

        return (dist < radius);
    };

    auto pred_circle = [this,seed] (const cv::Vec2i& neighbor) -> bool
    {
        cv::Vec2f xs;
        xs[0] = static_cast<float>(seed[1]) + 0.5f;
        xs[1] = static_cast<float>(seed[0]) + 0.5f;

        cv::Vec2f xo;
        xo[0] = static_cast<float>(neighbor[1]) + 0.5f;
        xo[1] = static_cast<float>(neighbor[0]) + 0.5f;

        const cv::Vec2f ns = mNormals(seed);

        const cv::Vec2f no = mNormals(neighbor);

        const float ns_dot_no = ns.dot(no);

        static constexpr float threshold = cos(2.0*M_PI/180.0);

        bool ret = false;

        if( ns_dot_no > threshold )
        {
            const float vertical = ns.dot(xo-xs);
            const float horizontal = -ns[1]*(xo[0]-xs[0]) + ns[0]*(xo[1]-xs[1]);
            ret =
                -5.0f < vertical && vertical < 2.0f &&
                -4.0f < horizontal && horizontal < 4.0f;
        }
        else if( ns_dot_no < -threshold )
        {
        }
        else
        {
            Eigen::Matrix2d A;
            A << no[0], -ns[0], no[1], -ns[1];

            Eigen::Vector2d Y;
            Y << xs[0] - xo[0], xs[1] - xo[1];

            if( std::fabs(A.determinant()) > 1.0e-6 )
            {
                const Eigen::Vector2d X = A.inverse() * Y;

                const float ro = -X[0];
                const float rs = -X[1];
                std::cout << ro << " " << rs << std::endl;

                mMinRadius = 2.0f;
                mMaxRadius = 800.0f;
                if( mMinRadius < ro && ro < mMaxRadius && mMinRadius < rs && rs < mMaxRadius && std::fabs(ro-rs) < 5.0f )
                {
                    ret = true;
                }
            }
            std::cout << ret << std::endl;
        }

        return ret;
    };

    patch.push_back(seed);
    growPatch(patch, pred_circle);

    if( patch.size() >= 20 )
        debugShowPatch("result", patch);
}

void TrackerCPUImpl::debugShowPatch(const std::string& name, const std::vector<cv::Vec2i>& patch)
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
bool TrackerCPUImpl::growPrimitive(
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
void TrackerCPUImpl::growPatch(std::vector<cv::Vec2i>& patch, const T& pred)
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

