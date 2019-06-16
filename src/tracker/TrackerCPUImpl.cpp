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
    cv::waitKey(1);
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

#if 0
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

void TrackerCPUImpl::detectCirclesWithRANSAC(std::vector<cv::Vec3f>& circles)
{
    const cv::Size image_size = mFlags.size();

    std::vector<cv::Point2i> pixels_to_process;
    
    pixels_to_process.reserve(image_size.width*image_size.height);

    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            if(mFlags(i,j) & FLAG_EDGE)
            {
                pixels_to_process.push_back(cv::Point2i(j,i));
            }
        }
    }

    circles.clear();

    while(pixels_to_process.empty() == false)
    {
        std::uniform_int_distribution<int> distrib(0, pixels_to_process.size()-1);

        const int selected_index = distrib(mEngine);

        const cv::Point2i seed = pixels_to_process[selected_index];
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

bool TrackerCPUImpl::findCircle(const cv::Point2i& seed, cv::Vec3f& circle)
{
    const cv::Point2f from(
        seed.x + 0.5f - 2.0f*mNormals(seed)[0]*mMinRadius,
        seed.y + 0.5f - 2.0f*mNormals(seed)[1]*mMinRadius );

    const cv::Point2f to(
        seed.x + 0.5f - 2.0f*mNormals(seed)[0]*mMaxRadius,
        seed.y + 0.5f - 2.0f*mNormals(seed)[1]*mMaxRadius );

    const cv::Point2i from_int = static_cast<cv::Point2i>(from);

    const cv::Point2i to_int = static_cast<cv::Point2i>(to);

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
        constexpr int N_points = 4;
        std::array<cv::Point2i,N_points> found_points;
        std::vector<cv::Point2i> patch;

        ret = true;

        if(ret)
        {
            const float dot_product = mNormals(seed).dot(mNormals(opposite));
            constexpr float threshold = -cos(10.0*M_PI/180.0);

            ret = (dot_product < threshold);
        }

        if(ret)
        {
            const float radius = 0.5f * std::hypot( opposite.x - seed.x, opposite.y - seed.y );

            circle[0] = seed.x + 0.5f - radius*mNormals(seed)[0];
            circle[1] = seed.y + 0.5f - radius*mNormals(seed)[1];
            circle[2] = radius;
        }

        if(ret)
        {
            std::uniform_real_distribution<float> angular_offset_distribution(0.0, 2.0*M_PI);

            const float alpha = angular_offset_distribution(mEngine);
            const float cos_alpha = std::cos(alpha);
            const float sin_alpha = std::sin(alpha);

            for(int i=0; ret && i<N_points; i++)
            {
                const float beta = alpha + 2.0*M_PI*float(i)/float(N_points);
                const float cos_beta = std::cos(beta);
                const float sin_beta = std::sin(beta);

                cv::Vec2f target;
                target[0] = circle[0] + cos_beta * circle[2];
                target[1] = circle[1] + sin_beta * circle[2];

                cv::Point2i target_point;
                target_point.x = static_cast<int>(target[0]);
                target_point.y = static_cast<int>(target[1]);

                ret = findEdgeInNeighborhood(target_point, 3, found_points[i]);

                if(ret)
                {
                    const cv::Vec2f normal = mNormals(found_points[i]);
                    const float scalar_product = normal[0]*cos_beta + normal[1]*sin_beta;
                    constexpr float threshold = std::cos(30.0*M_PI/180.0);
                    ret = scalar_product > threshold;
                }
            }
        }

        if(ret)
        {
            // re-estimate the circle from found_points.

            cv::Vec2f center(0,0);
            for(cv::Point2i pt : found_points)
            {
                center[0] += pt.x + 0.5f;
                center[1] += pt.y + 0.5f;
            }
            center[0] /= float(N_points);
            center[1] /= float(N_points);

            float radius = 0.0f;
            for(cv::Point2i pt : found_points)
            {
                radius += std::hypot( pt.x - center[0], pt.y - center[1] );
            }
            radius /= float(N_points);

            circle[0] = center[0];
            circle[1] = center[1];
            circle[2] = radius;
        }

        // optionnally perform mean square minimization.

        if(true)
        {
            if(ret)
            {
                patch.resize(N_points);
                std::copy(found_points.begin(), found_points.end(), patch.begin());

                auto pred = [this,&circle] (const cv::Point2i& pt) -> bool
                {
                    bool ret = false;

                    const float x = pt.x + 0.5f;
                    const float y = pt.y + 0.5f;
                    const cv::Vec2f normal = mNormals(pt);
                    const float dx = x - circle[0];
                    const float dy = y - circle[1];
                    const float dist = std::hypot(dx,dy);

                    if(dist > 1.0e-5)
                    {
                        const float cos_angle = dx*normal[0]/dist + dy*normal[1]/dist;

                        ret =
                            std::fabs(dist-circle[2]) < 3.0f &&
                            cos_angle >= std::cos(30.0*M_PI/180.0);
                    }

                    return ret;
                };

                growPatch(patch, pred);

                ret = (patch.size() >= 20);

                /*
                cv::Mat1b rien(mFlags.size());
                std::fill(rien.begin(), rien.end(), 0);
                for(cv::Point2i pt : patch)
                    rien(pt) = 255;
                cv::imshow("rien",rien);
                cv::waitKey(0);
                */
            }

            if(ret)
            {
                std::vector<cv::Vec2f> data(patch.size());

                auto pred = [] (const cv::Point2i& pt) -> cv::Vec2f
                {
                    return cv::Vec2f( pt.x + 0.5f, pt.y + 0.5f );
                };

                std::transform(patch.begin(), patch.end(), data.begin(), pred);

                CircleFitter f;
                f.setMinMaxRadius(mMinRadius, mMaxRadius);

                ret = f.fit(data, true, circle);
            }
        }

        if(ret)
        {
            const float clear_radius = circle[2] + 5.0f;
            const int N = static_cast<int>(std::ceil(clear_radius));

            cv::Point2i the_center;
            the_center.x = static_cast<int>(circle[0]);
            the_center.y = static_cast<int>(circle[1]);

            for(int di=-N; di<=N; di++)
            {
                for(int dj=-N; dj<=N; dj++)
                {
                    cv::Point2i pt;
                    pt.x = the_center.x + dj;
                    pt.y = the_center.y + di;

                    const float dx = pt.x + 0.5f - circle[0];
                    const float dy = pt.y + 0.5f - circle[1];
                    const float dist = std::hypot(dx,dy);

                    if( 0 <= pt.x && pt.x < mFlags.cols && 0 <= pt.y && pt.y < mFlags.rows && dist < clear_radius )
                    {
                        mFlags(pt) |= FLAG_NO_SEED;
                    }
                }
            }
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

/*
template<typename PrimitiveType, typename EstimatorType, typename ClassifierType>
bool TrackerCPUImpl::growPrimitive(
    std::vector<cv::Point2i>& patch,
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
*/

template<typename T>
void TrackerCPUImpl::growPatch(std::vector<cv::Point2i>& patch, const T& pred)
{
    // TODO: set these variable as class members to avoid many memory allocations.
    std::vector<cv::Point2i> visited;
    std::queue<cv::Point2i> queue;

    for(cv::Point2i& pt : patch)
    {
        mFlags(pt) |= FLAG_VISITED;
        visited.push_back(pt);
        queue.push(pt);
    }

    while( queue.empty() == false )
    {
        const cv::Point2i point = queue.front();
        queue.pop();

        for(int k=0; k<mNeighbors.size(); k++)
        {
            cv::Point2i neighbor = point;
            neighbor.x += mNeighbors[k][1];
            neighbor.y += mNeighbors[k][0];

            if( 0 <= neighbor.x && neighbor.x < mFlags.cols && 0 <= neighbor.y && neighbor.y < mFlags.rows )
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

    for( const cv::Point2i& pt : visited )
    {
        mFlags(pt) &= ~FLAG_VISITED;
    }
}

/*
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
*/

