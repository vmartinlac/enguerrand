#include <iostream>
#include <queue>
#include <opencv2/imgproc.hpp>
#include "CircleDetector.h"

//#define CIRCLESTRACKER_DEBUG
//#define CIRCLESTRACKER_SAVE_PREVALIDATION

#ifdef CIRCLESTRACKER_SAVE_PREVALIDATION
#include <iomanip>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#endif


#define CIRCLESTRACKER_EDGE 1
#define CIRCLESTRACKER_VISITED 2
#define CIRCLESTRACKER_NO_SEED 4
#define CIRCLESTRACKER_CIRCLE 8

CircleDetector::CircleDetector()
{
    mMinRadius = 5.0f;
    mMaxRadius = 600.0f;

    mNeighbors[0] = cv::Vec2i(-1,-1);
    mNeighbors[1] = cv::Vec2i(-1,0);
    mNeighbors[2] = cv::Vec2i(-1,1);
    mNeighbors[3] = cv::Vec2i(0,-1);
    mNeighbors[4] = cv::Vec2i(0,1);
    mNeighbors[5] = cv::Vec2i(1,-1);
    mNeighbors[6] = cv::Vec2i(1,0);
    mNeighbors[7] = cv::Vec2i(1,1);
}

void CircleDetector::setObservationValidator(ObservationValidatorPtr validator)
{
    mObservationValidator = validator;
}

void CircleDetector::setMinRadius(float x)
{
    mMinRadius = x;
}

void CircleDetector::setMaxRadius(float x)
{
    mMaxRadius = x;
}

void CircleDetector::detect(
    const cv::Mat3b& input_image,
    const cv::Mat1b& edges,
    const cv::Mat2f& normals,
    std::vector<cv::Vec3f>& circles)
{
    bool ok = true;

    if( input_image.size() != edges.size() ) throw std::runtime_error("internal error");
    if( input_image.size() != normals.size() ) throw std::runtime_error("internal error");

    if(ok)
    {
        ok = detect(edges, normals, circles);
    }

    //std::cout << "   Num circles after detection: " << circles.size() << std::endl;

    if(ok)
    {
        ok = filter(input_image, circles);
    }

    //std::cout << "   Num circles after filtering: " << circles.size() << std::endl;

    if(ok == false)
    {
        circles.clear();
    }

    //std::cout << "   Success: " << ok << std::endl;

#if CIRCLESTRACKER_DEBUG
    {
        cv::Mat output = input_image.clone();
        for(cv::Vec3f& c : circles)
        {
            cv::circle(output, cv::Point2f(c[0], c[1]), c[2], cv::Scalar(255, 255, 255));
        }

        cv::resize( output, output, cv::Size(), 0.5, 0.5);

        cv::imwrite("detected_circles.png", output);
    }
#endif
}

bool CircleDetector::detect(const cv::Mat1b& edges, const cv::Mat2f& normals, std::vector<cv::Vec3f>& circles)
{
    std::vector<cv::Point2i> pixels_to_process;

    const cv::Size image_size = edges.size();

    mFlags.create(image_size);
    pixels_to_process.reserve(image_size.width*image_size.height/10);

    for(int i=0; i<image_size.height; i++)
    {
        for(int j=0; j<image_size.width; j++)
        {
            if(edges(i,j))
            {
                pixels_to_process.push_back(cv::Point2i(j,i));
                mFlags(i,j) = CIRCLESTRACKER_EDGE;
            }
            else
            {
                mFlags(i,j) = 0;
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

        if( (mFlags(seed) & CIRCLESTRACKER_NO_SEED) == 0 )
        {
            cv::Vec3f circle;

            const bool found = findCircle(
                normals,
                seed,
                circle);

            if(found)
            {
                circles.push_back( circle );
            }
        }
    }

    return true;
}

bool CircleDetector::filter(const cv::Mat3b& input_image, std::vector<cv::Vec3f>& circles)
{
    if( mObservationValidator )
    {
        std::vector<cv::Vec3f>::iterator it = circles.begin();

        while(it != circles.end())
        {
            const bool keep = mObservationValidator->validate(input_image, *it);

            if(keep)
            {
                it++;
            }
            else
            {
                *it = circles.back();
                circles.pop_back();
            }
        }
    }

    return true;
}

/*
#ifdef CIRCLESTRACKER_SAVE_PREVALIDATION
void CircleDetector::saveDetection(const std::vector<cv::Vec3f>& circles)
{
    for(const cv::Vec3f& circle : circles)
    {
        const int margin = 20;
        const cv::Rect ROI(circle[0]-margin-circle[2], circle[1]-margin-circle[2], 2*margin+2*circle[2], 2*margin+2*circle[2]);
        const cv::Rect image_rect(0,0,image.cols,image.rows);
        if( (image_rect | ROI) == image_rect )
        {
            static std::ofstream file;
            static bool initialized = false;
            static int i = 0;
            if(initialized == false)
            {
                file = std::move(std::ofstream("listing.txt"));
                initialized = true;
            }

            std::stringstream ss;
            ss << std::setfill('0') << std::setw(6) << i++;
            const std::string id = ss.str();

            file << id << " " << circle[0]-ROI.x << " " << circle[1]-ROI.y << " " << circle[2] << std::endl << std::flush;
            cv::imwrite(id + "_image.png", image(ROI));
        }
    }
}
#endif
*/

bool CircleDetector::findCircle(
    const cv::Mat2f& normals,
    const cv::Point2i& seed,
    cv::Vec3f& circle)
{
    const cv::Point2f from(
        seed.x + 0.5f - 2.0f*normals(seed)[0]*mMinRadius,
        seed.y + 0.5f - 2.0f*normals(seed)[1]*mMinRadius );

    const cv::Point2f to(
        seed.x + 0.5f - 2.0f*normals(seed)[0]*mMaxRadius,
        seed.y + 0.5f - 2.0f*normals(seed)[1]*mMaxRadius );

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
        //std::cout << "hello" << std::endl;
        constexpr int N_points = 4;
        std::array<cv::Point2i,N_points> found_points;
        std::vector<cv::Point2i> patch;

        ret = true;

        if(ret)
        {
            const float dot_product = normals(seed).dot(normals(opposite));
            //constexpr float threshold = -cos(10.0*M_PI/180.0);
            constexpr float threshold = -0.984807753012208;

            ret = (dot_product < threshold);
        }

        if(ret)
        {
            const float radius = 0.5f * std::hypot( opposite.x - seed.x, opposite.y - seed.y );

            circle[0] = seed.x + 0.5f - radius*normals(seed)[0];
            circle[1] = seed.y + 0.5f - radius*normals(seed)[1];
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
                    const cv::Vec2f normal = normals(found_points[i]);
                    const float scalar_product = normal[0]*cos_beta + normal[1]*sin_beta;
                    //constexpr float threshold = std::cos(30.0*M_PI/180.0);
                    constexpr float threshold = 0.8660254037844387;
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

        /*
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
                    const cv::Vec2f normal = normals(pt);
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

                #if 0
                cv::Mat1b rien(mFlags.size());
                std::fill(rien.begin(), rien.end(), 0);
                for(cv::Point2i pt : patch)
                    rien(pt) = 255;
                cv::imshow("rien",rien);
                cv::waitKey(0);
                #endif
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
        */

        if(ret)
        {
            const float clear_radius = circle[2] + 10.0f;
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
                        mFlags(pt) |= CIRCLESTRACKER_NO_SEED;
                    }
                }
            }
        }
    }

    return ret;
}

bool CircleDetector::findEdgeInNeighborhood(
    const cv::Point& center,
    int half_size,
    cv::Point& edge)
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
                if( mFlags(other) & CIRCLESTRACKER_EDGE )
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

template<typename PrimitiveType, typename EstimatorType, typename ClassifierType>
bool CircleDetector::growPrimitive(
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

template<typename T>
void CircleDetector::growPatch(
    std::vector<cv::Point2i>& patch,
    const T& pred)
{
    // TODO: maybe set these variable as class members to avoid many memory allocations.
    std::vector<cv::Point2i> visited;
    std::queue<cv::Point2i> queue;

    for(cv::Point2i& pt : patch)
    {
        mFlags(pt) |= CIRCLESTRACKER_VISITED;
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

                if( (f & CIRCLESTRACKER_EDGE) != 0 && (f & CIRCLESTRACKER_VISITED) == 0 )
                {
                    if( pred(neighbor) )
                    {
                        mFlags(neighbor) |= CIRCLESTRACKER_VISITED;
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
        mFlags(pt) &= ~CIRCLESTRACKER_VISITED;
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

