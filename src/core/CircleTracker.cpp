#include <iostream>
#include <queue>
#include <opencv2/imgproc.hpp>
#include "CircleTracker.h"

CircleTracker::CircleTracker()
{
}

void CircleTracker::track(const cv::Size& image_size, const std::vector<cv::Vec3f>& input, std::vector<TrackedCircle>& output)
{
    std::vector<bool> to_remove(input.size(), false);
    cv::Mat1i new_detection_map(image_size);

    // create new detection map.

    {
        std::fill( new_detection_map.begin(), new_detection_map.end(), -1);

        int id = 0;

        for(const cv::Vec3f& c : input)
        {
            cv::Point center;
            center.x = c[0];
            center.y = c[1];

            const double radius = c[2];

            const int x0 = std::max<int>(0, std::floor(center.x - radius));
            const int x1 = std::min<int>(image_size.width-1, std::ceil(center.x + radius));
            const int y0 = std::max<int>(0, std::floor(center.y - radius));
            const int y1 = std::min<int>(image_size.height-1, std::ceil(center.y + radius));

            for(int i=y0; i<=y1; i++)
            {
                for(int j=x0; j<=x1; j++)
                {
                    const double candidate_radius = std::hypot( j+0.5 - center.x, i+0.5-center.y );
                    if( candidate_radius < radius )
                    {
                        int& value = new_detection_map(i,j);

                        if( value >= 0 )
                        {
                            to_remove[id] = true;
                            to_remove[value] = true;
                            //std::cout << "   REMOVED TWO OVERLAPPING CIRCLES!" << std::endl;
                        }
                        else
                        {
                            value = id;
                        }
                    }
                }
            }

            id++;
        }
    }

    mIntermediateOutput.resize(input.size());
    for(size_t i=0; i<input.size(); i++)
    {
        mIntermediateOutput[i].circle = input[i];
        mIntermediateOutput[i].has_previous = false;
        mIntermediateOutput[i].previous = 0;
    }

    //cv::imshow("rien", new_detection_map*65535/(id-1));
    //cv::waitKey(0);

    // if there is a previous detection map, use it to track.

    if( mLastDetectionMap.data )
    {
        if( mLastDetectionMap.size() != image_size )
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        auto it_last = mLastDetectionMap.begin();
        auto it_new = new_detection_map.begin();

        while( it_last != mLastDetectionMap.end() )
        {
            if( *it_new >= 0 && to_remove[*it_new] == false )
            {
                if( *it_last >= 0 )
                {
                    if( mIntermediateOutput[*it_new].has_previous == false )
                    {
                        mIntermediateOutput[*it_new].has_previous = true;
                        mIntermediateOutput[*it_new].previous = *it_last;
                    }
                    else if( *it_last != mIntermediateOutput[*it_new].previous )
                    {
                        //std::cout << "   REMOVED ONE CIRCLE BECAUSE AMBIGUOUS TRACK!" << std::endl;
                        to_remove[*it_new] = true;
                    }
                }
            }

            it_last++;
            it_new++;
        }
    }

    // remove eliminated circles.

    {
        std::vector<size_t> new_id(mIntermediateOutput.size());

        output.clear();

        for(size_t i=0; i<mIntermediateOutput.size(); i++)
        {
            if(to_remove[i])
            {
                new_id[i] = 0;
            }
            else
            {
                new_id[i] = output.size();
                output.push_back(mIntermediateOutput[i]);
            }
        }

        for(int& x : new_detection_map)
        {
            if(x >= 0)
            {
                if(to_remove[x])
                {
                    x = -1;
                }
                else
                {
                    x = new_id[x];
                }
            }
        }
    }

    // update detection map.

    mLastDetectionMap = std::move(new_detection_map);
}

void CircleTracker::reset()
{
    mLastDetectionMap.release();
}

