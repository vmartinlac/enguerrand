#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "EdgeCirclesPort.h"
#include "EdgeDetectionCPU.h"
#include "VideoPort.h"

EdgeDetectionCPU::EdgeDetectionCPU()
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

size_t EdgeDetectionCPU::getNumPorts() const
{
    return 2;
}

bool EdgeDetectionCPU::initialize()
{
    return true;
}

void EdgeDetectionCPU::finalize()
{
}

void EdgeDetectionCPU::compute(PipelinePort** ports)
{
    VideoPort* video = static_cast<VideoPort*>(ports[0]);
    EdgeCirclesPort* edge = static_cast<EdgeCirclesPort*>(ports[1]);

    cv::Mat input_image;
    cv::Size image_size;

    cv::Mat1b gray;
    cv::Mat1b blurred;
    cv::Mat1f sobel_x;
    cv::Mat1f sobel_y;
    cv::Mat1f edgeness;

    cv::Mat1b flags;
    cv::Mat2f normals;

    bool ok = true;

    edge->available = false;

    if(ok)
    {
        ok = ( video->frame.isValid() && video->frame.getNumViews() == 1 );
    }

    if(ok)
    {
        //std::cout << "Edge detection on frame " << video->frame.getId() << std::endl;
        input_image = video->frame.getView();
        ok = ( input_image.type() == CV_8UC3 );
    }

    if(ok)
    {
        image_size = input_image.size();
        edgeness.create(image_size);
        flags.create( image_size );
        normals.create( image_size );
    }

    // convert to gray.

    if(ok)
    {
        gray.create(image_size);
        const int from_to[2] = {0, 0};
        cv::mixChannels(&input_image, 1, &gray, 1, from_to, 1);
    }

    // smooth the image.

    if(ok)
    {
        cv::GaussianBlur(gray, gray, cv::Size(), 3.0);
        //cv::imshow("rien", gray);
        //cv::waitKey(0);
    }

    // compute sobel x-derivative and y-derivative.

    if(ok)
    {
        cv::Sobel(gray, sobel_x, CV_32F, 1, 0, 5);
        cv::Sobel(gray, sobel_y, CV_32F, 0, 1, 5);
    }

    // compute norm of gradient.

    if(ok)
    {
        const int margin = 3;

        for(int i=0; i<image_size.height; i++)
        {
            for(int j=0; j<image_size.width; j++)
            {
                flags(i,j) = 0;
                normals(i,j)[0] = 0.0f;
                normals(i,j)[1] = 0.0f;

                if( margin <= i && i+margin < image_size.height && margin <= j && j+margin < image_size.width )
                {
                    const float gradient_norm = std::hypot(sobel_x(i,j), sobel_y(i,j));

                    edgeness(i,j) = gradient_norm;

                    const float epsilon = 1.0e-5;

                    if( gradient_norm > epsilon )
                    {
                        flags(i,j) = EDGECIRCLESPORT_NONZERO_GRADIENT;
                        normals(i,j)[0] = sobel_x(i,j) / gradient_norm;
                        normals(i,j)[1] = sobel_y(i,j) / gradient_norm;
                    }
                }
            }
        }
    }

    // non-maximum suppression.

    if(ok)
    {
        for(int i=0; i<image_size.height; i++)
        {
            for(int j=0; j<image_size.width; j++)
            {
                if( flags(i,j) & EDGECIRCLESPORT_NONZERO_GRADIENT )
                {
                    const cv::Vec2f N = normals(i,j);

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
                            flags(i,j) |= EDGECIRCLESPORT_MAXIMUM_ALONG_GRADIENT;
                        }
                    }
                }
            }
        }
    }

    // binarization.

    if(ok)
    {
        const float high_threshold = 760.0f;
        const float low_threshold = 0.7f*high_threshold; //200.0f;

        for(int i=0; i<image_size.height; i++)
        {
            for(int j=0; j<image_size.width; j++)
            {
                if( flags(i,j) & EDGECIRCLESPORT_MAXIMUM_ALONG_GRADIENT )
                {
                    const float this_value = edgeness(i,j);

                    if( this_value >= high_threshold )
                    {
                        flags(i,j) |= EDGECIRCLESPORT_EDGE;
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
                                    flags(i,j) |= EDGECIRCLESPORT_EDGE;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /*
    {
        cv::Mat1b tmp(image_size);
        auto proc = [] (uint8_t f) { return (f & EDGECIRCLESPORT_EDGE) ? 255 : 0; };
        std::transform(flags.begin(), flags.end(), tmp.begin(), proc);
        cv::imshow("rien", tmp);
        cv::waitKey(1);
    }
    */

    // export result.

    if(ok)
    {
        edge->available = true;
        edge->flags = std::move(flags);
        edge->normals = std::move(normals);
    }
}

