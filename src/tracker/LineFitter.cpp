#include <opencv2/imgproc.hpp>
#include "LineFitter.h"

LineFitter::LineFitter()
{
    mUseOpenCV = false;
}

bool LineFitter::fit(
    const std::vector<cv::Vec2f>& points, 
    bool use_initial_solution,
    cv::Vec3f& line)
{
    bool ret = false;

    if(points.size() >= 2)
    {
        if( mUseOpenCV)
        {
            cv::Vec4f line_cv;
            cv::fitLine(points, line_cv, cv::DIST_L2, 0.0, 0.1, 0.01);

            line[0] = -line_cv[1];
            line[1] = line_cv[0];
            line[2] = -(line_cv[3]*line_cv[0] - line_cv[1]*line_cv[2]);

            ret = true;
        }
        else
        {
            // compute mean.

            cv::Vec2f mean(0.0f, 0.0f);
            for(const cv::Vec2f& pt : points)
            {
                mean += pt;
            }
            mean /= static_cast<float>(points.size());

            // compute covariance matrix.

            float sumxx = 0.0f;
            float sumxy = 0.0f;
            float sumyy = 0.0f;
            for(const cv::Vec2f& pt : points)
            {
                const cv::Vec2f delta = pt - mean;

                sumxx += delta[0]*delta[0];
                sumxy += delta[0]*delta[1];
                sumyy += delta[1]*delta[1];
            }
            sumxx /= static_cast<float>(points.size());
            sumxy /= static_cast<float>(points.size());
            sumyy /= static_cast<float>(points.size());

            // compute normal.

            const float a = 0.5*(sumxx - sumyy);
            const float b = sumxy;

            const float epsilon = 1.0e-5;

            if(a*a + b*b > epsilon)
            {
                const float theta = 0.5 * std::atan2(b,a);

                const cv::Vec2f normal( sin(theta), -cos(theta) );

                line[0] = normal[0];
                line[1] = normal[1];
                line[2] = -normal.dot(mean);

                ret = true;
            }
        }
    }

    return ret;
}

void LineFitter::setUseOpenCV(bool value)
{
    mUseOpenCV = value;
}

