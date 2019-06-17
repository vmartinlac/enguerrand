
#pragma once

namespace slam
{
    struct RectificationOutput
    {
        cv::Mat3b rectified_image;
    };

    struct EdgeDetectionOutput
    {
        cv::Mat1b flags;
        cv::Mat2f normals;
    };

    struct BallsDetectionOutput
    {
        struct Ball
        {
            cv::Point2f center;
            float radius;

            bool has_previous;
            size_t previous;
        };

        std::vector<Ball> balls;
    };
}

