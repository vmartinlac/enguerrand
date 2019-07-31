#pragma once

#include <opencv2/core.hpp>
#include <vector>

class LineFitter
{
public:

    LineFitter();

    void setUseOpenCV(bool);

    bool fit(
        const std::vector<cv::Vec2f>& points, 
        bool use_initial_solution,
        cv::Vec3f& line);

protected:

    bool mUseOpenCV;
};

