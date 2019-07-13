#pragma once

#include <opencv2/core.hpp>
#include <vector>

#define EDGECIRCLE_NONZERO_GRADIENT 1
#define EDGECIRCLE_MAXIMUM_ALONG_GRADIENT 2
#define EDGECIRCLE_EDGE 4
#define EDGECIRCLE_VISITED 8
#define EDGECIRCLE_NO_SEED 16
#define EDGECIRCLE_CIRCLE 32

struct EdgeCirclesDataCircle
{
    cv::Vec3f circle;
    bool has_previous;
    size_t previous;
};

class EdgeCirclesData
{
public:

    cv::Mat1b flags;
    cv::Mat2f normals;
    std::vector<EdgeCirclesDataCircle> circles;
};

