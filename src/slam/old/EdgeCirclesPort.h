
#pragma once

#include <opencv2/core.hpp>
#include "PipelinePort.h"

#define EDGECIRCLESPORT_NONZERO_GRADIENT 1
#define EDGECIRCLESPORT_MAXIMUM_ALONG_GRADIENT 2
#define EDGECIRCLESPORT_EDGE 4
#define EDGECIRCLESPORT_VISITED 8
#define EDGECIRCLESPORT_NO_SEED 16
#define EDGECIRCLESPORT_CIRCLE 32

struct EdgeCirclesPortCircle
{
    cv::Vec3f circle;
    bool has_previous;
    size_t previous;
};

class EdgeCirclesPort : public PipelinePort
{
public:

    EdgeCirclesPort();

    void reset() override;

public:

    bool available;
    cv::Mat1b flags;
    cv::Mat2f normals;
    std::vector<EdgeCirclesPortCircle> circles;
};

