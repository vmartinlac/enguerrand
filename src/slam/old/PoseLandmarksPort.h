
#pragma once

#include <opencv2/core.hpp>
#include <array>
#include <vector>
#include <sophus/se3.hpp>
#include "VideoFrame.h"
#include "PipelinePort.h"

struct PoseLandmarksPortLandmark
{
    cv::Vec3d position_in_world;
    cv::Matx33d position_covariance;
};

class PoseLandmarksPort : public PipelinePort
{
public:

    PoseLandmarksPort();

    void reset() override;

public:

    bool available;
    bool registered_wrt_previous_frame;
    Sophus::SE3d object_to_world;
    std::vector<PoseLandmarksPortLandmark> landmarks;
};

