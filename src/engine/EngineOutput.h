
#pragma once

#include <memory>
#include <chrono>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <QMetaType>

struct EngineOutputCircle
{
    cv::Vec3f circle;
};

struct EngineOutputLandmark
{
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
};

struct EngineOutputFrame
{
    double timestamp;
    Sophus::SE3d camera_to_world;
    Eigen::Matrix<double,6,6> pose_covariance;
    std::vector<EngineOutputCircle> circles;
};

struct EngineOutput
{
    std::chrono::microseconds frame_runtime;
    cv::Mat3b input_image;
    cv::Mat1b edges_image;
    cv::Mat3b traces_image;
    cv::Mat3b detection_image;
    std::vector<EngineOutputLandmark> landmarks;
    EngineOutputFrame current_frame;
};

using EngineOutputPtr = std::shared_ptr<EngineOutput>;

Q_DECLARE_METATYPE(EngineOutputPtr)

