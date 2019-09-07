
#pragma once

#include <chrono>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <QImage>
#include <QSharedPointer>

struct EngineOutputCircle
{
    cv::Vec3f circle;
};

struct EngineOutputLandmark
{
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
};

struct EngineOutputKeyFrame
{
    size_t frame_id;
    double timestamp;
    Sophus::SE3d camera_to_world;
    std::vector<EngineOutputCircle> circles;
    Eigen::Matrix<double,6,6> covariance;
};

struct EngineOutput
{
    std::chrono::microseconds frame_runtime;
    cv::Mat3b input_image;
    cv::Mat1b edges_image;
    cv::Mat3b traces_image;
    std::vector<EngineOutputLandmark> landmarks;
    std::vector<EngineOutputKeyFrame> keyframes;
    //Sophus::SE3d camera_to_world;
};

using EngineOutputPtr = QSharedPointer<EngineOutput>;

Q_DECLARE_METATYPE(EngineOutputPtr)

