
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
    cv::Vec3f position;
};

struct EngineOutput
{
    EngineOutput(
        size_t frame_id_,
        double timestamp_,
        const cv::Mat3b& input_image_) :

        frame_id(frame_id_),
        timestamp(timestamp_),
        input_image(input_image_)
    {
    }

    std::chrono::microseconds frame_runtime;
    size_t frame_id;
    double timestamp;
    const cv::Mat3b input_image;
    //const cv::Mat3b circles_image;
    //const cv::Mat3b detection_image;
    std::vector<EngineOutputCircle> circles;
    std::vector<EngineOutputLandmark> landmarks;
    Sophus::SE3d camera_to_world;
};

using EngineOutputPtr = QSharedPointer<EngineOutput>;

Q_DECLARE_METATYPE(EngineOutputPtr)

