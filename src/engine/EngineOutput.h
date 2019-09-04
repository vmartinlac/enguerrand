
#pragma once

#include <opencv2/core.hpp>
#include <QSharedPointer>

struct EngineOutput
{
    size_t frame_id;
    cv::Mat image;
};

using EngineOutputPtr = QSharedPointer<EngineOutput>;

