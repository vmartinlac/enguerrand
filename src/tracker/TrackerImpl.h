#pragma once

#include <random>
#include "LineFitter.h"
#include "CircleFitter.h"
#include "Tracker.h"

class TrackerImpl : public Tracker
{
public:

    TrackerImpl();

    void track(const cv::Mat& image, std::vector<TrackedLandmark>& result) override;

protected:

    enum PointFlag
    {
        FLAG_EDGE=1,
        FLAG_VISITED=2,
        FLAG_NO_SEED=4,
        FLAG_CIRCLE=8
    };

protected:

    void detectEdges(const cv::Mat& image);
    void findCircles();
    void findCircle(const cv::Vec2i& seed);
    void debugShowPatch(const std::string& name, const std::vector<cv::Vec2i>& patch);

    template<typename T>
    void growPatch(
        std::vector<cv::Vec2i>& patch,
        const T& pred);

    template<typename PrimitiveType, typename EstimatorType, typename ClassifierType>
    bool growPrimitive(
        std::vector<cv::Vec2i>& patch,
        const EstimatorType& estimator,
        const ClassifierType& classifier,
        PrimitiveType& result);

protected:

    std::array<cv::Vec2i,8> mNeighbors;
    std::default_random_engine mEngine;
    LineFitter mLineFitter;
    CircleFitter mCircleFitter;
    cv::Mat1b mFlags;
};

