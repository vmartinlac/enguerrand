
#pragma once

#include <random>
#include "EdgeCirclesData.h"
#include "TrackedCircle.h"

class CirclesTracker
{
public:

    CirclesTracker();

    void track(
        const cv::Mat3b& input_image,
        const cv::Mat1b& edges,
        const cv::Mat2f& normals,
        std::vector<TrackedCircle>& circles);

    void setMinRadius(float x);

    void setMaxRadius(float x);

protected:

    bool findCircle(
        const cv::Mat2f& normals,
        const cv::Point2i& seed,
        cv::Vec3f& circle);

    bool findEdgeInNeighborhood(
        const cv::Point2i& center,
        int half_size, cv::Point2i& edge);

    template<typename T>
    void growPatch(
        std::vector<cv::Point2i>& patch,
        const T& pred);

    template<typename PrimitiveType, typename EstimatorType, typename ClassifierType>
    bool growPrimitive(
        std::vector<cv::Point2i>& patch,
        const EstimatorType& estimator,
        const ClassifierType& classifier,
        PrimitiveType& result);

protected:

    cv::Mat1b mFlags;
    cv::Mat1i mLastDetectionMap;
    std::default_random_engine mEngine;
    std::array<cv::Vec2i,8> mNeighbors;
    float mMinRadius;
    float mMaxRadius;
};

