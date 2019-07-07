
#pragma once

#include <random>
#include "EdgeCirclesData.h"

class CirclesDetection
{
public:

    CirclesDetection();

    void detect(const cv::Mat3b& input_image, EdgeCirclesData& ecdata);

protected:

    bool findCircle(
        const cv::Mat2f& normals,
        const cv::Point2i& seed,
        cv::Mat1b& flags,
        cv::Vec3f& circle);

    bool findEdgeInNeighborhood(
        const cv::Mat1b& flags,
        const cv::Point2i& center,
        int half_size, cv::Point2i& edge);

    template<typename T>
    void growPatch(
        cv::Mat1b& flags,
        std::vector<cv::Point2i>& patch,
        const T& pred);

    template<typename PrimitiveType, typename EstimatorType, typename ClassifierType>
    bool growPrimitive(
        cv::Mat1b& flags,
        std::vector<cv::Point2i>& patch,
        const EstimatorType& estimator,
        const ClassifierType& classifier,
        PrimitiveType& result);

protected:

    std::default_random_engine mEngine;
    std::array<cv::Vec2i,8> mNeighbors;
    float mMinRadius;
    float mMaxRadius;
};

