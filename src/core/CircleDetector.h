
#pragma once

#include <random>
#include "ObservationValidator.h"

class CircleDetector
{
public:

    CircleDetector();

    void detect(
        const cv::Mat3b& input_image,
        const cv::Mat1b& edges,
        const cv::Mat2f& normals,
        std::vector<cv::Vec3f>& circles);

    void setObservationValidator(ObservationValidatorPtr validator);

    void setMinRadius(float x);

    void setMaxRadius(float x);

protected:

    bool detect(
        const cv::Mat1b& edges,
        const cv::Mat2f& normals,
        std::vector<cv::Vec3f>& circles);

    bool filter(
        const cv::Mat3b& input_image,
        std::vector<cv::Vec3f>& circles);

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
    std::default_random_engine mEngine;
    std::array<cv::Vec2i,8> mNeighbors;
    float mMinRadius;
    float mMaxRadius;
    ObservationValidatorPtr mObservationValidator;
};

