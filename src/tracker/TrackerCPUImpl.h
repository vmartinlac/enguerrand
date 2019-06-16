
#pragma once

#include <random>
#include "Tracker.h"

class TrackerCPUImpl : public Tracker
{
public:

    TrackerCPUImpl();

    void track(const cv::Mat& image, std::vector<TrackedLandmark>& result) override;

protected:

    enum PointFlag
    {
        FLAG_NONZERO_GRADIENT=1,
        FLAG_MAXIMUM_ALONG_GRADIENT=2,
        FLAG_EDGE=4,
        FLAG_VISITED=8,
        FLAG_NO_SEED=16,
        FLAG_CIRCLE=32
    };

protected:

    void detectEdges(const cv::Mat& image);
    void detectCirclesCentersWithHough(std::vector<cv::Vec2f>& centers);

    void detectCirclesWithRANSAC(std::vector<cv::Vec3f>& circles);

    bool findEdgeInNeighborhood(const cv::Point& center, int half_size, cv::Point& edge);

    void findCircleGrowingRegion(const cv::Vec2i& seed);
    bool findCircle(const cv::Vec2i& seed, cv::Vec3f& circle);
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

    float mMinRadius;
    float mMaxRadius;
    std::array<cv::Vec2i,8> mNeighbors;
    std::default_random_engine mEngine;
    cv::Mat1b mFlags;
    cv::Mat2f mNormals;
};

