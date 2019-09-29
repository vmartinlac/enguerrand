
#pragma once

#include <memory>
#include <random>
#include "Histogram.h"
#include "TrackedCircle.h"
//#include "svm.h"

class CirclesTracker
{
public:

    CirclesTracker();

    void track(
        const cv::Mat3b& input_image,
        const cv::Mat1b& edges,
        const cv::Mat2f& normals,
        std::vector<TrackedCircle>& circles);

    void setReferenceHistogram(HistogramPtr histogram);

    void setMinRadius(float x);

    void setMaxRadius(float x);

protected:

    /**
    * \brief First stage of detection-tracking.
    */
    bool detect(
        const cv::Mat1b& edges,
        const cv::Mat2f& normals,
        std::vector<TrackedCircle>& circles);

    /**
    * \brief Second stage of detection-tracking.
    */
    bool filter(
        const cv::Mat3b& input_image,
        std::vector<TrackedCircle>& circles);

    /**
    * \brief Third stage of detection-tracking.
    */
    bool track(
        std::vector<TrackedCircle>& circles);

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

    bool filterCircle(const cv::Mat3b& image, const cv::Vec3f& circle);

protected:

    cv::Mat1b mFlags;
    cv::Mat1i mLastDetectionMap;
    std::default_random_engine mEngine;
    std::array<cv::Vec2i,8> mNeighbors;
    float mMinRadius;
    float mMaxRadius;
    HistogramPtr mReferenceHistogram;
    bool mExportThumbnails;
    size_t mThumbnailsCount;
    bool mExportDetectionPicture;
    double mHistogramIntersectionThreshold;
};

using CirclesTrackerPtr = std::shared_ptr<CirclesTracker>;

