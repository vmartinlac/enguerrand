#pragma once

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <vector>

class CircleFitter
{
public:

    CircleFitter();

    void setUseOpenCV(bool);

    void setMinMaxRadius(float min_radius, float max_radius);

    void setMaxIterations(int max);

    bool fit(
        const std::vector<cv::Vec2f>& points, 
        bool use_initial_solution,
        cv::Vec3f& circle);

protected:

    double computeError(
        const std::vector<cv::Vec2f>& points,
        const Eigen::Vector3d& solution);

    void computeError(
        const std::vector<cv::Vec2f>& points,
        const Eigen::Vector3d& solution,
        Eigen::VectorXd& F,
        Eigen::MatrixXd& J);

protected:

    bool mUseOpenCV;
    float mMinRadius;
    float mMaxRadius;
    int mMaxIterations;
};

