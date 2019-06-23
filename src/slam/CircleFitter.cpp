#include <iostream>
#include <opencv2/imgproc.hpp>
#include "CircleFitter.h"

CircleFitter::CircleFitter()
{
    mUseOpenCV = false;
    mMaxRadius = 100.0f;
    mMinRadius = 1.0f;
    mMaxIterations = 1000;
}

bool CircleFitter::fit(
    const std::vector<cv::Vec2f>& points, 
    bool use_initial_solution,
    cv::Vec3f& circle)
{
    bool ret = false;

    if(mUseOpenCV && points.size() >= 5)
    {
        const cv::RotatedRect ellipse_cv = cv::fitEllipse(points);

        const float epsilon = 1.0e-5;

        if( ellipse_cv.size.width > epsilon && ellipse_cv.size.height > epsilon )
        {
            const float err = std::fabs(ellipse_cv.size.width-ellipse_cv.size.height) / std::min(ellipse_cv.size.width, ellipse_cv.size.height);

            if( err < 0.40 )
            {
                const float radius = 0.5 * 0.5 * (ellipse_cv.size.width + ellipse_cv.size.height);

                if( mMinRadius <= radius && radius <= mMaxRadius )
                {
                    circle[0] = ellipse_cv.center.x;
                    circle[1] = ellipse_cv.center.y;
                    circle[2] = radius;
                    ret = true;
                }
            }
        }
    }
    else if(points.size() >= 3)
    {
        // Levenberg-Marquardt algorithm.

        bool go_on = true;
        bool first = true;
        int num_iterations = 0;

        double lambda = 1.0;
        const double factor = 2.0;

        double best_error = 0.0;
        Eigen::Vector3d best_solution;
        Eigen::MatrixXd best_J;
        Eigen::VectorXd best_F;

        Eigen::LDLT<Eigen::Matrix3d> solver;

        // compute initial solution.

        if(use_initial_solution)
        {
            best_solution.x() = circle[0];
            best_solution.y() = circle[1];
            best_solution.z() = circle[2];
        }
        else
        {
            // initialize with gravity center and average distance from gravity center.

            best_solution.x() = 0.0;
            best_solution.y() = 0.0;

            for(const cv::Vec2f& pt : points)
            {
                best_solution.x() += pt[0];
                best_solution.y() += pt[1];
            }

            best_solution.x() /= static_cast<double>(points.size());
            best_solution.y() /= static_cast<double>(points.size());

            best_solution.z() = 0.0;

            for(const cv::Vec2f& pt : points)
            {
                const double dist = std::hypot( pt[0] - best_solution.x(), pt[1] - best_solution.y() );

                best_solution.z() += dist;
            }

            best_solution.z() /= static_cast<double>(points.size());
        }

        // compute error with initial solution.

        best_error = computeError(points, best_solution);
        computeError(points, best_solution, best_F, best_J);

        while(go_on)
        {
            const Eigen::Vector3d B = - (best_J.transpose() * best_F);

            const Eigen::Matrix3d JtJ = best_J.transpose() * best_J;

            if( first )
            {
                lambda = 0.1*JtJ.trace()/3.0;
                first = false;
            }

            const Eigen::Matrix3d A = JtJ + lambda * Eigen::Matrix3d::Identity();
            solver.compute(A);

            const Eigen::Vector3d delta = solver.solve(B);

            const Eigen::Vector3d candidate_solution = best_solution + delta;

            const double candidate_error = computeError(points, candidate_solution);

            if( candidate_error < best_error )
            {
                best_error = candidate_error;
                best_solution = candidate_solution;

                computeError(points, candidate_solution, best_F, best_J);

                lambda /= factor;
            }
            else
            {
                lambda *= factor;
            }

            //std::cout << "[" << num_iterations << "] " << best_error << std::endl;

            num_iterations++;

            go_on = (best_error > 1.0e-5) && (lambda < 1.0e5) && (num_iterations < mMaxIterations) && (B.norm() > 1.0e-5);
        }

        circle[0] = static_cast<float>( best_solution.x() );
        circle[1] = static_cast<float>( best_solution.y() );
        circle[2] = static_cast<float>( best_solution.z() );

        // check that radius is OK.

        ret = (mMinRadius < circle[2] && circle[2] < mMaxRadius);
    }

    return ret;
}

void CircleFitter::setUseOpenCV(bool value)
{
    mUseOpenCV = value;
}

void CircleFitter::setMinMaxRadius(float min_radius, float max_radius)
{
    mMinRadius = min_radius;
    mMaxRadius = max_radius;
}

double CircleFitter::computeError(
    const std::vector<cv::Vec2f>& points,
    const Eigen::Vector3d& solution)
{
    double error = 0.0;

    for(const cv::Vec2f& pt : points)
    {
        const double diff = std::hypot( pt[0] - solution.x(), pt[1] - solution.y() ) - solution.z();
        error += diff*diff;
    }

    error /= static_cast<double>(points.size());

    return error;
}

void CircleFitter::computeError(
    const std::vector<cv::Vec2f>& points,
    const Eigen::Vector3d& solution,
    Eigen::VectorXd& F,
    Eigen::MatrixXd& J)
{
    F.resize(points.size());
    J.resize(points.size(), 3);

    F.setZero();
    J.setZero();

    for(int i=0; i<points.size(); i++)
    {
        const cv::Vec2f& pt = points[i];

        const double dx = pt[0] - solution.x();
        const double dy = pt[1] - solution.y();
        const double norm = std::hypot(dx,dy);
        const double err = norm - solution.z();

        const double epsilon = 1.0e-6;

        if(norm > epsilon)
        {
            F(i) = err;
            J(i,0) = -err * dx / norm;
            J(i,1) = -err * dy / norm;
            J(i,2) = -1.0;
        }
        else
        {
            F(i) = 0.0;
            J(i,0) = 0.0;
            J(i,1) = 0.0;
            J(i,2) = 0.0;
        }
    }
}

void CircleFitter::setMaxIterations(int max)
{
    mMaxIterations = max;
}

