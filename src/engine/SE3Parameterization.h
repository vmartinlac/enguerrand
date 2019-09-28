
#pragma once

#include <ceres/local_parameterization.h>
#include <sophus/se3.hpp>

class SE3Parameterization : public ceres::LocalParameterization
{
public:

    bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const override;

    bool ComputeJacobian(const double* x_raw, double* jacobian_raw) const override;

    int GlobalSize() const override;

    int LocalSize() const override;
};
