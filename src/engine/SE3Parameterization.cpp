#include "SE3Parameterization.h"

bool SE3Parameterization::Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
{
    Eigen::Map<Sophus::SE3d const> const x(x_raw);
    Eigen::Map<Sophus::SE3d::Tangent const> const delta(delta_raw);
    Eigen::Map<Sophus::SE3d> x_plus_delta(x_plus_delta_raw);

    x_plus_delta = x * Sophus::SE3d::exp(delta);

    return true;
}

bool SE3Parameterization::ComputeJacobian(const double* x_raw, double* jacobian_raw) const
{
    Eigen::Map< Sophus::SE3d const > x(x_raw);
    Eigen::Map< Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > jacobian(jacobian_raw);
    jacobian = x.Dx_this_mul_exp_x_at_0();
    return true;
}

int SE3Parameterization::GlobalSize() const
{
    return Sophus::SE3d::num_parameters;
}

int SE3Parameterization::LocalSize() const
{
    return Sophus::SE3d::DoF;
}

