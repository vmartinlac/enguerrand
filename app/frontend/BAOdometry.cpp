#include <ceres/ceres.h>
#include "BAOdometry.h"

struct BAOdometry::LandmarkTriangulation
{
    template<typename T>
    bool operator()(const T* const parameters, T* residuals)
    {
        return false;
    }
};

