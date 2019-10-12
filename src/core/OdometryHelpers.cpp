#include <opencv2/imgproc.hpp>
#include "OdometryHelpers.h"

bool OdometryHelpers::triangulateLandmark(
    const cv::Vec3f& undistorted_circle,
    const CalibrationDataPtr calibration,
    double landmark_radius,
    Eigen::Vector3d& landmark)
{
    bool ret = false;

    const double cx = undistorted_circle[0];
    const double cy = undistorted_circle[1];
    const double r = undistorted_circle[2];

    const double IK00 = calibration->cameras[0].inverse_calibration_matrix(0,0);
    const double IK02 = calibration->cameras[0].inverse_calibration_matrix(0,2);
    const double IK11 = calibration->cameras[0].inverse_calibration_matrix(1,1);
    const double IK12 = calibration->cameras[0].inverse_calibration_matrix(1,2);

    const double los_cx = IK00*cx + IK02;
    const double los_cy = IK11*cy + IK12;
    const double los_cxminus = IK00*(cx-r) + IK02;
    const double los_cxplus = IK00*(cx+r) + IK02;
    const double los_cyminus = IK11*(cy-r) + IK12;
    const double los_cyplus = IK11*(cy+r) + IK12;

    const double alpha_xminus = std::atan(los_cxminus);
    const double alpha_xplus = std::atan(los_cxplus);
    const double alpha_yminus = std::atan(los_cyminus);
    const double alpha_yplus = std::atan(los_cyplus);

    const double los_dirx = std::tan( (alpha_xminus + alpha_xplus) / 2.0 );
    const double los_diry = std::tan( (alpha_yminus + alpha_yplus) / 2.0 );

    const double beta = ( (alpha_xplus - alpha_xminus)/2.0 + (alpha_yplus - alpha_yminus)/2.0 ) / 2.0;

    if( M_PI*0.3/180.0 < beta && beta < M_PI*150.0/180.0 )
    {
        const double distance = landmark_radius/std::sin(beta);

        Eigen::Vector3d dir;
        dir.x() = los_dirx;
        dir.y() = los_diry;
        dir.z() = 1.0;

        landmark = distance * dir.normalized();

        ret = true;
    }

    return ret;
}

cv::Vec3f OdometryHelpers::undistortCircle(const cv::Vec3f& circle, const CalibrationDataPtr calibration)
{
    const cv::Vec3f& c = circle;

    //cv::Matx<cv::Vec2d,4,1> distorted;
    //cv::Matx<cv::Vec2d,4,1> undistorted;

    // TODO: do not use dynamic arrays.
    std::vector<cv::Vec2d> distorted(4);
    std::vector<cv::Vec2d> undistorted(4);

    // require newer version of OpenCV.
    //std::array< cv::Vec2d, 4 > distorted;
    //std::array< cv::Vec2d, 4 > undistorted;

    distorted[0][0] = c[0]+c[2];
    distorted[0][1] = c[1];
    distorted[1][0] = c[0]-c[2];
    distorted[1][1] = c[1];
    distorted[2][0] = c[0];
    distorted[2][1] = c[1]+c[2];
    distorted[3][0] = c[0];
    distorted[3][1] = c[1]-c[2];

    cv::undistortPoints(
        distorted,
        undistorted,
        calibration->cameras[0].calibration_matrix,
        calibration->cameras[0].distortion_coefficients,
        cv::noArray(),
        calibration->cameras[0].calibration_matrix);

    const cv::Vec2d center = 0.25f * ( undistorted[0] + undistorted[1] + undistorted[2] + undistorted[3] );
    //center[0] = 0.25f * ( undistorted(0,0) + undistorted(1,0) + undistorted(2,0) + undistorted(3,0) );
    //center[1] = 0.25f * ( undistorted(0,1) + undistorted(1,1) + undistorted(2,1) + undistorted(3,1) );

    /*
    const double l0 = std::hypot(center[0]-undistorted(0,0), center[1]-undistorted(0,1));
    const double l1 = std::hypot(center[0]-undistorted(1,0), center[1]-undistorted(1,1));
    const double l2 = std::hypot(center[0]-undistorted(2,0), center[1]-undistorted(2,1));
    const double l3 = std::hypot(center[0]-undistorted(3,0), center[1]-undistorted(3,1));
    */

    const double l0 = cv::norm(center-undistorted[0]);
    const double l1 = cv::norm(center-undistorted[1]);
    const double l2 = cv::norm(center-undistorted[2]);
    const double l3 = cv::norm(center-undistorted[3]);

    cv::Vec3f ret;
    ret[0] = center[0];
    ret[1] = center[1];
    ret[2] = ( l0+l1+l2+l3 ) / 4.0;

    return ret;
}

