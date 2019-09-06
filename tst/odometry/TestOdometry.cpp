#include <Eigen/Eigen>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <random>
#include "TestOdometry.h"

//#define TESTODOMETRY_EXPORT_IMAGES

void TestOdometry::initTestCase()
{
    // setup parameters.

    myLandmarkRadius = 1.0;

    // setup landmarks.

    myLandmarks.resize(4);
    myLandmarks[0] << 10.0, 0.0, 70.0;
    myLandmarks[1] << -10.0, 0.0, 70.0;
    myLandmarks[2] << 0.0, 10.0, 50.0;
    myLandmarks[3] << 0.0, -10.0, 50.0;

    // setup calibration.

    const double fx = 1.80993344e+03;
    const double fy = 1.80745383e+03;
    const double cx = 9.20081895e+02;
    const double cy = 5.39890210e+02;

    /*
    0.05806194,
    0.07819791,
    0.0009545,
    -0.00109695,
    -0.61395187
    */

    myCalibration.reset(new CalibrationData());
    myCalibration->cameras.resize(1);
    myCalibration->cameras[0].image_size.width = 1920;
    myCalibration->cameras[0].image_size.height = 1080;
    myCalibration->cameras[0].calibration_matrix = { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
    myCalibration->cameras[0].inverse_calibration_matrix = { 1.0/fx, 0.0, -cx/fx, 0.0, 1.0/fy, -cy/fy, 0.0, 0.0, 1.0 };

    // setup reference trajectory, timestamps and circles.

    const size_t N = 500;
    myCameraToWorldTrajectory.resize(N);
    myTimestamps.resize(N);
    myCircles.resize(N);
    for(size_t i=0; i<N; i++)
    {
        const double kappa = double(i)/double(N-1);
        const double theta0 = 2.0*M_PI*kappa;
        const double alpha = 5.0;
        const double gamma = -80.0 * kappa*(1.0-kappa)/0.25;

        const Eigen::Vector3d camera_to_world_t
        {
            alpha*( std::cos(theta0) - 1),
            alpha*std::sin(theta0),
            gamma
        };

        const double beta = kappa * 2.0*M_PI;

        const Eigen::Quaterniond camera_to_world_q(Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitZ()));

        myCameraToWorldTrajectory[i] = Sophus::SE3d(camera_to_world_q, camera_to_world_t);

        myTimestamps[i] = 10.0*kappa;

        myCircles[i].resize(myLandmarks.size());

        for(size_t j=0; j<myLandmarks.size(); j++)
        {
            const Eigen::Vector3d landmark_in_camera = myCameraToWorldTrajectory[i].inverse() * myLandmarks[j];

            QVERIFY( landmark_in_camera.z() > myLandmarkRadius*0.1 );

            const double alpha = std::asin( myLandmarkRadius/landmark_in_camera.norm() );

            const Eigen::Vector3d los = landmark_in_camera / landmark_in_camera.z();

            const double betax = std::acos(los.x());
            const double betay = std::acos(los.y());

            Eigen::Matrix<double,4,3> tangent_los;
            tangent_los <<
                std::cos(betax - alpha), los.y(), 1.0,
                std::cos(betax + alpha), los.y(), 1.0,
                los.x(), std::cos(betay - alpha), 1.0,
                los.x(), std::cos(betay + alpha), 1.0;

            Eigen::Matrix3d K;
            cv::cv2eigen(myCalibration->cameras[0].calibration_matrix, K);

            const Eigen::Matrix<double, 3, 4> tangent_points = K * tangent_los.transpose();

            const Eigen::Vector2d circle_center = 0.25 * ( tangent_points.block<2,1>(0,0) + tangent_points.block<2,1>(0,1) + tangent_points.block<2,1>(0,2) + tangent_points.block<2,1>(0,3) );

            const double circle_radius = 0.25 * ( std::fabs( tangent_points(0,1) - tangent_points(0,0) ) + std::fabs( tangent_points(1,3) - tangent_points(1,2) ) );

            myCircles[i][j].circle = cv::Vec3f( circle_center.x(), circle_center.y(), circle_radius );
            myCircles[i][j].has_previous = (i>0);
            myCircles[i][j].previous = j;
        }
    }
}

void TestOdometry::cleanupTestCase()
{
}

void TestOdometry::testEKFOdometry()
{
    OdometryCodePtr code(new EKFOdometry(myCalibration));
    testOdometry(code);
}

void TestOdometry::testBAOdometry()
{
    OdometryCodePtr code(new BAOdometry(myCalibration));
    testOdometry(code);
}

void TestOdometry::testOdometry(OdometryCodePtr code)
{
    const size_t N = myCameraToWorldTrajectory.size();

    for(size_t i=0; i<N; i++)
    {
        Sophus::SE3d estimated_camera_to_world;
        bool aligned;

#ifdef TESTODOMETRY_EXPORT_IMAGES
        cv::Mat1b image(myCalibration->cameras[0].image_size);
        std::fill(image.begin(), image.end(), 0);
        for(const TrackedCircle& c : myCircles[i])
        {
            cv::circle(image, cv::Point(c.circle[0], c.circle[1]), c.circle[2], 255, -1);
        }
        std::stringstream s;
        s << "rien_" << i << ".png";
        cv::imwrite(s.str(), image);
#endif

        const bool ok = code->track(
            myTimestamps[i],
            myCircles[i],
            estimated_camera_to_world,
            aligned);

        QVERIFY(ok);
        QVERIFY(i == 0 || aligned);

        //std::cout << "G-T = " << myCameraToWorldTrajectory[i].translation().transpose() << std::endl;
        //std::cout << "Est = " << estimated_camera_to_world.translation().transpose() << std::endl;

        const Sophus::SE3d::Tangent err = ( myCameraToWorldTrajectory[i].inverse() * estimated_camera_to_world ).log();
        std::cout << "[ " << i << " ] translation error: " << err.head<3>().transpose() << std::endl;
        std::cout << "[ " << i << " ] rotation error (deg): " << err.tail<3>().transpose()*180.0/M_PI << std::endl;

        // TODO: check error.
    }
}

QTEST_MAIN(TestOdometry)
//#include "odometry.moc"

