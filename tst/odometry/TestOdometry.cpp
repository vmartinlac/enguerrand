#include <iostream>
#include <random>
#include "TestOdometry.h"

void TestOdometry::initTestCase()
{
    // setup parameters.

    myLandmarkRadius = 1.0;

    // setup landmarks.

    myLandmarks.resize(4);
    myLandmarks[0] << 10.0, 0.0, 50.0;
    myLandmarks[1] << -10.0, 0.0, 50.0;
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
    myCalibration->num_cameras = 1;
    myCalibration->cameras[0].image_size.width = 1920;
    myCalibration->cameras[0].image_size.height = 1080;
    myCalibration->cameras[0].calibration_matrix = { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
    myCalibration->cameras[0].inverse_calibration_matrix = { 1.0/fx, 0.0, -cx/fx, 0.0, 1.0/fy, -cy/fy, 0.0, 0.0, 1.0 };

    // setup reference trajectory, timestamps and circles.

    const size_t N = 2000;
    myCameraToWorldTrajectory.resize(N);
    myTimestamps.resize(N);
    myCircles.resize(N);
    for(size_t i=0; i<N; i++)
    {
        const double kappa = double(i)/double(N-1);
        const double theta0 = 2.0*M_PI*kappa;
        const double alpha = 5.0;
        const double gamma = -8.0 * kappa*(kappa-1.0)/0.25;
        const Eigen::Vector3d camera_to_world_t
        {
            alpha*std::cos(theta0),
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
            // TODO: set circle.
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

        const bool ok = code->track(
            myTimestamps[i],
            myCircles[i],
            estimated_camera_to_world,
            aligned);

        QVERIFY(ok);
        QVERIFY(aligned);

        const Sophus::SE3d tmp = myCameraToWorldTrajectory[i] * estimated_camera_to_world.inverse();
        const Sophus::SE3d::Tangent err = tmp.log();
        const double err_translation = err.tail<3>().norm();
        std::cout << "[ " << i << " ] translation error: " << err_translation << std::endl;

        // TODO: check error.
    }
}

QTEST_MAIN(TestOdometry)
//#include "odometry.moc"

