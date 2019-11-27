#include <Eigen/Eigen>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <random>
#include <PFOdometry.h>
#include "Test.h"

//#define TESTODOMETRY_EXPORT_IMAGES

void TestStaticOdometry::initTestCase()
{
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

    myCircles.emplace_back(100.0f, 100.0f, 15.0f);
    myCircles.emplace_back(300.0f, 100.0f, 15.0f);
    myCircles.emplace_back(100.0f, 300.0f, 15.0f);
    myCircles.emplace_back(300.0f, 300.0f, 15.0f);
}

void TestStaticOdometry::cleanupTestCase()
{
}

void TestStaticOdometry::testPFOdometry()
{
    OdometryCodePtr code(new PFOdometry(myCalibration));
    testOdometry(code);
}

void TestStaticOdometry::testOdometry(OdometryCodePtr code)
{
    const size_t N = 10;

    for(size_t frame=0; frame<N; frame++)
    {
        std::cout << "= FRAME " << frame << " =" << std::endl;
        std::vector<TrackedCircle> circles(myCircles.size());
        for(size_t i=0; i<myCircles.size(); i++)
        {
            circles[i].circle = myCircles[i];
            circles[i].has_previous = (frame > 0);
            circles[i].previous = (frame > 0) ? i : 0;
        }

        const double timestamp = double(frame)*1.0/33.0;

        OdometryFrame odoframe;
        const bool ok = code->track(timestamp, circles, odoframe);

        QVERIFY(ok);
        QVERIFY(frame == 0 || odoframe.aligned_wrt_previous);
    }
}

QTEST_MAIN(TestStaticOdometry)
//#include "odometry.moc"

