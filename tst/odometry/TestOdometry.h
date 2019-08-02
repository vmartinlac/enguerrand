
#pragma once

#include <QtTest>
#include "EKFOdometry.h"
#include "BAOdometry.h"

class TestOdometry : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase();

    void cleanupTestCase();

    void testEKFOdometry();

    void testBAOdometry();

private:

    void testOdometry(OdometryCodePtr code);

private:

    double myLandmarkRadius;
    CalibrationDataPtr myCalibration;
    std::vector<Eigen::Vector3d> myLandmarks;
    std::vector<Sophus::SE3d> myCameraToWorldTrajectory;
    std::vector<double> myTimestamps;
    std::vector< std::vector<TrackedCircle> > myCircles;
};
