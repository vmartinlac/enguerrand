#pragma once

#include <QtTest>
#include <CalibrationData.h>
#include <OdometryCode.h>

class TestStaticOdometry : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase();

    void cleanupTestCase();

    void testPFOdometry();

private:

    void testOdometry(OdometryCodePtr code);

private:

    std::vector<cv::Vec3f> myCircles;
    CalibrationDataPtr myCalibration;
};
