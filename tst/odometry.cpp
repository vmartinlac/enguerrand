#include <iostream>
#include <random>
#include <QtTest>
#include "EKFOdometry.h"

class TestOdometry : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
    }

    void cleanupTestCase()
    {
    }

    void testEKFOdometry()
    {
        // TODO
    }
};

QTEST_MAIN(TestOdometry)
#include "odometry.moc"

