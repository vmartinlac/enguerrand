#include <iostream>
#include <random>
#include <QtTest>
#include "LineFitter.h"
#include "CircleFitter.h"

class TestFitting : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase();

    void cleanupTestCase();

    void testLine();

    void testCircle();

protected:

    std::default_random_engine mEngine;
};

