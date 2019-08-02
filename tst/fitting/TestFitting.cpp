#include "TestFitting.h"

void TestFitting::initTestCase()
{
}

void TestFitting::cleanupTestCase()
{
}

void TestFitting::testLine()
{
    LineFitter f;
    std::vector<cv::Vec2f> points;

    const int num_trials = 30;
    const int num_points = 100;
    std::uniform_real_distribution<float> theta_distrib(0.0f, M_PI);
    std::uniform_real_distribution<float> point_distrib(-100.0f, 100.0f);
    std::normal_distribution<float> noise_distrib(0.0, 1.5f);
    const float length = 60.0f;

    for(int i=0; i<num_trials; i++)
    {
        const cv::Vec2f origin(point_distrib(mEngine), point_distrib(mEngine));
        const float theta = theta_distrib(mEngine);
        const cv::Vec2f normal(std::cos(theta), std::sin(theta));

        points.clear();
        for(int i=0; i<num_points; i++)
        {
            const double lambda = -0.5*length + length*float(i)/float(num_points-1);
            const cv::Vec2f noise( noise_distrib(mEngine), noise_distrib(mEngine) );
            const cv::Vec2f pt = origin + lambda * cv::Vec2f(-normal[1], normal[0]) + noise;
            points.push_back(pt);
        }

        cv::Vec3f line;
        f.setUseOpenCV( bool(i%2) );
        QVERIFY( f.fit(points, false, line) );

        cv::Vec3f reference_line;
        reference_line[0] = normal[0];
        reference_line[1] = normal[1];
        reference_line[2] = -normal.dot(origin);

        if( line[0]*normal[0] + line[1]*normal[1] < 0.0f )
        {
            reference_line = -reference_line;
        }

        std::cout << "Line fitting trial #" << i+1 << std::endl;
        std::cout << "Reference: " << reference_line[0] << ' ' << reference_line[1] << ' ' << reference_line[2] << std::endl;
        std::cout << "Estimated: " << line[0] << ' ' << line[1] << ' ' << line[2] << std::endl;

        const double err_theta = std::acos( line[0]*normal[0] + line[1]*normal[1] );
        const double err_rho = reference_line[2] - line[2];

        QVERIFY( err_theta < 5.0f );
        QVERIFY( err_rho < 3.0f );
    }
}

void TestFitting::testCircle()
{
    CircleFitter f;
    f.setUseOpenCV(false);
    f.setMinMaxRadius(1.0f, 100.0f);

    std::uniform_real_distribution<float> position_distrib(-100.0f, 100.0f);
    std::uniform_real_distribution<float> radius_distrib(70.0f, 80.0f);
    std::normal_distribution<float> noise_distrib(0.0f, 1.0f);
    const int num_trials = 30;
    const int num_points = 300;

    for(int trial=0; trial<num_trials; trial++)
    {
        const cv::Vec2f center( position_distrib(mEngine), position_distrib(mEngine) );
        const float radius = radius_distrib(mEngine);

        std::vector<cv::Vec2f> points;

        for(int i=0; i<num_points; i++)
        {
            const double theta = 2.0*M_PI*float(i)/float(num_points-1);
            const cv::Vec2f noise( noise_distrib(mEngine), noise_distrib(mEngine) );
            const cv::Vec2f pt = center + radius * cv::Vec2f( std::cos(theta), std::sin(theta) ) + noise;
            points.push_back(pt);
        }

        cv::Vec3f circle(0.0f, 0.0f, 1.0f);
        QVERIFY( f.fit(points, false, circle) );

        const double err_center = std::hypot(circle[0]-center[0], circle[1]-center[1]);
        const double err_radius = circle[2] - radius;

        std::cout << "Circle fitting trial #" << trial+1 << std::endl;
        std::cout << "Reference: " << center[0] << ' ' << center[1] << ' ' << radius << std::endl;
        std::cout << "Estimated: " << circle[0] << ' ' << circle[1] << ' ' << circle[2] << std::endl;

        QVERIFY( err_center < 4.0f );
        QVERIFY( std::fabs(err_radius) < 4.0f );
    }
}

QTEST_MAIN(TestFitting)

