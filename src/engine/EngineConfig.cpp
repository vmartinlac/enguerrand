#include <QJsonObject>
#include <QJsonDocument>
#include <QFile>
#include "EKFOdometry.h"
#include "FileVideoSource.h"
#include "ObservationValidatorSimple.h"
#include "EngineConfig.h"

EngineConfig::EngineConfig()
{
    //debug = false;
}

bool EngineConfig::loadFromFile(const std::string& path)
{
    QFile file(path.c_str());
    QJsonDocument doc;
    QJsonObject root;

    bool ret = true;
    const char* err = "";

    if(ret)
    {
        ret = file.open(QIODevice::ReadOnly);
        err = "Could not open config file!";
    }

    if(ret)
    {
        doc = QJsonDocument::fromJson(file.readAll());
        ret = doc.isObject();
    }

    if(file.isOpen())
    {
        file.close();
    }

    if(ret)
    {
        root = doc.object();
    }

    // set video_input.

    if(ret)
    {
        FileVideoSourcePtr file_video_source(new FileVideoSource());
        file_video_source->setFileName( root["video_input"].toObject()["path"].toString().toStdString() );
        video_input = file_video_source;
        err = "Could not set video input!";
    }

    // set balls_histogram.

    if(ret)
    {
        ObservationValidatorSimplePtr validator = std::make_shared<ObservationValidatorSimple>();
        observation_validator = validator;
        ret = validator->load( root["balls_histogram"].toString().toStdString() );
        err = "Could not initialize observation validator!";
    }

    // set calibration.

    if(ret)
    {
        QJsonObject obj = root["camera_calibration"].toObject();

        calibration.reset(new CalibrationData());

        calibration->cameras.resize(1);

        calibration->cameras[0].image_size.width = obj["image_width"].toDouble();
        calibration->cameras[0].image_size.height = obj["image_height"].toDouble();

        const double cam_fx = obj["fx"].toDouble();
        const double cam_fy = obj["fy"].toDouble();
        const double cam_cx = obj["cx"].toDouble();
        const double cam_cy = obj["cy"].toDouble();
        const double cam_k1 = obj["k1"].toDouble();
        const double cam_k2 = obj["k2"].toDouble();
        const double cam_p1 = obj["p1"].toDouble();
        const double cam_p2 = obj["p2"].toDouble();
        const double cam_k3 = obj["k3"].toDouble();

        calibration->cameras[0].calibration_matrix = { cam_fx, 0.0, cam_cx, 0.0, cam_fy, cam_cy, 0.0, 0.0, 1.0 };
        calibration->cameras[0].inverse_calibration_matrix = { 1.0/cam_fx, 0.0, -cam_cx/cam_fx, 0.0, 1.0/cam_fy, -cam_cy/cam_fy, 0.0, 0.0, 1.0 };
        calibration->cameras[0].distortion_coefficients.assign({ cam_k1, cam_k2, cam_p1, cam_p2, cam_k3 });
    }

    // set odometry_code.

    if(ret)
    {
        odometry_code.reset(new EKFOdometry(calibration));
        err = "Could not set odometry code!";
    }

    // set debug.

    /*
    if(ret)
    {
        debug = root["debug"].toBool(false);
    }
    */

    // if fail, reset.

    if(ret == false)
    {
        std::cerr << err << std::endl;
        clear();
    }

    /*
    Eigen::Matrix3d A;
    Eigen::Matrix3d B;
    cv::cv2eigen(calibration->cameras[0].calibration_matrix, A);
    cv::cv2eigen(calibration->cameras[0].inverse_calibration_matrix, B);
    std::cout << A << std::endl;
    std::cout << B*A << std::endl;
    exit(0);
    */

    return ret;
}

void EngineConfig::clear()
{
    video_input.reset();
    odometry_code.reset();
}

