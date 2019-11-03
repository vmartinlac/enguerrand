#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include "CalibrationData.h"

CameraCalibrationData::CameraCalibrationData()
{
}

CalibrationData::CalibrationData()
{
}

bool CalibrationData::loadCamerasFromFile(const std::string& path)
{
    bool ok = true;

    if(path.empty() == false)
    {
        QJsonDocument doc;
        QFile file(path.c_str());

        if(ok)
        {
            ok = file.open(QFile::ReadOnly);
        }

        if(ok)
        {
            QByteArray data = file.readAll();
            doc = QJsonDocument::fromJson(data);
            ok = (doc.isNull() == false) && doc.isArray();
        }

        if(ok)
        {
            QJsonArray array = doc.array();
            cameras.resize(array.size());

            for(int i=0; ok && i<array.size(); i++)
            {
                ok = array[i].isObject();

                if(ok)
                {
                    CameraCalibrationData& cam = cameras[i];
                    QJsonObject obj = array[i].toObject();

                    cam.image_size.width = obj["image_width"].toDouble();
                    cam.image_size.height = obj["image_height"].toDouble();

                    cam.calibration_matrix <<
                        obj["fx"].toDouble(), 0.0, obj["cx"].toDouble(),
                        0.0, obj["fy"].toDouble(), obj["cy"].toDouble(),
                        0.0, 0.0, 1.0;

                    cam.inverse_calibration_matrix <<
                        1.0/cam.calibration_matrix(0,0), 0.0, -cam.calibration_matrix(0,2)/cam.calibration_matrix(0,0),
                        0.0, 1.0/cam.calibration_matrix(1,1), -cam.calibration_matrix(1,2)/cam.calibration_matrix(1,1),
                        0.0, 0.0, 1.0;

                    cam.distortion_coefficients.resize(5);
                    cam.distortion_coefficients[0] = obj["k1"].toDouble();
                    cam.distortion_coefficients[1] = obj["k2"].toDouble();
                    cam.distortion_coefficients[2] = obj["p1"].toDouble();
                    cam.distortion_coefficients[3] = obj["p2"].toDouble();
                    cam.distortion_coefficients[4] = obj["k3"].toDouble();

                    ok = obj["camera_to_robot"].isArray();

                    if(ok)
                    {
                        QJsonArray arr = obj["camera_to_robot"].toArray();
                        ok = (arr.size() == 3*4);

                        if(ok)
                        {
                            Eigen::Matrix4d H;
                            H.setIdentity();

                            for(int i=0; i<3; i++)
                            {
                                for(int j=0; j<4; j++)
                                {
                                    H(i,j) = arr[4*i+j].toDouble();
                                }
                            }

                            cam.camera_to_robot = Sophus::SE3d(H);
                        }
                    }
                }
            }
        }
    }

    return ok;
}

void CalibrationData::dump() const
{
    std::cout << "== Calibration data ==" << std::endl;

    int cam_id = 0;
    for(const CameraCalibrationData& cam : cameras)
    {
        std::cout << "Camera " << cam_id << std::endl;
        std::cout << "   image_width = " << cam.image_size.width << std::endl;
        std::cout << "   image_height = " << cam.image_size.height << std::endl;
        std::cout << "   fx = " << cam.calibration_matrix(0,0) << std::endl;
        std::cout << "   fy = " << cam.calibration_matrix(1,1) << std::endl;
        std::cout << "   cx = " << cam.calibration_matrix(0,2) << std::endl;
        std::cout << "   cy = " << cam.calibration_matrix(1,2) << std::endl;
        std::cout << "   distortion_coefficients = { ";
        for(double value : cam.distortion_coefficients)
        {
            std::cout << value << " ";
        }
        std::cout << "}" << std::endl;
        std::cout << "   camera_to_robot_t = " << cam.camera_to_robot.translation().transpose() << std::endl;
        std::cout << "   camera_to_robot_q = " << cam.camera_to_robot.unit_quaternion().coeffs().transpose() << std::endl;

        cam_id++;
    }
}

