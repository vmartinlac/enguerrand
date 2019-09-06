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

bool CalibrationData::loadFromFile(const std::string& path)
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
                        ok = (arr.size() == 16);

                        if(ok)
                        {
                            Eigen::Matrix4d H;
                            H.setIdentity();

                            for(int i=0; i<4; i++)
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

